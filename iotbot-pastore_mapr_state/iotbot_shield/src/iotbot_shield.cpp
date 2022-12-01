#include "iotbot_shield.hpp"


namespace iotbot {


Shield::Shield(rclcpp::Logger logger) :
    rosLogger_(logger),
    isPortOpened_(false),
    killThread_(false),
    dataIsReadyToSend_(false)
{
    txBuffer_[ESerialRequestOffset::REQ_START]  = static_cast<char>(SERIAL_REQUEST_START_VALUE);
    txBuffer_[ESerialRequestOffset::REQ_CRC]    = static_cast<char>(SERIAL_REQUEST_CRC_VALUE);
}



Shield::~Shield()
{
    killThread_ = true;
    execThreadHandle_.notify_one();
    sendReceiveThread_.join();
}



bool Shield::openPort(const std::string & portName,
    const uint32_t baudrate,
    const uint8_t dataBits,
    const uint8_t stopBits,
    const uint8_t parity,
    const bool xonXoff,
    const bool rtsCts)
{
    if (true == isPortOpened_)
    {
        RCLCPP_ERROR(rosLogger_, "Shield::openPort() -> port is already opened, close it first with closePort()");
        return false;
    }
    
    try
    {
        uart_ = std::make_unique<mraa::Uart>(portName);
    }
    catch (std::exception & e)
    {
        RCLCPP_ERROR(rosLogger_, "Shield::openPort() -> error in constructor mraa::Uart(): %s", e.what());
        return false;
    }

    if (false == setBaudrate(baudrate))
    {
        return false;
    }

    if (false == setMode(dataBits, static_cast<mraa::UartParity>(parity), stopBits))
    {
        return false;
    }

    if (false == setFlowControl(xonXoff, rtsCts))
    {
        return false;
    }

    mraa::Result flushResult = uart_->flush();

    if (mraa::Result::SUCCESS != flushResult)
    {
        RCLCPP_ERROR(rosLogger_, "Shield::openPort() -> error in function flush() with code: %d", flushResult);
        return false;
    }

    sendReceiveThread_ = std::thread(&Shield::sendReceiveInThread, this);

    RCLCPP_INFO(rosLogger_, "Shield::openPort() -> %s (baudrate: %d, data bits: %d, stop bits: %d, parity: %d) has been successfully opened", portName.c_str(), baudrate, dataBits, stopBits, parity);

    isPortOpened_ = true;
    return true;
}



void Shield::closePort()
{
    killThread_ = true;
    execThreadHandle_.notify_one();
    sendReceiveThread_.join();

    uart_.reset(nullptr);

    isPortOpened_       = false;
    killThread_         = false;
    dataIsReadyToSend_  = false;

    RCLCPP_INFO(rosLogger_, "Shield::closePort() -> port successfully closed");
}



bool Shield::isPortOpened() const
{
    return isPortOpened_;
}



bool Shield::setBaudrate(const uint32_t baudrate)
{
    if (mraa::Result::SUCCESS != uart_->setBaudRate(baudrate))
    {
        RCLCPP_ERROR(rosLogger_, "Shield::setBaudrate() -> error setting baudrate on UART port");
        return false;
    }

    return true;
}



bool Shield::setMode(const uint8_t dataBits, const mraa::UartParity parity, const uint8_t stopBits)
{
    if (mraa::Result::SUCCESS != uart_->setMode(dataBits, parity, stopBits))
    {
        RCLCPP_ERROR(rosLogger_, "Shield::setMode() -> error setting modes on UART port");
        return false;
    }

    return true;
}



bool Shield::setFlowControl(const bool xOnOff, const bool rtsCts)
{
    if (mraa::Result::SUCCESS != uart_->setFlowcontrol(xOnOff, rtsCts))
    {
        RCLCPP_ERROR(rosLogger_, "Shield::setFlowControl() -> error setting flow control on UART port");
        return false;
    }

    return true;
}



bool Shield::send(const SerialRequestData & data)
{
    if (true == isPortOpened_)
    {
        {
            std::lock_guard<std::mutex> lock(sharedMutex_);

            fillTxBuffer(data);
            dataIsReadyToSend_ = true;
        }
        
        execThreadHandle_.notify_one();
    }
    else
    {
        RCLCPP_ERROR(rosLogger_, "Shield::send() -> port is not opened, set it up with openPort()");
        return false;
    }

    return true;
}



bool Shield::receive(SerialResponseData & data)
{
    if (true == isPortOpened_)
    {    
        std::unique_lock<std::mutex> lock(sharedMutex_);
        execThreadHandle_.wait(lock, [&] { return (false == dataIsReadyToSend_); });

        fillDataFromRxBuffer(data);
    }
    else
    {
        RCLCPP_ERROR(rosLogger_, "Shield::receive() -> port is not opened, set it up with openPort()");
        return false;
    }

    return true;
}



void Shield::fillTxBuffer(const SerialRequestData & data)
{
    setTxBufferWith(data.command, ESerialRequestOffset::REQ_COMMAND);
    setTxBufferWith(data.payload, ESerialRequestOffset::REQ_PAYLOAD);
}



void Shield::fillDataFromRxBuffer(SerialResponseData & data)
{
    getValueFromRxBuffer(data.command,      ESerialResponseOffset::RES_COMMAND);
    getValueFromRxBuffer(data.rpm,          ESerialResponseOffset::RES_RPM);
    getValueFromRxBuffer(data.imu,          ESerialResponseOffset::RES_IMU);
    getValueFromRxBuffer(data.tof,          ESerialResponseOffset::RES_TOF);
    getValueFromRxBuffer(data.voltage,      ESerialResponseOffset::RES_VOLTAGE);
    getValueFromRxBuffer(data.temperature,  ESerialResponseOffset::RES_TEMPERATURE);
}



template <typename T>
void Shield::setTxBufferWith(const T & value, const ESerialRequestOffset offset)
{
    const uint32_t valSize = sizeof(T);

    if (SERIAL_REQUEST_BUFFER_LEN < (offset + valSize))
    {
        RCLCPP_ERROR(rosLogger_, "Shield::setTxBufferWith() -> memory failure possible because tx buffer length (%d) < than offset (%d) + value size (%d)", SERIAL_REQUEST_BUFFER_LEN, offset, valSize);
        return;
    }

    // Assign value byte(s) to tx buffer data at given offset
    std::memcpy(&txBuffer_[offset], &value, valSize);
}



template <typename T>
void Shield::getValueFromRxBuffer(T & value, const ESerialResponseOffset offset)
{
    const uint32_t valSize = sizeof(T);

    if (SERIAL_RESPONSE_BUFFER_LEN < (offset + valSize))
    {
        RCLCPP_ERROR(rosLogger_, "Shield::getValueFromRxBuffer() -> memory failure possible because tx buffer length (%d) < than offset (%d) + value size (%d)", SERIAL_RESPONSE_BUFFER_LEN, offset, valSize);
        return;
    }
    
    // Assign rx buffer data at given offset to value byte(s)
    std::memcpy(&value, &rxBuffer_[offset], valSize);
}



void Shield::sendReceiveInThread()
{
    while (1)
    {
        {
            std::unique_lock<std::mutex> lock(sharedMutex_);
            execThreadHandle_.wait(lock, [&] { return ((true == dataIsReadyToSend_) || (true == killThread_)); }); // wait until data is available or command to end thread

            if (true == killThread_)
            {
                RCLCPP_INFO(rosLogger_, "Shield::sendReceiveInThread() -> exiting thread ...");
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(8)); // sleep for 8 milliseconds to ensure that serial port ist not busy

            // send data to shield module
            int32_t bytesTransmitted = 0;
            
            bytesTransmitted = uart_->write(txBuffer_, SERIAL_REQUEST_BUFFER_LEN);

            if (SERIAL_REQUEST_BUFFER_LEN != bytesTransmitted)
            {
                RCLCPP_ERROR(rosLogger_, "Shield::sendReceiveInThread() -> number of bytes written (%d) not equal to send array length (%d) \n", bytesTransmitted, SERIAL_REQUEST_BUFFER_LEN);
            }

            // read data from shield module
            bytesTransmitted = 0;
            
            bytesTransmitted = uart_->read(rxBuffer_, SERIAL_RESPONSE_BUFFER_LEN);

            if (SERIAL_RESPONSE_BUFFER_LEN != bytesTransmitted)
            {
                RCLCPP_ERROR(rosLogger_, "Shield::sendReceiveInThread() -> number of bytes read (%d) not equal to receive array length (%d) \n", bytesTransmitted, SERIAL_RESPONSE_BUFFER_LEN);
            }

            dataIsReadyToSend_ = false;
        }

        execThreadHandle_.notify_one();
    }
}


} // namespace iotbot
