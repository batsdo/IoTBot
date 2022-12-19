#ifndef IOTBOT_SHIELD__SHIELD_HPP_
#define IOTBOT_SHIELD__SHIELD_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include <iterator>
#include <memory>
#include <thread>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "mraa/common.hpp"
#include "mraa/uart.hpp"


namespace iotbot {


#include "iotbot_uart_protocol.h"


/**
 * @class Shield
 * @brief Serial interfacing class (via UART) to shield hardware module
 * @author Stefan May, Manuel Kreutz
 * @date 27.08.2021
 */
class Shield
{
public:
    /**
     * constructor
     * @param[in] logger ROS2 logger for handling terminal output messages
     */
    Shield(rclcpp::Logger logger);

    /**
     * destructor
     */
    ~Shield();

    Shield(const Shield &) = delete;              // no copy constructor
    Shield &operator=(const Shield &) = delete;   // no copy assignment operator

    Shield(Shield &&) = default;                  // move constructor -> let the compiler create the constructor by default
    Shield &operator=(Shield &&) = default;       // move via assignment operator

    /**
     * open the internal port using default serial port parameters
     * @param[in] portName serial port file of the linux system e.g. /dev/ttyS1
     * @param[in] baudrate speed of transmittion in bits/s [9600, 19200, 38400, 57600, 115200]
     * @param[in] dataBits amount of data bits [5, 6, 7, 8]
     * @param[in] stopBits amount of data bits [1, 2]
     * @param[in] parity operational parity mode [NONE: 0, EVEN: 1, ODD: 2] 
     * @param[in] xonXoff software flow control [enable: true, disable: false]
     * @param[in] rtsCts hardware flow control [enable: true, disable: false]
     * @return true: if port was opened successfully, false: if an error occured or the port was already opened
     */
    bool openPort(const std::string & portName, const uint32_t baudrate, const uint8_t dataBits, const uint8_t stopBits, const uint8_t parity, const bool xonXoff, const bool rtsCts);

    /**
     * close the internal port
     */
    void closePort();

    /**
     * check if the internal port is opened 
     * @return true: if port was opened successfully, false: if the port is not opened
     */
    bool isPortOpened() const;

    /**
     * send data over specified port -> has to be opened before
     * @param[in] data the structured data to send
     * @return true: successfully written to UART, false: an error occured
     */
    bool send(const SerialRequestData & data);

    /**
     * receive data over specified port -> has to be opened before
     * @param[out] data the structured receiving data
     * @return true: successfully read from UART, false: an error occured
     */
    bool receive(SerialResponseData & data);
    
private:
    std::unique_ptr<mraa::Uart> uart_;          // the underlying UART interface
    std::thread sendReceiveThread_;             // separated thread to handle send and receive functionality over UART
    std::mutex sharedMutex_;                    // the mutex to be used by the different locks to garantee serialization
    std::condition_variable execThreadHandle_;  // condition variable between the threads to notify each other

    rclcpp::Logger rosLogger_;

    bool isPortOpened_;                         // specifies if the UART port is opened to communicate
    bool killThread_;                           // specifies if the send-receive thread should be exited
    bool dataIsReadyToSend_;                    // condition (memory) for thread handle to specify if data is ready to send (true) or ready to be read (false)

    char txBuffer_[SERIAL_REQUEST_BUFFER_LEN];
    char rxBuffer_[SERIAL_RESPONSE_BUFFER_LEN];


    /**
     * set the baudrate of the internal UART port
     * @param[in] baudrate a valid baudrate e.g. 115200
     * @return true: set up was successful, false: error occured
     */
    bool setBaudrate(const uint32_t baudrate);

    /**
     * set the mode of the internal UART port
     * @param[in] dataBits a valid amount of data bits e.g. 8
     * @param[in] parity the parity mode
     * @param[in] stopBits a valid amount of stop bits e.g. 1
     * @return true: set up was successful, false: error occured
     */
    bool setMode(const uint8_t dataBits, const mraa::UartParity parity, const uint8_t stopBits);

    /**
     * set the flow control of the internal UART port
     * @param[in] xOnOff use of XON / XOFF as flow control
     * @param[in] rtsCts use of RTS / CTS as flow control
     * @return true: set up was successful, false: error occured
     */
    bool setFlowControl(const bool xOnOff, const bool rtsCts);

    /**
     * fill the tx buffer with the given data in order of the specified protocol 
     * !!! changes in the UART request protocol have to be corrected in this function !!!
     * @param[in] data the data struct storing all values to send
     */
    void fillTxBuffer(const SerialRequestData & data);

    /**
     * fill the data struct with the data of the received rx buffer in order of the specified protocol 
     * !!! changes in the UART response protocol have to be corrected in this function !!!
     * @param[out] data the structured receiving data
     */
    void fillDataFromRxBuffer(SerialResponseData & data);

    /**
     * copy a value to the tx buffer at a specified buffer offset
     * @param[in] value the value to copy to the tx buffer
     * @param[in] offset specifies the place in the tx buffer to be filled
     */
    template <typename T>
    void setTxBufferWith(const T & value, const ESerialRequestOffset offset);

    /**
     * copy a via offset specified value from the rx buffer to the given value
     * @param[out] value the value that will be filled
     * @param[in] offset specifies the place in the rx buffer to get the data from
     */
    template <typename T>
    void getValueFromRxBuffer(T & value, const ESerialResponseOffset offset);

    /**
     * the seperate thread loop function for send and receive data over UART
     */
    void sendReceiveInThread();
};


} // namespace iotbot


#endif // IOTBOT_SHIELD__SHIELD_HPP_
