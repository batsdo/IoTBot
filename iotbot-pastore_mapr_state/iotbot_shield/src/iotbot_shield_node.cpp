#include <functional>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_srvs/srv/empty.hpp"

#include "iotbot_shield.hpp"
#include "iotbot_interface/msg/rotation_speed.hpp"
#include "iotbot_interface/msg/battery.hpp"
#include "iotbot_interface/msg/uint8.hpp"
#include "iotbot_interface/msg/bool.hpp"
#include "iotbot_interface/srv/send_lighting.hpp"
#include "iotbot_interface/srv/send_enable.hpp"
#include <std_srvs/srv/empty.hpp>


namespace iotbot {


// TODO: Manuel Kreutz: place enums in separated header file directory
// -> e.g. create a new "util" folder in repo and add all global header declarations (enum classes etc) there
enum EWheelPosition
{
    frontLeft   = 2,
    frontRight  = 1,
    rearLeft    = 3,
    rearRight   = 0
};


enum EJoyButton
{
    beamLight       = 0,
    flashLeftLight  = 4,
    flashRightLight = 5,
    rotationalLight = 8,
    runningLight    = 9,
    enableRobot     = 10,
    warningLight    = 1
};


enum EColorRGB
{
    red     = 0,
    green   = 1,
    blue    = 2
};


/**
 * @class ShieldNode
 * @brief Shield node class for connecting to the motor shield
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 01.09.2021
 */
class ShieldNode : public rclcpp::Node
{
public:
    /**
     * constructor
     */
    ShieldNode() :
        rclcpp::Node("iotbot_shield", rclcpp::NodeOptions()),
        currentTime_(rclcpp::Clock().now()),
        lastTime_(rclcpp::Clock().now()),
        deltaTime_(std::chrono::nanoseconds::zero()),
        serialPort_(this->get_logger())
    {
        // create callback group for /joy and /iotbot/rpm topics -> subscriber callbacks in separated thread
        auto callbackGroupSubscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions subscriberCallbackGroupOptions;
        subscriberCallbackGroupOptions.callback_group = callbackGroupSubscriber;
        
        // ****************************** subscriber ******************************
        joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                          rclcpp::SystemDefaultsQoS(),
                                                                          std::bind(&ShieldNode::joyCallback, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                    this,
                                                                                    std::placeholders::_1),
                                                                          subscriberCallbackGroupOptions);
        
        rpmSubscriber_ = this->create_subscription<iotbot_interface::msg::RotationSpeed>("iotbot/rpm",
                                                                                         rclcpp::SensorDataQoS(),
                                                                                         std::bind(&ShieldNode::rpmCallback, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                                   this,
                                                                                                   std::placeholders::_1),
                                                                                         subscriberCallbackGroupOptions);

        // ****************************** publisher ******************************
        rpmPublisher_ = this->create_publisher<iotbot_interface::msg::RotationSpeed>("iotbot/rpm/return",
                                                                                     rclcpp::SystemDefaultsQoS());
        
        imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("iotbot/imu",
                                                                      rclcpp::SensorDataQoS());

        tofPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("iotbot/tof",
                                                                                 rclcpp::SensorDataQoS());

        batteryPublisher_ = this->create_publisher<iotbot_interface::msg::Battery>("iotbot/battery",
                                                                                   rclcpp::SensorDataQoS());

        temperaturePublisher_ = this->create_publisher<std_msgs::msg::Float32>("iotbot/temperature",
                                                                               rclcpp::SensorDataQoS());

        // ****************************** services ******************************
        srvSendLighting_ = this->create_service<iotbot_interface::srv::SendLighting>("iotbot/srv/send_lighting",
                                                                                     std::bind(&ShieldNode::handleSendLighting, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                               this, 
                                                                                               std::placeholders::_1, 
                                                                                               std::placeholders::_2),
                                                                                     rclcpp::ServicesQoS().get_rmw_qos_profile());

        srvSendEnable_ = this->create_service<iotbot_interface::srv::SendEnable>("iotbot/srv/send_enable", 
                                                                                 std::bind(&ShieldNode::handleSendEnable, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                           this,
                                                                                           std::placeholders::_1, 
                                                                                           std::placeholders::_2));

        srvSendIMUCalibration_ = this->create_service<std_srvs::srv::Empty>("iotbot/srv/send_imu_calibration", 
                                                                                 std::bind(&ShieldNode::handleCalibrateIMU,
                                                                                           this,
                                                                                           std::placeholders::_1, 
                                                                                           std::placeholders::_2));

        srvSendIMUMode_ = this->create_service<iotbot_interface::srv::SendEnable>("iotbot/srv/send_imu_mode", 
                                                                                 std::bind(&ShieldNode::handleIMUMode,
                                                                                           this,
                                                                                           std::placeholders::_1, 
                                                                                           std::placeholders::_2));
    }


    /**
     * destructor
     */
    ~ShieldNode()
    {
        rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
        rgb_[EColorRGB::green]  = 0x00;
        rgb_[EColorRGB::blue]   = 0x00;

        sendLighting(iotbot::CMD_LIGHTS_PULSATION);
        sendDisable();
    }


    /**
     * initialize UART and shield module with specific values for hardware setup
     */
    bool init()
    {
        // Serial port parameters
        std::string portName        = this->declare_parameter("portName", "/dev/ttyS1");
        uint32_t baudrate           = this->declare_parameter("baudrate", 115200);
        uint8_t dataBits            = this->declare_parameter("dataBits", 8);
        uint8_t stopBits            = this->declare_parameter("stopBits", 1);
        uint8_t parity              = this->declare_parameter("parity", 0);
        bool xonXoff                = this->declare_parameter("xonXoff", 0);
        bool rtsCts                 = this->declare_parameter("rtsCts", 0);

        // Default-Values: Performance version
        float gearRatio             = this->declare_parameter("gearRatio", 72.f);
        float encoderTicksPerRev    = this->declare_parameter("encoderTicksPerRev", 2048.f);
        float kp                    = this->declare_parameter("kp", 0.5f);
        float ki                    = this->declare_parameter("ki", 10.f);
        float kd                    = this->declare_parameter("kd", 0.f);
        uint32_t controlFrequency   = this->declare_parameter("controlFrequency", 16000);
        float lowPassInputFilter    = this->declare_parameter("lowPassInputFilter", 0.3f);
        float lowPassEncoderTicks   = this->declare_parameter("lowPassEncoderTicks", 0.1f);
        bool rawIMUData             = this->declare_parameter("rawIMUData", 0);
        float driftWeight           = this->declare_parameter("driftWeight", 0.02f);
        
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] portName set with value:      %s", portName.c_str());
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] baudrate set with value:      %d", baudrate);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] dataBits set with value:      %d", dataBits);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] stopBits set with value:      %d", stopBits);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] parity set with value:        %d", parity);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] xonXoff set with value:       %d", xonXoff);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> [UART] rtsCts set with value:        %d", rtsCts);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> gearRatio set with value:            %.2f", gearRatio);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> encoderTicksPerRev set with value:   %.2f", encoderTicksPerRev);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> kp set with value:                   %.2f", kp);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> ki set with value:                   %.2f", ki);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> kd set with value:                   %.2f", kd);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> controlFrequency set with value:     %d Hz", controlFrequency);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> lowPassInputFilter set with value:   %.2f", lowPassInputFilter);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> lowPassEncoderTicks set with value:  %.2f", lowPassEncoderTicks);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> rawIMUData set with value:           %d", rawIMUData);
        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> driftWeight set with value:          %.2f", driftWeight);

        if (false == serialPort_.openPort(portName, baudrate, dataBits, stopBits, parity, xonXoff, rtsCts))
        {
            RCLCPP_ERROR(this->get_logger(), "ShieldNode::init() -> exiting because of an error with serial connection function Shield::openPort()");
            return false;
        }

        rgb_[EColorRGB::red]    = 0x00; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
        rgb_[EColorRGB::green]  = 0xFF;
        rgb_[EColorRGB::blue]   = 0x00;

        sendLighting(iotbot::CMD_LIGHTS_DIM_LIGHT);
        sendGearRatio(gearRatio);
        sendTicksPerRev(encoderTicksPerRev);
        sendKp(kp);
        sendKi(ki);
        sendKd(kd);
        sendControlFrequency(controlFrequency);
        sendLowPassSetPoint(lowPassInputFilter);
        sendLowPassEncoder(lowPassEncoderTicks);
        sendIMUMode(rawIMUData);
        sendDriftWeight(driftWeight);
        sendCalibrateIMU();

        RCLCPP_INFO(this->get_logger(), "ShieldNode::init() -> successfully initialized shield module with parameters");
        return true;
    }


    /**
     * check whether the subscribed RPM message is new or not and send current RPM to shield module
     */
    void checkAndSendCurrentRPM()
    {
        currentTime_ = rclcpp::Clock().now();
        deltaTime_   = lastTime_ - currentTime_;

        if (deltaTime_.seconds() > 0.5)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::checkAndSendCurrentRPM() -> time difference since last RPM message greater than 0.5 sec -> disabling motor control");

            for (auto & it : rpm_)
            {
                it = 0.f;
            }

            rgb_[EColorRGB::red]    = 0x00; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0xFF;
            rgb_[EColorRGB::blue]   = 0x00;

            sendLighting(CMD_LIGHTS_DIM_LIGHT);
            sendRPM();
            sendDisable();
        }
        else
        {
            sendRPM();
        }
    }

    // ********************************************************************************* publish functions *********************************************************************************

    /**
     * publish the newly calculated RPM data to the network
     */
    void publishRPM()
    {
        iotbot_interface::msg::RotationSpeed rpmMessage;
        
        rpmMessage.front_left_rpm  = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::frontLeft])  / 100.f;
        rpmMessage.front_right_rpm = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::frontRight]) / 100.f;
        rpmMessage.rear_left_rpm   = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::rearLeft])   / 100.f;
        rpmMessage.rear_right_rpm  = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::rearRight])  / 100.f;

        rpmPublisher_->publish(rpmMessage);
    }


    /**
     * publish the IMU data to the network
     */
    void publishIMU()
    {
        sensor_msgs::msg::Imu imuMessage;
        
        imuMessage.header.frame_id             = "imu_link";
        imuMessage.header.stamp                = currentTime_;

        imuMessage.orientation_covariance      = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        imuMessage.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        if (true == isRawIMUData_)
        {
            // convert from mg*10 to g
            imuMessage.linear_acceleration.x   = static_cast<double>(serialReceiveData_.imu[0]) / 10000.0;
            imuMessage.linear_acceleration.y   = static_cast<double>(serialReceiveData_.imu[1]) / 10000.0;
            imuMessage.linear_acceleration.z   = static_cast<double>(serialReceiveData_.imu[2]) / 10000.0;

            // convert from mdps/10 to dps
            imuMessage.angular_velocity.x      = static_cast<double>(serialReceiveData_.imu[3]) / 100.0;
            imuMessage.angular_velocity.y      = static_cast<double>(serialReceiveData_.imu[4]) / 100.0;
            imuMessage.angular_velocity.z      = static_cast<double>(serialReceiveData_.imu[5]) / 100.0;

            imuMessage.orientation.w           = 0.0;
            imuMessage.orientation.x           = 0.0;
            imuMessage.orientation.y           = 0.0;
            imuMessage.orientation.z           = 0.0;
        }
        else
        {
            imuMessage.linear_acceleration.x   = 0.0;
            imuMessage.linear_acceleration.y   = 0.0;
            imuMessage.linear_acceleration.z   = 0.0;

            imuMessage.angular_velocity.x      = 0.0;
            imuMessage.angular_velocity.y      = 0.0;
            imuMessage.angular_velocity.z      = 0.0;

            imuMessage.orientation.w           = static_cast<double>(serialReceiveData_.imu[0]) / 10000.0;
            imuMessage.orientation.x           = static_cast<double>(serialReceiveData_.imu[1]) / 10000.0;
            imuMessage.orientation.y           = static_cast<double>(serialReceiveData_.imu[2]) / 10000.0;
            imuMessage.orientation.z           = static_cast<double>(serialReceiveData_.imu[3]) / 10000.0;
        }

        imuPublisher_->publish(imuMessage);
    }
    

    /**
     * publish the TOF data to the network
     */
    void publishTOF()
    {
        std_msgs::msg::Float32MultiArray tofMessage;
        
        tofMessage.data = { static_cast<float>(serialReceiveData_.tof[0]) / 1000.f, 
                            static_cast<float>(serialReceiveData_.tof[1]) / 1000.f, 
                            static_cast<float>(serialReceiveData_.tof[2]) / 1000.f, 
                            static_cast<float>(serialReceiveData_.tof[3]) / 1000.f  };

        tofPublisher_->publish(tofMessage);
    }


    /**
     * publish the actual voltage to the network
     */
    void publishVoltage()
    {
        iotbot_interface::msg::Battery batteryMessage;
        
        batteryMessage.voltage = static_cast<float>(serialReceiveData_.voltage) / 100.f;

        batteryPublisher_->publish(batteryMessage);
    }


    /**
     * publish the actual temperature to the network
     */
    void publishTemperature()
    {
        // TODO: Manuel Kreutz: change temperature data to rx[31] and 1 byte long (not imu array [4] -> rx [17] and rx [18]) -> otherwise this byte in protocol is useless !!!
        // -> temperatureMessage.data = static_cast<float>(serialReceiveData_.temperature) / 100.f;

        std_msgs::msg::Float32 temperatureMessage;
        
        if (true == isRawIMUData_)
        {
            temperatureMessage.data = -273.f;
        }
        else
        {
            temperatureMessage.data = static_cast<float>(serialReceiveData_.imu[4]) / 100.f;
        }

        temperaturePublisher_->publish(temperatureMessage);
    }

private:
    rclcpp::Subscription<iotbot_interface::msg::RotationSpeed>::SharedPtr   rpmSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                  joySubscriber_;

    rclcpp::Publisher<iotbot_interface::msg::RotationSpeed>::SharedPtr      rpmPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                     imuPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr          tofPublisher_;
    rclcpp::Publisher<iotbot_interface::msg::Battery>::SharedPtr            batteryPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                    temperaturePublisher_;

    rclcpp::Service<iotbot_interface::srv::SendLighting>::SharedPtr         srvSendLighting_;
    rclcpp::Service<iotbot_interface::srv::SendEnable>::SharedPtr           srvSendEnable_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                        srvSendIMUCalibration_;
    rclcpp::Service<iotbot_interface::srv::SendEnable>::SharedPtr           srvSendIMUMode_;


    rclcpp::Time        currentTime_;
    rclcpp::Time        lastTime_;
    rclcpp::Duration    deltaTime_;

    Shield              serialPort_;
    SerialRequestData   serialSendData_;
    SerialResponseData  serialReceiveData_;

    std::array<float,   4>  rpm_{0}; // TODO: Manuel Kreutz: set to global constexpr values in util header
    std::array<uint8_t, 3>  rgb_{0}; 

    int32_t buttonBeamLight_{0};    // beam light
    int32_t buttonFlashLeft_{0};    // flash left side light
    int32_t buttonFlashRight_{0};   // flash right side light
    int32_t buttonRotational_{0};   // rotational light
    int32_t buttonRunning_{0};      // running light
    int32_t buttonEnable_{0};       // enable robot
    int32_t buttonWarning_{0};      // warning light

    bool isRawIMUData_;


    // ********************************************************************************* callback functions *********************************************************************************

    /**
     * the subscription callback function of the RPM message
     * @param[in] rpmMsg specifies the RPM parameters
     */
    void rpmCallback(const iotbot_interface::msg::RotationSpeed::SharedPtr rpmMsg)
    {
        lastTime_ = rclcpp::Clock().now();

        rpm_[EWheelPosition::frontLeft]  = rpmMsg->front_left_rpm;
        rpm_[EWheelPosition::frontRight] = rpmMsg->front_right_rpm;
        rpm_[EWheelPosition::rearLeft]   = rpmMsg->rear_left_rpm;
        rpm_[EWheelPosition::rearRight]  = rpmMsg->rear_right_rpm;
    }


    /**
     * the subscription callback function of the Joy message to set lightnings
     * @param[in] joyMsg specifies the joy values from e.g. a PS4 controller
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
    {   
        if (joyMsg->buttons[EJoyButton::beamLight] && !buttonBeamLight_)
        {   
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting beam light");

            rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0xFF;
            rgb_[EColorRGB::blue]   = 0xFF;

            sendLighting(CMD_LIGHTS_HIGH_BEAM);
        }

        if (joyMsg->buttons[EJoyButton::flashLeftLight] && !buttonFlashLeft_)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting flash light on left side");

            rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0x88;
            rgb_[EColorRGB::blue]   = 0x00;

            sendLighting(CMD_LIGHTS_FLASH_LEFT);
        }

        if (joyMsg->buttons[EJoyButton::flashRightLight] && !buttonFlashRight_)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting flash light on right side");

            rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0x88;
            rgb_[EColorRGB::blue]   = 0x00;

            sendLighting(CMD_LIGHTS_FLASH_RIGHT);
        }

        if (joyMsg->buttons[EJoyButton::rotationalLight] && !buttonRotational_)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting rotational light");

            rgb_[EColorRGB::red]    = 0x00; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0x00;
            rgb_[EColorRGB::blue]   = 0xFF;

            sendLighting(CMD_LIGHTS_ROTATION);
        }

        if (joyMsg->buttons[EJoyButton::runningLight] && !buttonRunning_)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting running light");

            rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0x00;
            rgb_[EColorRGB::blue]   = 0x00;

            sendLighting(CMD_LIGHTS_RUNNING);
        }

        if (joyMsg->buttons[EJoyButton::enableRobot] && !buttonEnable_)
        {   
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> enabling robot");

            sendEnable();
        }

        if (joyMsg->buttons[EJoyButton::warningLight] && !buttonWarning_)
        {   
            RCLCPP_INFO(this->get_logger(), "ShieldNode::joyCallback() -> setting warning light (flash all)");

            rgb_[EColorRGB::red]    = 0xFF; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
            rgb_[EColorRGB::green]  = 0x88;
            rgb_[EColorRGB::blue]   = 0x00;

            sendLighting(CMD_LIGHTS_FLASH_ALL);
        }

        buttonBeamLight_    = joyMsg->buttons[EJoyButton::beamLight];
        buttonFlashLeft_    = joyMsg->buttons[EJoyButton::flashLeftLight];
        buttonFlashRight_   = joyMsg->buttons[EJoyButton::flashRightLight];
        buttonRotational_   = joyMsg->buttons[EJoyButton::rotationalLight];
        buttonRunning_      = joyMsg->buttons[EJoyButton::runningLight];
        buttonEnable_       = joyMsg->buttons[EJoyButton::enableRobot];
        buttonWarning_      = joyMsg->buttons[EJoyButton::warningLight];
    }


    /**
     * the service call handle to send a lighting command and colors to the shield module
     * @param[in] request the lighting command and corresponding RGB values to send
     * @param[out] response returns true: successfully sent and received data, false: an error occured
     */
    void handleSendLighting(const std::shared_ptr<iotbot_interface::srv::SendLighting::Request> request, // TODO: Manuel Kreutz: change types to ...::SharedPtr
                            std::shared_ptr<iotbot_interface::srv::SendLighting::Response> response)
    {  
        RCLCPP_INFO(this->get_logger(), "ShieldNode::handleSendLighting() -> called service: send lighting with command: 0x%x", request->command);
        
        rgb_[EColorRGB::red]    = request->red; // TODO: Manuel Kreutz: change color values to ENUM ones -> create function like setColor(COLOR_TYPE)
        rgb_[EColorRGB::green]  = request->green;
        rgb_[EColorRGB::blue]   = request->blue;

        response->sending_succesfull = sendLighting(static_cast<ESerialCommand>(request->command));
    }


    /**
     * the service call handle to send a enable or disable command to the shield module
     * @param[in] request enable / disable command message
     * @param[out] response returns true: successfully sent and received data, false: an error occured
     */
    void handleSendEnable(const std::shared_ptr<iotbot_interface::srv::SendEnable::Request> request, // TODO: Manuel Kreutz: change types to ...::SharedPtr
                          std::shared_ptr<iotbot_interface::srv::SendEnable::Response> response)
    {    
        if (true == request->enable)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::handleSendEnable() -> called service: enable robot");

            response->sending_succesfull = sendEnable();
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::handleSendEnable() -> called service: disable robot");

            response->sending_succesfull = sendDisable();
        }
    }


    /**
     * the service call handle for triggering IMU calibration
     * @param[in] request empty placeholder as input
     * @param[out] response empty placeholder as output
     */
    void handleCalibrateIMU([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty_Request> request,
                            [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty_Response> response)
    {  
        // (void)request; // TODO: Manuel Kreutz: set to [[maybe_unused]] in declaration
        
        RCLCPP_INFO(this->get_logger(), "ShieldNode::handleCalibrateIMU() -> called service: calibrate IMU");

        sendCalibrateIMU();

        // (void)response;
    }

    /**
     * the service call handle for changing IMU Mode
     * @param[in] request enable / disable command for imu fusion
     * @param[out] response empty placeholder as output
     */
    void handleIMUMode(const std::shared_ptr<iotbot_interface::srv::SendEnable::Request> request,
                          std::shared_ptr<iotbot_interface::srv::SendEnable::Response> response)
    {    
        if (true == request->enable)
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::handleIMUMode() -> called service: enable sensorfusion");

            response->sending_succesfull = sendIMUMode(true);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "ShieldNode::handleIMUMode() -> called service: disable sensorfusion");

            response->sending_succesfull = sendIMUMode(false);
        }
    }

    // ********************************************************************************* send functions *********************************************************************************

    /**
     * send enable command to shield module (must be done before steering)
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendEnable()
    {
        serialSendData_.command = ESerialCommand::CMD_ENABLE;

        for (auto & it : serialSendData_.payload)
        {
            it = 0;
        }

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_ENABLE != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendEnable() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send disable command to shield module (no motion can be performed after disabling)
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendDisable()
    {
        serialSendData_.command = ESerialCommand::CMD_DISABLE;

        for (auto & it : serialSendData_.payload)
        {
            it = 0;
        }

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_DISABLE != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendDisable() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send IMU mode command to shield module to enable / disable IMU raw data transmission
     * @param[in] useRawData true: raw data is transmitted (default), false: fused data as quaternion is transmitted
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendIMUMode(const bool useRawData)
    {
        isRawIMUData_ = useRawData;
        
        serialSendData_.command = ESerialCommand::CMD_IMU_RAW_DATA;
        
        serialSendData_.payload[0]  = (useRawData ? 0xFF : 0x00) << 8;
        serialSendData_.payload[1]  = 0;
        serialSendData_.payload[2]  = 0;
        serialSendData_.payload[3]  = 0;

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_IMU_RAW_DATA != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendIMUMode() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
    * send time without UART communication until an enable state will be removed
    * @param [in] timeout the timeout value to be set as timer
    * @return true: successfully sent and received data, false: an error occured
    */
    bool sendTimeout(const float timeout)
    {
        serialSendData_.command = ESerialCommand::CMD_UART_TIMEOUT;

        std::memcpy(serialSendData_.payload, &timeout, sizeof(timeout));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_UART_TIMEOUT != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendTimeout() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
    * send weight of drift parameter. Gyro data is used for fast orientation changes,
    * while the accelerometer comensates drift for 2 of the degrees of freedom
    * @param[in] weight a larger weight increases the belief in the accelerometer values [0; 1], default: 0.02f
    * return true: successfully sent and received data, false: an error occured
    */
    bool sendDriftWeight(const float weight)
    {
        serialSendData_.command = ESerialCommand::CMD_FUSE_IMU_WEIGHT;

        std::memcpy(serialSendData_.payload, &weight, sizeof(weight));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_FUSE_IMU_WEIGHT != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendDriftWeight() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
    * send trigger IMU calibration (takes about 1 second). The platform needs to stand still during this procedure
    * @return true: successfully sent and received data, false: an error occured
    */
    bool sendCalibrateIMU()
    {
        serialSendData_.command = ESerialCommand::CMD_IMU_CALIBRATE;

        for (auto & it : serialSendData_.payload)
        {
            it = 0;
        }

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_IMU_CALIBRATE != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendCalibrateIMU() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send gear ratio command to shield module
     * @param[in] gearRatio specifies the ratio of the used gear
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendGearRatio(const float gearRatio)
    {
        serialSendData_.command = ESerialCommand::CMD_GEAR_RATIO;

        std::memcpy(serialSendData_.payload, &gearRatio, sizeof(gearRatio));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_GEAR_RATIO != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendGearRatio() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send ticks per revision command to shield module
     * @param[in] ticksPerRev specifies the ticks per revoluation (raising and falling edges)
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendTicksPerRev(const float ticksPerRev)
    {
        serialSendData_.command = ESerialCommand::CMD_TICKS_PER_REV;

        std::memcpy(serialSendData_.payload, &ticksPerRev, sizeof(ticksPerRev));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_TICKS_PER_REV != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendTicksPerRev() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send proportional coefficient of closed loop controller
     * @param[in] kP specifies the proportional weight
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendKp(const float kP)
    {
        serialSendData_.command = ESerialCommand::CMD_CTL_KP;

        std::memcpy(serialSendData_.payload, &kP, sizeof(kP));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_CTL_KP != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendKp() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send integration coefficient of closed loop controller
     * @param[in] kI specifies the integration coefficient
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendKi(const float kI)
    {
        serialSendData_.command = ESerialCommand::CMD_CTL_KI;

        std::memcpy(serialSendData_.payload, &kI, sizeof(kI));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_CTL_KI != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendKi() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send differentiating coefficient of closed loop controller
     * @param[in] kD specifies the differentiating coefficient
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendKd(const float kD)
    {
        serialSendData_.command = ESerialCommand::CMD_CTL_KD;

        std::memcpy(serialSendData_.payload, &kD, sizeof(kD));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_CTL_KD != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendKd() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send frequency of bridge driver for motor control
     * @param[in] frequency frequency in HZ in range [1000, 1000000]
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendControlFrequency(const uint32_t frequency)
    {
        serialSendData_.command = ESerialCommand::CMD_FREQ;

        std::memcpy(serialSendData_.payload, &frequency, sizeof(frequency));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_FREQ != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendControlFrequency() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send low pass coefficient of set point. New values are weighted with this value
     * @param[in] weight weight low pass coefficient of set point
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendLowPassSetPoint(const float weight)
    {
        serialSendData_.command = ESerialCommand::CMD_CTL_INPUT_FILTER;

        std::memcpy(serialSendData_.payload, &weight, sizeof(weight));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_CTL_INPUT_FILTER != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendLowPassSetPoint() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send low pass coefficient of encoder measurements. New values are weighted with this value
     * @param[in] weight low pass coefficient of encoder measurements
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendLowPassEncoder(const float weight)
    {
        serialSendData_.command = ESerialCommand::CMD_CTL_ENC_LOWPASS;

        std::memcpy(serialSendData_.payload, &weight, sizeof(weight));

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_CTL_ENC_LOWPASS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendLowPassEncoder() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send switch on/off auxiliary output (channel 1)
     * @param[in] on set to true to set AUX1 on, false to set it off
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendAUX1(const bool on)
    {
        serialSendData_.command     = ESerialCommand::CMD_AUX1;

        serialSendData_.payload[0]  = (on ? 1 : 0) << 8;
        serialSendData_.payload[1]  = 0;
        serialSendData_.payload[2]  = 0;
        serialSendData_.payload[3]  = 0;

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_AUX1 != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendAUX1() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send switch on/off auxiliary output (channel 2)
     * @param[in] on set to true to set AUX2 on, false to set it off
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendAUX2(const bool on)
    {
        serialSendData_.command     = ESerialCommand::CMD_AUX2;

        serialSendData_.payload[0]  = (on ? 1 : 0) << 8;
        serialSendData_.payload[1]  = 0;
        serialSendData_.payload[2]  = 0;
        serialSendData_.payload[3]  = 0;

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_AUX2 != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendAUX2() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send PWM command to shield module
     * @param[in] pwm specifies the pulse width modulation for each light
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendPWM(const std::array<uint16_t, 4> & pwm)
    {
        serialSendData_.command = ESerialCommand::CMD_SET_PWM;

        for (std::size_t i = 0; i < pwm.size(); ++i)
        {
            serialSendData_.payload[i] = static_cast<int16_t>(pwm[i]);
        }

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_SET_PWM != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendPWM() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }


    /**
     * send LED lightning command with color settings to shield module (uses the internal RGB member array)
     * @param[in] command specifies the lightning command for the LEDs -> starts with CMD_LIGHTS_...
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendLighting(const ESerialCommand command)
    {
        serialSendData_.command = command;

        serialSendData_.payload[0]  = rgb_[EColorRGB::red];         // 1. byte of payload -> red
        serialSendData_.payload[0] |= rgb_[EColorRGB::green] << 8;  // 2. byte of payload -> green
        serialSendData_.payload[1]  = rgb_[EColorRGB::blue];        // 3. byte of payload -> blue
        serialSendData_.payload[2]  = 0;
        serialSendData_.payload[3]  = 0;

        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (command != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendLighting() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendLighting() -> tried to send command: 0x%x with colors red: %d , green: %d , blue: %d", command, rgb_[EColorRGB::red], rgb_[EColorRGB::green], rgb_[EColorRGB::blue]);
            return false;
        }

        return true;
    }


    /**
     * send RPM command to shield module (uses the internal RPM member array)
     * @return true: successfully sent and received data, false: an error occured
     */
    bool sendRPM()
    {
        serialSendData_.command = ESerialCommand::CMD_SET_RPM;
        
        for (std::size_t i = 0; i < rpm_.size(); ++i)
        {
            serialSendData_.payload[i] = static_cast<int16_t>(rpm_[i] * 100.f + 0.5f); // * 100.f (for two digits after decimal point) + 0.5f (round up)
        }
        
        if (false == serialPort_.send(serialSendData_)) return false;
        if (false == serialPort_.receive(serialReceiveData_)) return false;

        if (ESerialCommand::CMD_SET_RPM != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "ShieldNode::sendRPM() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
            return false;
        }

        return true;
    }

}; // ShieldNode


} // namespace iotbot



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto shieldNode = std::make_shared<iotbot::ShieldNode>();

    if (false == shieldNode->init())
    {
        return EXIT_FAILURE;
    }

    // multithreaded executor to handle separated subscriber threads
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(shieldNode);

    rclcpp::Rate loopRate(50); // loop rate in milliseconds

    while (rclcpp::ok())
    {
        shieldNode->checkAndSendCurrentRPM();

        shieldNode->publishRPM();
        shieldNode->publishIMU();
        shieldNode->publishTOF();
        shieldNode->publishVoltage();
        shieldNode->publishTemperature();

        executor.spin_some();
        loopRate.sleep(); 
    }

    return EXIT_SUCCESS;
}