#include "iotbot_motion_control.hpp"


namespace iotbot {


MotionControl::MotionControl() : rclcpp::Node("iotbot_motion_control", rclcpp::NodeOptions())    
{
    setMotorParams();
    setChassisParams();
    calculateRobotProperties();

    rpmPublisher_ = this->create_publisher<iotbot_interface::msg::RotationSpeed>("iotbot/rpm",
                                                                                 rclcpp::SystemDefaultsQoS());   

    velPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                                                                      rclcpp::SystemDefaultsQoS());   
 
    joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy_extended",
                                                                      rclcpp::SystemDefaultsQoS(),
                                                                      std::bind(&MotionControl::joyCallback, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                this,
                                                                                std::placeholders::_1));
}



// ********************************************************************************* parameter functions *********************************************************************************
void MotionControl::setMotorParams()
{
    // Default-values: Performance version
    motorParams_.gearRatio      = this->declare_parameter("gearRatio", 72.f);            
    motorParams_.encoderRatio   = this->declare_parameter("encoderTicksPerRev", 2048.f);
    motorParams_.rpmMax         = this->declare_parameter("maxRPM", 90.f);                

    RCLCPP_INFO(this->get_logger(), "MotionControl::setMotorParams() -> gear ratio is set to:                       %.2f", motorParams_.gearRatio );
    RCLCPP_INFO(this->get_logger(), "MotionControl::setMotorParams() -> encoder ratio is set to:                    %.2f", motorParams_.encoderRatio);
    RCLCPP_INFO(this->get_logger(), "MotionControl::setMotorParams() -> the maximum speed of the motors are set to: %.2f rpm", motorParams_.rpmMax);
}



void MotionControl::setChassisParams()
{
    // Default-value: IOTBot v2
    chassisParams_.track            = this->declare_parameter("track", 0.35f);
    // Default-value: IOTBot v2 
    chassisParams_.wheelBase        = this->declare_parameter("wheelBase", 0.24f);
    // Default-value: d=17cm RC-Wheels
    chassisParams_.wheelDiameter    = this->declare_parameter("wheelDiameter", 0.17f);

    chassisParams_.direction        = 1; // TODO: Manuel Kreutz: set following magic numbers to enum class https://stackoverflow.com/questions/18335861/why-is-enum-class-preferred-over-plain-enum

    chassisParams_.chFrontLeft      = 2; 
    chassisParams_.chFrontRight     = 1;
    chassisParams_.chRearLeft       = 3;
    chassisParams_.chRearRight      = 0;

    RCLCPP_INFO(this->get_logger(), "MotionControl::setChassisParams() -> track of the robot is set to:             %.2f m", chassisParams_.track);
    RCLCPP_INFO(this->get_logger(), "MotionControl::setChassisParams() -> wheelbase of the robot is set to:         %.2f m", chassisParams_.wheelBase );
    RCLCPP_INFO(this->get_logger(), "MotionControl::setChassisParams() -> wheel diameter of the robot is set to:    %.2f m", chassisParams_.wheelDiameter);
}



void MotionControl::calculateRobotProperties()
{
    // ensure that the direction parameter is set properly (either 1 or -1)
    if (0 < chassisParams_.direction)
    {
        chassisParams_.direction = 1;
    }
    else
    {
        chassisParams_.direction = -1;
    }

    conversionParams_.rad2rpm   = (chassisParams_.wheelBase + chassisParams_.track) / chassisParams_.wheelDiameter; // (lx+ly)/2 * 1/r
    conversionParams_.rpm2rad   = 1.f / conversionParams_.rad2rpm;

    conversionParams_.ms2rpm    = 60.f / (chassisParams_.wheelDiameter * M_PI);
    conversionParams_.rpm2ms    = 1.f / conversionParams_.ms2rpm;

    conversionParams_.vMax      = motorParams_.rpmMax * conversionParams_.rpm2ms;
    conversionParams_.omegaMax  = motorParams_.rpmMax * conversionParams_.rpm2rad;

    RCLCPP_INFO(this->get_logger(), "MotionControl::calculateRobotProperties() -> initialized IotBot with vMax:     %.2f m/s", conversionParams_.vMax );
    RCLCPP_INFO(this->get_logger(), "MotionControl::calculateRobotProperties() -> initialized IotBot with omegaMax: %.2f rad", conversionParams_.omegaMax); 
}
  


void MotionControl::normalizeValues()
{
    float rpmCurrentMax = motorParams_.rpmMax;
    
    for (auto & rpmIterator : rpm_)
    {
        // possibility to flip directions
        rpmIterator *= chassisParams_.direction;
        
        // normalize values, if any value exceeds the maximum
        if (std::abs(rpmIterator) > motorParams_.rpmMax)
        {
            rpmCurrentMax = std::abs(rpmIterator);        
        }
    }

    if (rpmCurrentMax > motorParams_.rpmMax)
    {
        float factor = motorParams_.rpmMax / rpmCurrentMax;

        for (auto & rpmIterator : rpm_)
        {
            rpmIterator *= factor;
        }
    }
}



// ********************************************************************************* callback functions *********************************************************************************
void MotionControl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
{
    // Assignment of joystick axes to motor commands
    float fwd      = joyMsg->axes[1];               // Range of values [-1:1]
    float left     = joyMsg->axes[0];               // Range of values [-1:1]
    float turn     = joyMsg->axes[3];               // Range of values [-1:1]
    float throttle = (joyMsg->axes[5] + 1.f) / 2.f; // Range of values [0:1]

    geometry_msgs::msg::Twist velMsg;

    velMsg.linear.x    = static_cast<double>(throttle * fwd  * conversionParams_.vMax);
    velMsg.linear.y    = static_cast<double>(throttle * left * conversionParams_.vMax);
    velMsg.linear.z    = 0.0;

    velMsg.angular.x   = 0.0;
    velMsg.angular.y   = 0.0;
    velMsg.angular.z   = static_cast<double>(throttle * turn * conversionParams_.omegaMax);

    velPublisher_->publish(velMsg);
}



// ********************************************************************************* publish functions *********************************************************************************
void MotionControl::publishRpm()
{
    iotbot_interface::msg::RotationSpeed msgRpm;
    
    msgRpm.front_left_rpm  = rpm_[chassisParams_.chFrontLeft];
    msgRpm.front_right_rpm = rpm_[chassisParams_.chFrontRight];
    msgRpm.rear_left_rpm   = rpm_[chassisParams_.chRearLeft];
    msgRpm.rear_right_rpm  = rpm_[chassisParams_.chRearRight];

    rpmPublisher_->publish(msgRpm);
}


} // namespace iotbot