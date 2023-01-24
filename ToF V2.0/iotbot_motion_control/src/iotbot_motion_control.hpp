#ifndef IOTBOT_MOTION_CONTROL__MOTION_CONTROL_HPP_
#define IOTBOT_MOTION_CONTROL__MOTION_CONTROL_HPP_

#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "iotbot_interface/msg/rotation_speed.hpp"
#include "iotbot_robot_params.h"


namespace iotbot {


/**
 * @class MotionControl
 * @brief Basic class for motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 15.04.2021
 */
class MotionControl : public rclcpp::Node
{
public:
    /**
     * constructor
     */
    MotionControl();

    /**
     * destructor
     */
    ~MotionControl() = default;

protected:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  velSubscriber_; 

    MotorParams             motorParams_;
    ChassisParams           chassisParams_;
    RobotConversionParams   conversionParams_;

    std::array<float, 4> rpm_{0};


    /**
     * set the RPM values to normalized values
     */
    void normalizeValues();

    /**
     * publish the newly calculated RPM data
     */
    void publishRpm();

    /**
     * callback function for the subscription of the velocity message -> has to be implemented in the child class
     * @param[in] cmdMsg specifies the linear and angular parameters
     */
    virtual void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmdMsg) = 0;

    /**
     * calculate the specific kinematics of the used wheels -> has to be implemented in the child class
     * @param[in] cmdMsg specifies the linear and angular parameters
     */
    virtual void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmdMsg) = 0;

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              joySubscriber_; 
    
    rclcpp::Publisher<iotbot_interface::msg::RotationSpeed>::SharedPtr  rpmPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             velPublisher_;

    
    /**
     * set specific motor parameters
     */
    void setMotorParams();
    
    /**
     * set specific chassis parameters
     */
    void setChassisParams();

    /**
     * calculate some additionally needed parameters with the specified motor & chassis parameters
     */
    void calculateRobotProperties();

    /**
     * conversion from joy type to cmd_vel type
     * @param[in] joyMsg specifies the joy values from e.g. a PS4 controller
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg);
};


} // namespace iotbot


#endif // IOTBOT_MOTION_CONTROL__MOTION_CONTROL_HPP_
