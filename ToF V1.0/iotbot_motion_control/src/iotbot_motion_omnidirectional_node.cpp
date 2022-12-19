#include "iotbot_motion_control.hpp"


namespace iotbot {


/**
 * @class MotionOmnidirectionalNode
 * @brief Chield class for specific motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 17.06.2021
 */
class MotionOmnidirectionalNode : public MotionControl
{
public:
    /**
     * constructor
     */
    MotionOmnidirectionalNode()
    {
        velSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                              rclcpp::SensorDataQoS(),
                                                                              std::bind(&MotionOmnidirectionalNode::velocityCallback, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                        this,
                                                                                        std::placeholders::_1));  
    }


    /**
     * destructor
     */
    ~MotionOmnidirectionalNode() = default;
    
private:
    /**
     * callback function for the subscription of the velocity message
     * @param[in] cmdMsg specifies the linear and angular parameters
     */
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmdMsg) override
    {     
        calculateKinematic(cmdMsg);
        normalizeValues();

        publishRpm();
    }


    /**
     * calculate the specific kinematics of the used omnidirectional wheels
     * @param[in] cmdMsg specifies the linear and angular parameters
     */
    void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmdMsg) override
    {
        // for mecanum wheel mode  
        rpm_[chassisParams_.chFrontLeft]  = 10.f * (( 2.f / chassisParams_.wheelDiameter) * (   cmdMsg->linear.x - cmdMsg->linear.y )) - (conversionParams_.rad2rpm * cmdMsg->angular.z);
        rpm_[chassisParams_.chFrontRight] = 10.f * (( 2.f / chassisParams_.wheelDiameter) * ( - cmdMsg->linear.x - cmdMsg->linear.y )) - (conversionParams_.rad2rpm * cmdMsg->angular.z);
        rpm_[chassisParams_.chRearLeft]   = 10.f * (( 2.f / chassisParams_.wheelDiameter) * (   cmdMsg->linear.x + cmdMsg->linear.y )) - (conversionParams_.rad2rpm * cmdMsg->angular.z);
        rpm_[chassisParams_.chRearRight]  = 10.f * (( 2.f / chassisParams_.wheelDiameter) * ( - cmdMsg->linear.x + cmdMsg->linear.y )) - (conversionParams_.rad2rpm * cmdMsg->angular.z);  
    }
}; // MotionOmnidirectionalNode


} // namespace iotbot



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iotbot::MotionOmnidirectionalNode>());
  rclcpp::shutdown();

  return 0;
}
