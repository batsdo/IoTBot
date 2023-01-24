#include "iotbot_motion_control.hpp"


namespace iotbot {


/**
 * @class MotionDifferentialNode
 * @brief Chield class for specific motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 17.06.2021
 */
class MotionDifferentialNode : public MotionControl
{
public:
    /**
     * constructor
     */
    MotionDifferentialNode()
    {
        velSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                              rclcpp::SensorDataQoS(),
                                                                              std::bind(&MotionDifferentialNode::velocityCallback, // TODO: Manuel Kreutz: change bind to lambda functions
                                                                                        this,
                                                                                        std::placeholders::_1));  
    }


    /**
     * destructor
     */
    ~MotionDifferentialNode() = default;

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
     * calculate the specific kinematics of the used differential wheels
     * @param[in] cmdMsg specifies the linear and angular parameters
     */
    void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmdMsg) override
    {
        // For differantial mode
        float rpmFwd   = cmdMsg->linear.x  * conversionParams_.ms2rpm;
        float rpmLeft  = cmdMsg->linear.y  * conversionParams_.ms2rpm;
        float rpmOmega = cmdMsg->angular.z * conversionParams_.rad2rpm;       
        
        rpmLeft = 0.f; // deactivate movement in y-direction
        
        // leading signs -> see derivation: Stefan May, Skriptum "Mobile Robotik"
        rpm_[chassisParams_.chFrontLeft]  =  rpmFwd - rpmLeft - rpmOmega;
        rpm_[chassisParams_.chFrontRight] = -rpmFwd - rpmLeft - rpmOmega;
        rpm_[chassisParams_.chRearLeft]   =  rpmFwd + rpmLeft - rpmOmega;
        rpm_[chassisParams_.chRearRight]  = -rpmFwd + rpmLeft - rpmOmega;
    }
}; // MotionDifferentialNode


} // namespace iotbot



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<iotbot::MotionDifferentialNode>());
    rclcpp::shutdown();

    return 0;
}
