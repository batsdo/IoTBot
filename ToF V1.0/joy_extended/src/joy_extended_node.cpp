#include "joy_extended.hpp"


namespace iotbot{
Joy_Extended::Joy_Extended() : rclcpp::Node("joy_extended", rclcpp::NodeOptions()){

        joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",rclcpp::SystemDefaultsQoS(),std::bind(&Joy_Extended::joyCallback,this,std::placeholders::_1));
        tofSubscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("iotbot/tof",rclcpp::SystemDefaultsQoS(),std::bind(&Joy_Extended::tofCallback,this,std::placeholders::_1));

        joyPublisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy_extended",rclcpp::SystemDefaultsQoS());
}


void Joy_Extended::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg){
    float fwd       = joyMsg->axes[1];
    float bwd       = joyMsg->axes[2];
    float turn      = joyMsg->axes[3];
    float throttle  = joyMsg->axes[5];

    sensor_msgs::msg::Joy joyMsg_;

    if (tof[0] <= 0.03 || tof[1] <= 0.03 ) {
        joyMsg_.axes[1]     = 0;
        joyMsg_.axes[2]     = bwd;
        joyMsg_.axes[3]     = 0;
        joyMsg_.axes[5]     = throttle;

    } else if(tof[2] <= 0.03 || tof[3] <= 0.03) {
        joyMsg_.axes[1]     = fwd;
        joyMsg_.axes[2]     = 0;
        joyMsg_.axes[3]     = 0;
        joyMsg_.axes[5]     = throttle;
    } else {
        joyMsg_.axes[1]     = fwd;
        joyMsg_.axes[2]     = bwd;
        joyMsg_.axes[3]     = turn;
        joyMsg_.axes[5]     = throttle;
    }

    joyPublisher_->publish(joyMsg_);


    
}

void Joy_Extended::tofCallback(const std_msgs::msg::Float32MultiArray::SharedPtr tofMsg){
    tof[0] = tofMsg->data[0];
    tof[1] = tofMsg->data[1];
    tof[2] = tofMsg->data[3];
    tof[5] = tofMsg->data[4];
}
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<iotbot::Joy_Extended>());
    rclcpp::shutdown();

    return 0;
}