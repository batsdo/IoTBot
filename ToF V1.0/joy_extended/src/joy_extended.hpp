#ifndef JOY_EXTENDED__JOY_EXTENDED_HPP_
#define JOY_EXTENDED__JOY_EXTENDED_HPP_

#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace iotbot {
    class Joy_Extended : public rclcpp::Node{
        public:

        Joy_Extended(); //constructor

        ~Joy_Extended() = default; //deconstructor

        float tof[4];

        protected:
         void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg);

         void tofCallback(const std_msgs::msg::Float32MultiArray::SharedPtr tofMsg);

        /* void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg); */ 

         void publishJoy_Extended();

        private:

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              joySubscriber_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   tofSubscriber_;

        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr                      joyPublisher_;
    };
}


#endif //JOY_EXTENDED__JOY_EXTENDED_H_