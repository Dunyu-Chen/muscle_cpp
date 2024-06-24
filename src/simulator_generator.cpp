//
// Created by dunyu on 24-6-24.
//
#include "muscle_cpp/lib_signal.h"
#include "muscle_cpp/lib_controller.h"
#include "muscle_interfaces/msg/muscle_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"

class  Simu_generator : public  rclcpp::Node {
public:
    Simu_generator(double Sample_period,MSD_Params params) :
    Node("Simulator_Generator"),
    simulator(Sample_period,params){
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("pub_topic",10);
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>("sub_topic",rclcpp::SystemDefaultsQoS(),);

    }
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;


    MSD_Simulator simulator;
};