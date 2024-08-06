//
// Created by dunyu on 24-6-20.
//

#include <memory>
#include <chrono> // 这里用来配置时间间隔变量
#include <Eigen/Dense>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "muscle_cpp/lib_signal.h"
//#include "muscle_interfaces/msg/muscle_state.hpp"

class Signal_sin_generator : public  rclcpp::Node {
public:
    Signal_sin_generator(rclcpp::QoS Publisher_QOS,double Sample_period, Sin_Params Signal_params) :
    // sub_classes init
    Node("Signal_sin_Generator"),
    my_signal(Sample_period, Signal_params)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("target",Publisher_QOS);
        timer_ = this->create_wall_timer(this->my_signal.sample_time.chrono_ms, std::bind(&Signal_sin_generator::timer_callback, this));

    }
    Signal_Sin my_signal;
    //publisher message
    std_msgs::msg::Float64 pub_msg;
    rclcpp::Clock clock;
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback(){
        my_signal.step();
        pub_msg.data = my_signal.state[0];
        publisher_->publish(pub_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::QoS pub_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    pub_qos.history(rclcpp::HistoryPolicy::KeepLast);
    pub_qos.keep_last(5);
    pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    // create node
    Sin_Params sin_params{};
    sin_params.Amp = 100;
    sin_params.Fre = 10;
    sin_params.Mar = 100;
    sin_params.Pha = 0;
    auto node = std::make_shared<Signal_sin_generator>(10,0.001,sin_params);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}