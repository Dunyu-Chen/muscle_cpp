
#include <memory>
#include <chrono> // 这里用来配置时间间隔变量
#include <Eigen/Dense>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "muscle_cpp/lib_signal.h"
#include "muscle_interfaces/msg/muscle_state.hpp"

class Signal_stair_generator : public  rclcpp::Node {
public:
    Signal_stair_generator(rclcpp::QoS Publisher_QOS,double Sample_period, Stair_Params Signal_params) :
    // sub_classes init
            Node("Signal_stair_Generator"),
            signal(Sample_period,Signal_params)
    {
        //publisher_ = this->create_publisher<std_msgs::msg::Float64>("published_topic",Publisher_QOS);
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("target",Publisher_QOS);
        //pub_msg.header.frame_id = '0';
        timer_ = this->create_wall_timer(this->signal.sample_time.chrono_ms,std::bind(&Signal_stair_generator::timer_callback,this));

    }
    Signal_stair signal;
    //publisher message
    std_msgs::msg::Float64 pub_msg;
    //muscle_interfaces::msg::MuscleState pub_msg;
    rclcpp::Clock clock;
private:
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback(){
        Eigen::VectorXd zero_input(1);
        signal.step(zero_input);
        //pub_msg.header.stamp = clock.now();
        //pub_msg.position = signal.signal_data;
        pub_msg.data = signal.output_vector[0];
        publisher_->publish(pub_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::QoS pub_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    pub_qos.history(rclcpp::HistoryPolicy::KeepLast);
    pub_qos.keep_last(5);
    pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    // create node
    Stair_Params stair_params{};
    stair_params.period = 2.0;
    stair_params.time_list = {0,0.2,0.4,0.6,0.8};
    stair_params.data_list = {0,2,4,6,8};

    auto node = std::make_shared<Signal_stair_generator>(pub_qos,0.001,stair_params);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}