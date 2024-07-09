//
// Created by dunyu on 24-6-13.
//

#include "muscle_cpp/lib_signal.h"
#include "muscle_cpp/matplotlibcpp.h"
#include "vector"
#include <Eigen/Dense>
#include <torch/torch.h>
#include <iostream>

namespace plt = matplotlibcpp;

int main(){
    torch::Tensor tensor = torch::rand({2, 3});
    std::cout << tensor << std::endl;
    Sin_Params params{};
    params.Amp = 1;
    params.Fre = 1;
    params.Pha = 0;
    params.Mar = 1;

    Signal_sin test_signal(0.001,params) ;

    for (int i = 0; i < 10000; i++){
        Eigen::VectorXd zero_vector(1);
        test_signal.step(zero_vector);
    }

    Stair_Params params1{};
    params1.time_list = {0,0.2,0.4,0.6,0.8};
    params1.data_list = {0,2,4,6,8};
    params1.period = 1; //s

    Signal_stair test_signal2(0.001,params1);

    std::vector<double> time_stamp_list = {test_signal2.Time_global};
    std::vector<double> data_list = {test_signal2.output_vector[0]};

    for (int i = 0; i<10000;i++){
        Eigen::VectorXd zero_vector(1);
        test_signal2.step(zero_vector);
        time_stamp_list.emplace_back(test_signal2.Time_global);
        data_list.emplace_back(test_signal2.output_vector[0]);
    }

    //plt::plot<>(test_signal.time_stamp_list,test_signal.signal_data_list);
    plt::plot<>(time_stamp_list,data_list);
    plt::show();
    return 0;
}
