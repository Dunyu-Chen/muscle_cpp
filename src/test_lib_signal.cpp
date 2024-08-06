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
    double sample_period = 0.001;
    double m = 1;
    double b = 1;
    double k = 1;
    double A_list[4] = {0,-k/m,1,-b/m};
    double B_list[2] = {0,1/m};
    Eigen::Map<Eigen::MatrixXd> A_mat(A_list,2,2);
    Eigen::Map<Eigen::MatrixXd> B_mat(B_list,2,1);
    Linear_System msd_system (sample_period,2,1);
    msd_system.set_A_matrix(A_mat.matrix());
    msd_system.set_B_matrix(B_mat.matrix());
    Sin_Params params{};
    params.Amp = 1;
    params.Fre = 1;
    params.Pha = 0;
    params.Mar = params.Amp;
    Signal_Sin target_sin(sample_period,params);
    PID_Incremental controller(sample_period);
    controller.set_params(500,0.01,80000);

    std::vector<double> state;
    std::vector<double> time;
    std::vector<double> error;
    std::vector<double> input;
    std::vector<double> target;
    target.emplace_back(target_sin.state[0]);
    state.emplace_back(msd_system.state_vector[0]);
    time.emplace_back(msd_system.global_time);
    error.emplace_back(controller.error_buffer[0]);
    input.emplace_back(controller.output_vector[0]);
    for (int i =0;i<20000;i++){
        controller.observe(target_sin.state,msd_system.state_vector.row(0));
        msd_system.step(controller.step());
        target_sin.step();

        target.emplace_back(target_sin.state[0]);
        error.emplace_back(controller.error_buffer[0]);
        input.emplace_back(controller.output_vector[0]);
        state.emplace_back(msd_system.state_vector[0]);
        time.emplace_back(msd_system.global_time);
    }
    plt::plot(time,state);
    plt::plot(time,target);
    plt::plot(time,error);
    plt::show();
}