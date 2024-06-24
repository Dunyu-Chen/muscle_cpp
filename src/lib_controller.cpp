//
// Created by dunyu on 24-6-21.
//

#include "muscle_cpp/lib_controller.h"

Discrete_Controller::Discrete_Controller() {
}

void Discrete_Controller::observe(std::vector<double> State, std::vector<double> Target) {
    this->state_vector = State;
    this->target_vector = Target;
}

std::vector<double> Discrete_Controller::step() {
    std::vector<double> return_value(1,0);
    return return_value;
}

PID_incremental::PID_incremental(PID_Params Params)
    : Discrete_Controller()
{   state_vector.resize(1);
    target_vector.resize(1);
    output_vector.resize(1);

    error_buffer.resize(3);
    output_buffer.resize(2);
    std::fill(error_buffer.begin(),error_buffer.end(),0.0);
    std::fill(output_buffer.begin(),output_buffer.end(),0.0);
    this->params = Params;
}

std::vector<double> PID_incremental::step() {
    error_buffer[2] = error_buffer[1];
    error_buffer[1] = error_buffer[0];
    error_buffer[0] = target_vector[0] - state_vector[0];
    output_buffer[1] = output_buffer[0];
    output_buffer[0] += params.P*(error_buffer[0]-error_buffer[1])+params.I*error_buffer[0]+params.D*(error_buffer[0]-2*error_buffer[1]+error_buffer[2]);
    output_vector[0] = output_buffer[0];
    return output_vector;
}

Linear_System::Linear_System(int State_dim, int Input_dim){
    input_dim = Input_dim;
    state_vector.resize(State_dim);
    input_vector.resize(Input_dim);
    A_matrix.resize(State_dim,State_dim);
    B_matrix.resize(State_dim,Input_dim);
}

void Linear_System::set_A_matrix_from_list(double *Element_list) {
    Eigen::Map<Eigen::MatrixXd> matrix (Element_list,state_dim,state_dim);
    A_matrix = matrix.matrix();
}

void Linear_System::set_B_matrix_from_list(double *Element_list) {
    Eigen::Map<Eigen::MatrixXd> matrix (Element_list,state_dim,input_dim);
    B_matrix = matrix.matrix();
}