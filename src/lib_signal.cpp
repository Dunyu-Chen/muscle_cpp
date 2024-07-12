#include <utility>

#include "muscle_cpp/lib_signal.h"

Node_Sample_Time init_node_sample_time(double Period){
    Node_Sample_Time return_struct;
    return_struct.period = Period; // S
    return_struct.chrono_us = std::chrono::microseconds(static_cast<int>(std::round(Period * 1000000.0)));
    return_struct.chrono_ms = std::chrono::milliseconds (static_cast<int>(std::round(Period * 1000.0)));
    return return_struct;
}
Signal::Signal(double Sample_time,int Input_dim,int Output_dim) {
    input_vector.resize(Input_dim);
    output_vector.resize(Output_dim);
    this->sample_time = init_node_sample_time(Sample_time);
    Eigen::VectorXd init_state(Output_dim);
    init_state.setZero();
    this->reset(init_state);
}

void Signal::reset(Eigen::VectorXd init_state) {
    this->Time_global = 0.0;
    this->sequence = 0;
    output_vector = init_state.matrix();
}

void Signal::step(Eigen::VectorXd Input) {
    this->Time_global += this->sample_time.period;
    this->sequence += 1;
    this->input_vector = Input.matrix();
}

Signal_sin::Signal_sin(double Sample_time,Sin_Params Params) : Signal(Sample_time,1,1){
    params = Params;
}

double Signal_sin::search(double Time) const {
    double return_value= params.Amp * sin(Time*params.Fre + params.Pha) + params.Mar;
    return return_value;
}

void Signal_sin::step(Eigen::VectorXd Input){
    Signal::step(Input);
    output_vector[0] = this->search(Time_global);
}

Signal_stair::Signal_stair(double Sample_time, Stair_Params Params) : Signal(Sample_time,1,1){
    params = Params;
}

double Signal_stair::search(double Time) {
    double return_value = 0;
    int data_index =0;
    int no_coordinates = this->params.time_list.size();
    double relative_time = Time - this->params.period*floor(Time/this->params.period);
    for (int i=0;i<no_coordinates;i+=1){
        if( (relative_time/this->params.period) >= this->params.time_list[i]){
            data_index = i;
        }
    }
    return_value = this->params.data_list[data_index];
    return return_value;
}

void Signal_stair::step(Eigen::VectorXd Input) {
    Signal::step(Input);
    output_vector[0] = this->search(Time_global);

}


PID_Incremental::PID_Incremental(PID_Params Params){
    params = Params;
    coe1 = params.K_p+params.K_i+params.K_d;
    coe2 = -params.K_p-2*params.K_d;
    coe3 = params.K_d;
    reset();
}

void PID_Incremental::reset() {
    output_buffer[0] = output_buffer[1] = 0.0;
    error_buffer[0] = error_buffer[1] =error_buffer[2] = 0.0;
}

double PID_Incremental::step(double Target, double State) {
    error_buffer[2] = error_buffer[1];
    error_buffer[1] = error_buffer[0];
    error_buffer[0] = Target - State;

    output_buffer[1] = output_buffer[0];
    output_buffer[0] = output_buffer[1] + coe1*error_buffer[0] + coe2*error_buffer[1]+coe3*error_buffer[2];
    return output_buffer[0];
}

Linear_system::Linear_system(double Sample_time,int Input_dim,int State_dim)
: Signal(Sample_time,Input_dim,State_dim){
    input_dim = Input_dim;
    state_dim = State_dim;
    A_matrix.resize(State_dim,State_dim);
    B_matrix.resize(State_dim,Input_dim);


}

void Linear_system::set_A_matrix_from_list(double *Data_list) {
    Eigen::Map<Eigen::MatrixXd> matrixMap(Data_list, state_dim, state_dim);
    A_matrix = matrixMap.matrix();
}

void Linear_system::set_B_matrix_from_list(double *Data_list) {
    Eigen::Map<Eigen::MatrixXd> matrixMap(Data_list, state_dim, input_dim);
    B_matrix = matrixMap.matrix();
}

void Linear_system::explicit_step(Eigen::VectorXd Input) {
    Signal::step(Input);
    output_vector = output_vector+ sample_time.period*(A_matrix*output_vector+B_matrix*input_vector);
}

//ld;sfaj;lkfdsja
