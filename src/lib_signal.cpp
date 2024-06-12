//
// Created by dunyu on 24-6-12.
//
#include <cmath>
#include "muscle_cpp/lib_signal.h"

Node_Sample_Time init_node_sample_time(double Period){
    Node_Sample_Time return_struct;
    return_struct.period = Period; // S
    return_struct.chrono_us = std::chrono::microseconds(static_cast<int>(std::round(Period * 1000000.0)));
    return_struct.chrono_ms = std::chrono::milliseconds (static_cast<int>(std::round(Period * 1000.0)));
    return return_struct;
}

Signal::Signal(double Sample_Period) {
    this->sample_time = init_node_sample_time(Sample_Period);
    this->reset();
}

void Signal::reset() {
    sequence_list ={};
    sequence = 0;
    time_stamp = 0.0;
    time_stamp_list={};
    signal_data_list={};
    signal_data = this->search(time_stamp);
}

double Signal::search (double Time_stamp){
    return 0;
}

Signal_sin::Signal_sin(
    double Sample_Period,
    Signal_sin_Params Params)
    : Signal(Sample_Period) {

}

double Signal_sin::search(double Time_Stamp) {
    double return_value;
    return_value = this->Params.Amp * sin(this->Params.Fre * Time_Stamp + this->Params.pha) + this->Params.Margin;
    return return_value;
}