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
    sequence = 0;
    sequence_list ={sequence};
    time_global = 0.0;
    time_stamp_list={time_global};
    signal_data = this->search(time_global);
    signal_data_list={signal_data};

}

double Signal::search (double Time){
    return 0*Time;
}

//
double Signal::step(){
    this->sequence += 1;
    this->sequence_list.emplace_back(this->sequence);

    this->time_global += this->sample_time.period;
    this->time_stamp_list.emplace_back(this->time_global);

    this->signal_data = this->search(this->time_global);
    this->signal_data_list.emplace_back(this->signal_data);
    return this->signal_data;
}

Signal_sin::Signal_sin(double Sample_Period,Signal_sin_Params Params)
    : Signal(Sample_Period) {
    this->params = Params;
    this->reset();
}

double Signal_sin::search(double Global_Time) {
    double return_value;
    return_value = this->params.Amp * sin(this->params.Fre * Global_Time + this->params.Pha) + this->params.Margin;
    return return_value;
}

//
Signal_step::Signal_step(double Sample_Period, Signal_step_Params Params)
    : Signal(Sample_Period){
    this->params = Params;
    this->reset();
}

double Signal_step::search(double Global_Time) {
    double return_value = 0;
    int data_index =0;
    int no_coordinates = this->params.time_coordinates.size();
    double relative_time = Global_Time - this->params.period*floor(Global_Time/this->params.period);
    for (int i=0;i<no_coordinates;i+=1){
        if( (relative_time/this->params.period) >= this->params.time_coordinates[i]){
            data_index = i;
        }
    }
    return_value = this->params.signal_coordinates[data_index];
    return return_value;
}