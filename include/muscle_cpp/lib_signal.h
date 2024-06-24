//
// Created by dunyu on 24-6-12.
//

#ifndef MUSCLE_CPP_LIB_SIGNAL_H
#define MUSCLE_CPP_LIB_SIGNAL_H
#include <vector>
#include <chrono>
#include "muscle_cpp/lib_signal.h"

struct Node_Sample_Time{
    double period; // in s
    std::chrono::microseconds chrono_us{};
    std::chrono::milliseconds chrono_ms{};
};

Node_Sample_Time init_node_sample_time(double Period);

class Signal{
public:
    virtual double search (double Time_stamp);
    double step ();
    Node_Sample_Time sample_time;
    std::vector<double> time_stamp_list;
    double time_global;
    std::vector<int> sequence_list;
    int sequence;
    double signal_data;
    std::vector<double> signal_data_list;
    void reset();
    explicit Signal(double Sample_Period);

private:

};

// signal sin
struct Signal_sin_Params {
    double Amp;
    double Pha;
    double Fre;
    double Margin;
};

class Signal_sin : public Signal {
public:
    Signal_sin(double Sample_Period, Signal_sin_Params Params);
    Signal_sin_Params params;
    double search(double Global_Time) override;
};

struct Signal_step_Params {
    std::vector<double> time_coordinates;
    std::vector<double> signal_coordinates;
    double period;
};

class Signal_step : public Signal {
public:
    Signal_step(double Sample_Period, Signal_step_Params Params);
    Signal_step_Params params;
    double search(double Global_Time) override;
};
#endif //MUSCLE_CPP_LIB_SIGNAL_H
