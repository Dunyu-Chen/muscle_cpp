//
// Created by dunyu on 24-6-12.
//

#ifndef MUSCLE_CPP_LIB_SIGNAL_H
#define MUSCLE_CPP_LIB_SIGNAL_H
#include <vector>
#include <chrono>

struct Node_Sample_Time{
    double period; // in us
    std::chrono::microseconds chrono_us{};
    std::chrono::milliseconds chrono_ms{};
};

Node_Sample_Time init_node_sample_time(double Period);

class Signal{
public:
    virtual double search (double Time_stamp);
    Node_Sample_Time sample_time;
    std::vector<double> time_stamp_list;
    double time_stamp;
    std::vector<uint> sequence_list;
    uint sequence;
    double signal_data;
    std::vector<double> signal_data_list;
    void reset();
    explicit Signal(double Sample_Period);

private:

};

struct Signal_sin_Params {
    double Amp;
    double pha;
    double Fre;
    double Margin;
};

class Signal_sin : Signal {
public:
    Signal_sin(double Sample_Period, Signal_sin_Params Params);
    Signal_sin_Params Params;
    double search(double Time_Stamp) override;
};
#endif //MUSCLE_CPP_LIB_SIGNAL_H
