//
// Created by dunyu on 24-6-13.
//

#include "muscle_cpp/lib_signal.h"
#include "muscle_cpp/matplotlibcpp.h"
#include "vector"

namespace plt = matplotlibcpp;

int main(int argc, char * argv[]){
    Signal_sin_Params params{};
    params.Amp = 1;
    params.Fre = 1;
    params.Pha = 0;
    params.Margin = 1;

    Signal_sin test_signal(0.001,params) ;

    for (int i = 0; i < 10000; i++){
        test_signal.step();
    }

    Signal_step_Params params1{};
    params1.time_coordinates = {0,0.2,0.4,0.8};
    params1.signal_coordinates = {0,2,4,8};
    params1.period = 1; //s

    Signal_step test_signal2(0.001,params1);
    for (int i = 0; i<10000;i++){
        test_signal2.step();
    }

    //plt::plot<>(test_signal.time_stamp_list,test_signal.signal_data_list);
    plt::plot<>(test_signal2.time_stamp_list,test_signal2.signal_data_list);
    plt::show();
    return 0;
}
