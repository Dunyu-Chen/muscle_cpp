//
// Created by dunyu on 24-6-13.
//

#include "muscle_cpp/lib_signal.h"

int main(int argc, char * argv[]){
    Signal_sin_Params params{};
    params.Amp = 1;
    params.Fre = 1;
    params.Pha = 0;
    params.Margin = 1;

    Signal_sin test_signal(0.001,params) ;

    for (int i = 0; i < 100; i++){
        test_signal.step();
    }

    Signal_step_Params params1{};
    params1.time_coordinates = {0,0.4,0.8};
    params1.signal_coordinates = {1,4,8};
    params1.period = 0.01; //s

    Signal_step test_signal2(0.001,params1);
    for (int i = 0; i<100;i++){
        test_signal2.step();
    }

    return 0;
}
