//
//  Utils.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/22.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//
#ifndef Utils_h
#define Utils_h

#define Debug 1

#include <chrono>

#define CALC_START() chrono::system_clock::time_point  st, en;\
st = chrono::system_clock::now();

#define CALC_END(frase) en = chrono::system_clock::now();\
double elapsed = chrono::duration_cast<std::chrono::milliseconds>(en-st).count();\
cout << frase << elapsed << "ms" << endl;

#define CALC_ENDV2(frase) en = chrono::system_clock::now();\
double elapsed = chrono::duration_cast<std::chrono::milliseconds>(en-st).count();\
stringstream ss;\
ss << frase << elapsed << "ms";\
{string s_calc = ss.str();\
dispatch_async(dispatch_get_main_queue(), ^{\
    trackingTimeLabel.text =  [NSString stringWithUTF8String:s_calc.c_str()];\
});}

#define Infomation(inf) {string s_inf = inf;\
dispatch_async(dispatch_get_main_queue(), ^{\
    informationView.text =  [NSString stringWithUTF8String:s_inf.c_str()];\
});}

#define State(state) {string s_state = state;\
dispatch_async(dispatch_get_main_queue(), ^{\
currentStateLabel.text =  [NSString stringWithUTF8String:s_state.c_str()];\
});}


#endif /* Utils_h */
