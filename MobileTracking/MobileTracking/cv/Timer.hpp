//
//  Timer.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/07.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Timer_hpp
#define Timer_hpp

/*!
 @file		Timer.h
 @brief		header of CTimer
 */

#pragma once

#include <string>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#endif

/*!
 @class		CTimer
 @brief		for timer
 */
class CTimer
{
public:
    int Pop(const bool flag = false);
    void Push(const std::string &name);
    
private:
    
    /*!
     @struct		sTask
     @brief		task name
     */
    struct sTask{
        std::string name;
        
#ifdef _WIN32
        LARGE_INTEGER stime;
#else
        std::chrono::system_clock::time_point stime;
#endif
    };
    
    std::vector<sTask> mTasks;		//!< task
};

#endif /* Timer_hpp */
