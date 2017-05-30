//
//  PathFinder.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef PathFinder_hpp
#define PathFinder_hpp

#include "GLMain.hpp"


using namespace std;

class  PathFinder {
private:
    PathFinder(void) {}
public:
    string ResourcesPath;
    
    static PathFinder& getInstance(void);
};

#endif /* PathFinder_hpp */
