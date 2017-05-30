//
//  ParticleFileter.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/21.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef ParticleFileter_hpp
#define ParticleFileter_hpp

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include "CalcDistance.hpp"
#include "Utils.hpp"

using namespace std;
using namespace cv;
using namespace glm;

class Particle{
public:
    float weight = 1.0;
    vec2 pos;
    vec2 vel;
    
    Particle(vec2 pos, vec2 vel);
    void PrintOut();
};

class ParticleFilter{
private:
    const int n_sample = 80;
    vector<Particle> particle_vector;
    
    vec2 area;
    const int field =  10;
    
    Mat search_image;
    vec2 image_size;
    
    vector<int> upper;
    vector<int> lower;
    
    vec2 position;
public:
    CalcDistance cd;
    
    ParticleFilter(){};
    
    void Define(vec2 position, Mat image, vec2 space);
    
    void Redifine(vec2 position);
    void Predict();
    void CalcWeight(Mat &input_image);
    float Likelihood(vec2 position, Mat &inputimage);
    void Resampling();
    vec2 Measure();
    
    void drawResult(Mat &image);
    
    vector<Particle> GetParticle();    
};

#endif /* ParticleFileter_hpp */
