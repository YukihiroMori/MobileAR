//
//  ParticleFileter.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/21.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ParticleFilter.hpp"

Particle::Particle(vec2 position, vec2 velocity){
    pos = position;
    vel = velocity;
}

void Particle::PrintOut(){
    cout << "Position : " << to_string(pos) << endl;
    cout << "Velocity : " << to_string(vel) << endl;
}

void ParticleFilter::Define(vec2 position, Mat image , vec2 space){
    area = space;
    image_size = vec2(image.size().width,image.size().height);
    
    for(int t = 0; t < n_sample; t++){
        Particle particle(position + image_size / 2.0f, vec2(1.0f));
        particle_vector.push_back(particle);
    }
    
    upper = {static_cast<int>(area.x),static_cast<int>(area.y),field,field};
    lower = {0,0,-field,-field};
    search_image = image;
    
    cd.setTargetHist(search_image);
}

void ParticleFilter::Redifine(vec2 position){
    particle_vector.clear();
    
    for(int t = 0; t < n_sample; t++){
        Particle particle(position + image_size / 2.0f, vec2(1.0f));
        particle_vector.push_back(particle);
    }
}

void ParticleFilter::Predict(){
    for(int index = 0; index < n_sample; index++){
        random_device rd;
        
        mt19937 mt(rd());
        uniform_real_distribution<float> score(-field / 2.0f, field / 2.0f);
        
        int n[4];
        n[0] = score(mt);
        n[1] = score(mt);
        n[2] = score(mt);
        n[3] = score(mt);
        
        Particle *p = &particle_vector[index];
        p->pos.x += p->vel.x + n[0];
        p->pos.y += p->vel.y + n[1];
        p->vel.x += n[2];
        p->vel.y += n[3];
        
        p->pos.x = clamp(p->pos.x, (float)lower[0], (float)upper[0]);
        p->pos.y = clamp(p->pos.y, (float)lower[1], (float)upper[1]);
        p->vel.x = clamp(p->vel.x, (float)lower[2], (float)upper[2]);
        p->vel.y = clamp(p->vel.y, (float)lower[3], (float)upper[3]);
    }
}

void ParticleFilter::CalcWeight(Mat &input_image){
    
    float sum_weitgh = 0.0;
    for(int index = 0; index < particle_vector.size(); index++){
        vec2 pos = particle_vector[index].pos;
        
        float weight = Likelihood(pos, input_image);
        
        particle_vector[index].weight = weight;
        sum_weitgh += weight;
    }
    
    for(int index = 0; index < particle_vector.size(); index++){
        particle_vector[index].weight /= sum_weitgh;
    }

}

float ParticleFilter::Likelihood(vec2 position, Mat &input_image){
    vec2 min_pos = position - image_size / 2.0f;
    vec2 max_pos = position + image_size / 2.0f;
    
    if(!isArea(area, position, image_size)){
        return 0.0001;
    }
    
    Mat part_image(input_image,createRect(position, image_size));

    float dist = cd.calcDistance(part_image);
    return dist;
}

void ParticleFilter::Resampling(){
    vector<float> sum_weights(n_sample);
    sum_weights[0] = particle_vector[0].weight;
    for(int index = 1; index < n_sample; index++){
        sum_weights[index] = sum_weights[index-1] + particle_vector[index].weight;
    }
    
    vector<Particle> copy_particle_vector(particle_vector);
    for(int index = 0; index < n_sample; index++){
        float weight_threshold = (float)(rand()%10000) / 10000.0;
        for(int k = 0; k < n_sample; k++){
            if(weight_threshold > sum_weights[k]){
                continue;
            }else{
                particle_vector[index] = copy_particle_vector[k];
                particle_vector[index].weight = 0.0;
                break;
            }
        }
    }
}

vec2 ParticleFilter::Measure(){
    float x = 0.0;
    float y = 0.0;
    float weight = 0.0;
    
    for(int index = 0; index < n_sample; index++){
        auto p = particle_vector[index];
        x += p.pos.x * p.weight;
        y += p.pos.y * p.weight;
    }
    
    position.x = x;
    position.y = y;
    
    return position;
}

void ParticleFilter::drawResult(Mat &image){
    double angle = 30;
    ellipse(image, cv::Point(position.x, position.y), cv::Size(10, 10), angle, angle, angle+360, Scalar(0,200,0,255), 1, CV_AA );
    
    for(int i = 0; i < particle_vector.size(); i++){
        ellipse(image, cv::Point(particle_vector[i].pos.x,particle_vector[i].pos.y), cv::Size(1, 1), angle, angle, angle+360, Scalar(0,200,0,255), 1, CV_AA );
    }
}

vector<Particle> ParticleFilter::GetParticle(){
    return particle_vector;
}


































