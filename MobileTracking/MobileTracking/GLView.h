//
//  GLView.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef GLView_h
#define GLView_h

#include "ATAM.hpp"
#include "State.hpp"
#include "GLEngine.hpp"
#include "ViewController.h"
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#include "GLMain.hpp"

@class ViewController;

@interface GLView : UIView
{
    GLint               backingWidth;
    GLint               backingHeight;
    GLuint              frameBuffer;
    GLuint              renderBuffer;
    GLuint              depthBuffer;
    
    GLEngine engine;
}

@property (nonatomic, retain) IBOutlet ViewController *controller;
@property (nonatomic) NSInteger animationFrameInterval;
@property (nonatomic, getter=isAnimating) bool animating;
@property (nonatomic, retain) EAGLContext *glContext;
@property (nonatomic, retain) CADisplayLink *displayLink;


- ( void )Update;//:(CATAM&)atam

@end


#endif /* GLView_h */
