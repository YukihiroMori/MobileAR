//
//  GLView.m
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#import "GLView.h"
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>

@implementation GLView


+ (Class)layerClass
{
    return [ CAEAGLLayer class ];
}

- ( id ) initWithCoder:(NSCoder *)aDecoder
{
    self = [super initWithCoder:aDecoder];
    
    if (self)
    {
        self.glContext = [ [EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES3 ];
        [ EAGLContext setCurrentContext:self.glContext ];
        
        CAEAGLLayer* pGLLayer = ( CAEAGLLayer* )self.layer;
        
        pGLLayer.opaque = NO;
        
        pGLLayer.drawableProperties = [ NSDictionary dictionaryWithObjectsAndKeys:
                                       [ NSNumber numberWithBool:FALSE ],
                                       kEAGLDrawablePropertyRetainedBacking,
                                       kEAGLColorFormatRGBA8,
                                       kEAGLDrawablePropertyColorFormat,
                                       nil ];
        self.animating = NO;
        self.animationFrameInterval = 2;
    }
    
    ASSERT_GL(glEnable(GL_DEPTH_TEST));
    
    
    NSBundle *bundle = [NSBundle mainBundle];
    NSString *resourceDirectoryPath = [bundle bundlePath];
    
    engine.Path = [resourceDirectoryPath UTF8String];
    engine.Path += "/";
    
    engine.init();
    
    return self;
}

- (void)createBuffers
{
    ASSERT_GL(glGenFramebuffers(1, &frameBuffer));
    ASSERT_GL(glGenRenderbuffers(1, &renderBuffer));
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer));
    ASSERT_GL(glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer));
    [self.glContext renderbufferStorage:GL_RENDERBUFFER fromDrawable:(CAEAGLLayer *)self.layer];
    ASSERT_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderBuffer));
    ASSERT_GL(glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &backingWidth));
    ASSERT_GL(glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &backingHeight));
    
    ASSERT_GL(glGenRenderbuffers(1, &depthBuffer));
    ASSERT_GL(glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer));
    ASSERT_GL(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, backingWidth, backingHeight));
    ASSERT_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer));
}

- (void)destroyBuffers
{
    ASSERT_GL(glDeleteFramebuffers(1, &frameBuffer));
    frameBuffer = 0;
    ASSERT_GL(glDeleteRenderbuffers(1, &renderBuffer));
    renderBuffer = 0;
    
    if(depthBuffer)
    {
        ASSERT_GL(glDeleteRenderbuffers(1, &depthBuffer));
        depthBuffer = 0;
    }
}

- (void)layoutSubviews
{
    [EAGLContext setCurrentContext:self.glContext];
    
    [self destroyBuffers];
    [self createBuffers];
    
    ASSERT_GL(glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer));
    
    ASSERT_GL(glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &backingWidth));
    ASSERT_GL(glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &backingHeight));
    
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        NSLog(@"Failed to make complete framebuffer object %x", glCheckFramebufferStatus(GL_FRAMEBUFFER));
    ASSERT_GL(glViewport(0, 0, backingWidth, backingHeight));
    
    engine.camera.setSize(backingWidth, backingHeight);
    
    [self startAnimation];
    
}
- (void)startAnimation
{
    if (!self.animating)
    {
        self.displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(Update)];
        [self.displayLink setFrameInterval:self.animationFrameInterval];
        [self.displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        
        self.animating = YES;
    }
}
- (void)stopAnimation
{
    if (self.animating)
    {
        [self.displayLink invalidate];
        self.displayLink = nil;
        
        self.animating = NO;
    }
}

- ( void )Update//:( CATAM& )atam
{
    if(_controller->prossesor.atam.mState == STATE::TAM){
        sPose pose;
        _controller->prossesor.atam.transformToWorld(_controller->prossesor.atam.mPose, pose);
        
        dvec3 t = vec3(pose.tvec.at<double>(0),pose.tvec.at<double>(1),pose.tvec.at<double>(2));
        dvec3 r = vec3(pose.rvec.at<double>(0),pose.rvec.at<double>(1),pose.rvec.at<double>(2));
    
        engine.camera.position = t;
        engine.camera.rotation = r;
        
        cv::Mat rr;
        pose.rvec.at<double>(0) = -pose.rvec.at<double>(0);
        Rodrigues(pose.rvec, rr);
        
        vec4 pt3d = vec4(0.0,0.0,0.0,1.0);
        mat4 T;
        mat4 R;
        
        T = translate(mat4(1.0), vec3(pose.tvec.at<double>(0),
                      pose.tvec.at<double>(1),
                      pose.tvec.at<double>(2)));
        R = mat4(rr.at<double>(0),rr.at<double>(3),rr.at<double>(6),0,
                  rr.at<double>(1),rr.at<double>(4),rr.at<double>(7),0,
                  rr.at<double>(2),rr.at<double>(5),rr.at<double>(8),0,
                  0,0,0,1);
        
        engine.camera.R = mat4(rr.at<double>(0),rr.at<double>(3),rr.at<double>(6),0,
                               rr.at<double>(1),rr.at<double>(4),rr.at<double>(7),0,
                               rr.at<double>(2),rr.at<double>(5),rr.at<double>(8),0,
                               0,0,0,1);
    }
    
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer));
    
    ASSERT_GL(glClearColor( 0.0, 0.0, 0.0, 0.0));
    ASSERT_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    
    if(_controller->prossesor.atam.mState == STATE::TAM && _controller->prossesor.atam.mData.haveScale) engine.draw(frameBuffer);
    
    ASSERT_GL(glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer));
    [self.glContext presentRenderbuffer:GL_RENDERBUFFER];
}


- ( void )dealloc
{
    [EAGLContext setCurrentContext:self.glContext];
    
    if (frameBuffer)
    {
        ASSERT_GL(glDeleteFramebuffers(1, &frameBuffer));
        frameBuffer = 0;
    }
    if (renderBuffer)
    {
        ASSERT_GL(glDeleteRenderbuffers(1, &renderBuffer));
        renderBuffer = 0;
    }
    if (depthBuffer)
    {
        ASSERT_GL(glDeleteRenderbuffers(1, &depthBuffer));
        depthBuffer = 0;
    }
    
    if ([EAGLContext currentContext] == self.glContext) {
        [EAGLContext setCurrentContext:nil];
    }
}

@end
