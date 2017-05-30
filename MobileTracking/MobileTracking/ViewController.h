//
//  ViewController.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ImageProcessor.hpp"
#include "GLView.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <CoreImage/CoreImage.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreGraphics/CoreGraphics.h>
#include <OpenGLES/ES3/gl.h>
#include <OpenGLES/ES3/glext.h>
#include <OpenGLES/EAGL.h>


@interface ViewController : UIViewController <AVCaptureVideoDataOutputSampleBufferDelegate>
{
    @public
    ImageProcessor prossesor;
}
@end


