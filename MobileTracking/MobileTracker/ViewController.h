//
//  ViewController.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ImageProcessor.hpp"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <CoreImage/CoreImage.h>
#import <CoreGraphics/CoreGraphics.h>
#import <QuartzCore/QuartzCore.h>
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>
#include <glm/glm.hpp>

@interface ViewController : UIViewController <AVCaptureVideoDataOutputSampleBufferDelegate , AVCaptureMetadataOutputObjectsDelegate>
{
    ImageProcessor prossesor;
    CGRect highlightViewRect;
}
@end



