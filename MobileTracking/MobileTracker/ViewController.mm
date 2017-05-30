//
//  ViewController.m
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#import "ViewController.h"


@interface ViewController ()

@property (strong, nonatomic) AVCaptureDeviceInput *videoInput;
@property (strong, nonatomic) AVCaptureVideoDataOutput *videoDataOutput;
@property (strong, nonatomic) AVCaptureSession *session;
@property (strong, nonatomic) AVCaptureMetadataOutput *output;

@property (strong, nonatomic) IBOutlet UIImageView *previewImageView;
@property (strong, nonatomic) IBOutlet UIImageView *searchImageView;
@property (strong, nonatomic) IBOutlet UILabel *trackingTimeLabel;
@property (strong, nonatomic) IBOutlet UILabel *currentStateLabel;
@property (strong, nonatomic) IBOutlet UILabel *informationLabel;

@end

@implementation ViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    prossesor.searchImageView = _searchImageView;
    prossesor.trackingTimeLabel = _trackingTimeLabel;
    prossesor.currentStateLabel = _currentStateLabel;
    prossesor.informationLabel = _informationLabel;
    
    // シングルタップ
    UILongPressGestureRecognizer *singleFingerDTap = [[UILongPressGestureRecognizer alloc]
                                                initWithTarget:self action:@selector(handleSingleTap:)];
    singleFingerDTap.minimumPressDuration = 0;
    
    [self.previewImageView addGestureRecognizer:singleFingerDTap];
    
    // 撮影開始
    [self setupAVCapture];
}

// セレクター
- (void)handleSingleTap:(UIGestureRecognizer *)sender {
    if (sender.state == UIGestureRecognizerStateBegan){
        CGPoint tapPoint = [sender locationInView:sender.view];
        
        float width = sender.view.frame.size.width;
        float height = sender.view.frame.size.height;
        float px = tapPoint.x;
        float py = tapPoint.y;
        
        vec2 position = vec2(px/width, py/height);
        
        prossesor.touchStart(position);
        
        AudioServicesPlaySystemSound(1110);
    }
    if (sender.state == UIGestureRecognizerStateChanged){
        CGPoint tapPoint = [sender locationInView:sender.view];
        
        float width = sender.view.frame.size.width;
        float height = sender.view.frame.size.height;
        float px = tapPoint.x;
        float py = tapPoint.y;
        
        vec2 position = vec2(px/width, py/height);
        
        prossesor.touchMove(position);
    }
    if (sender.state == UIGestureRecognizerStateEnded){
        CGPoint tapPoint = [sender locationInView:sender.view];
        
        float width = sender.view.frame.size.width;
        float height = sender.view.frame.size.height;
        float px = tapPoint.x;
        float py = tapPoint.y;
        
        vec2 position = vec2(px/width, py/height);
        
        prossesor.touchEnd(position);
        
        AudioServicesPlaySystemSound(1111);
    }
}

- (void)setupAVCapture
{
    NSError *error = nil;
    
    // 入力と出力からキャプチャーセッションを作成
    self.session = [[AVCaptureSession alloc] init];
    
    self.session.sessionPreset = AVCaptureSessionPresetMedium;
    
    // カメラからの入力を作成
    AVCaptureDevice *camera = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    // カメラからの入力を作成し、セッションに追加
    self.videoInput = [AVCaptureDeviceInput deviceInputWithDevice:camera error:&error];
    [self.session addInput:self.videoInput];
    
    // 画像への出力を作成し、セッションに追加
    self.videoDataOutput = [[AVCaptureVideoDataOutput alloc] init];
    [self.session addOutput:self.videoDataOutput];
    
    // ビデオ出力のキャプチャの画像情報のキューを設定
    dispatch_queue_t queue = dispatch_queue_create("myQueue", NULL);
    [self.videoDataOutput setAlwaysDiscardsLateVideoFrames:TRUE];
    [self.videoDataOutput setSampleBufferDelegate:self queue:queue];
    
    // ビデオへの出力の画像は、BGRAで出力
    self.videoDataOutput.videoSettings = @{
                                           (id)kCVPixelBufferPixelFormatTypeKey : [NSNumber numberWithInt:kCVPixelFormatType_32BGRA]
                                           };
    
    // ビデオ入力のAVCaptureConnectionを取得
    AVCaptureConnection *videoConnection = [self.videoDataOutput connectionWithMediaType:AVMediaTypeVideo];
    
    _output = [[AVCaptureMetadataOutput alloc] init];
    [_session addOutput:_output];
    
    _output.metadataObjectTypes = [_output availableMetadataObjectTypes];
    [_output setMetadataObjectsDelegate:self queue:dispatch_queue_create(nil, nil)];
    
    [self.session startRunning];
}

// AVCaptureVideoDataOutputSampleBufferDelegateプロトコルのメソッド。新しいキャプチャの情報が追加されたときに呼び出される。
- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection
{
    // キャプチャしたフレームからCGImageを作成
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    
    UIImage* correctImage = image;
    UIGraphicsBeginImageContext(correctImage.size);
    [correctImage drawInRect:CGRectMake(0, 0, correctImage.size.width, correctImage.size.height)];
    correctImage = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
    
    image = prossesor.processing(correctImage);
    
    int x = highlightViewRect.origin.x;
    int y = highlightViewRect.origin.y;
    int w = highlightViewRect.size.width;
    int h = highlightViewRect.size.height;
    
    //cout << x << ":" << y << ":" << w << ":" << h << endl;
    
    image = prossesor.drawRect(image,x,y,w,h);
    
    // 画像を画面に表示
    dispatch_async(dispatch_get_main_queue(), ^{
        self.previewImageView.image = image;
    });
    
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputMetadataObjects:(NSArray *)metadataObjects fromConnection:(AVCaptureConnection *)connection
{
    highlightViewRect = CGRectZero;
    AVMetadataMachineReadableCodeObject *barCodeObject;
    NSString *detectionString = nil;
    NSArray *barCodeTypes = @[AVMetadataObjectTypeQRCode];
    
    for (AVMetadataObject *metadata in metadataObjects) {
        for (NSString *type in barCodeTypes) {
            if ([metadata.type isEqualToString:type]){
                 /*barCodeObject = (AVMetadataMachineReadableCodeObject *)[_previewImageView transformedMetadataObjectForMetadataObject:(AVMetadataMachineReadableCodeObject *)metadata];*/
                highlightViewRect = barCodeObject.bounds;
                detectionString = [(AVMetadataMachineReadableCodeObject *)metadata stringValue];
                
                int x = highlightViewRect.origin.x;
                int y = highlightViewRect.origin.y;
                int w = highlightViewRect.size.width;
                int h = highlightViewRect.size.height;
                
                cout << x << ":" << y << ":" << w << ":" << h << endl;
                
                break;
            }
        }
        
    }
}

// サンプルバッファのデータからCGImageRefを生成する
- (UIImage *)imageFromSampleBuffer:(CMSampleBufferRef)sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    
    // ピクセルバッファのベースアドレスをロックする
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    
    // Get information of the image
    uint8_t *baseAddress = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
    
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    
    // RGBの色空間
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGContextRef newContext = CGBitmapContextCreate(baseAddress,
                                                    width,
                                                    height,
                                                    8,
                                                    bytesPerRow,
                                                    colorSpace,
                                                    kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    
    CGImageRef cgImage = CGBitmapContextCreateImage(newContext);
    
    CGContextRelease(newContext);
    CGColorSpaceRelease(colorSpace);
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
    
    UIImage *image = [UIImage imageWithCGImage:cgImage scale:1.0 orientation:UIImageOrientationRight];
    
    CGImageRelease(cgImage);
    
    return image;
}


@end
