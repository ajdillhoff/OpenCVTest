//
//  MainViewController.h
//  OpenCVTest
//
//  Created by Ayoka Systems on 6/30/14.
//  Copyright (c) 2014 Ayoka Systems. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

@interface MainViewController : UIViewController {
  
  BOOL FrontCamera;
  BOOL haveImage;
}

@property (strong, nonatomic) IBOutlet UIImageView *imageView;
@property (strong, nonatomic) IBOutlet UIView *imagePreview;
@property (strong, nonatomic) AVCaptureVideoPreviewLayer *previewLayer;
@property (strong, nonatomic) AVCaptureSession *captureSession;
@property(nonatomic, retain) AVCaptureStillImageOutput *stillImageOutput;
@property (strong, nonatomic) IBOutlet UILabel *outputLabel;

- (void)processImage:(UIImage *)image;
- (IBAction)snapImage:(id)sender;
- (IBAction)clearImage:(id)sender;

@end
