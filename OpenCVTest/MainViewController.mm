//
//  MainViewController.m
//  OpenCVTest
//
//  Created by Ayoka Systems on 6/30/14.
//  Copyright (c) 2014 Ayoka Systems. All rights reserved.
//

#import "MainViewController.h"

@interface MainViewController ()

@end

@implementation MainViewController

// Helpers
cv::Point2f center(0,0);

cv::Point2f computeIntersect(cv::Vec4i a,
                             cv::Vec4i b)
{
  int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
  
  if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
  {
    cv::Point2f pt;
    pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
    pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
    return pt;
  }
  else
    return cv::Point2f(-1, -1);
}

void sortCorners(std::vector<cv::Point2f>& corners,
                 cv::Point2f center)
{
  std::vector<cv::Point2f> top, bot;
  
  for (int i = 0; i < corners.size(); i++)
  {
    if (corners[i].y < center.y)
      top.push_back(corners[i]);
    else
      bot.push_back(corners[i]);
  }
  
  cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
  cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
  cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
  cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
  
  corners.clear();
  corners.push_back(tl);
  corners.push_back(tr);
  corners.push_back(br);
  corners.push_back(bl);
}

/*
 * Name: findExtendedPeaks
 * PRECONDITION:  Valid hough peaks given from the hough transform are used
 *                to detect extended peaks
 * POSTCONDITION: Extended peaks are found based on the given hough peaks
 * Description: Modified implementation of "Rectangle Detection based on a Windowed Hough Transform"
 *              http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.59.4239&rep=rep1&type=pdf
 */
std::vector<cv::Vec4f> findExtendedPeaks(std::vector<cv::Vec2f> lines) {
  // Local thresholds
  double t_theta = 10 * (CV_PI/180);
  double t_rho   = 50;
  std::vector<cv::Vec4f> P; // stores extended peaks
  
  for (int i = 0; i < (int)lines.size() - 1; i++) {
    for (int j = i + 1; j < (int)lines.size() - 1; j++) {
      // Center points
      double rho_i = lines[i][0];
      double rho_j = lines[j][0];
      double theta_i = lines[i][1];
      double theta_j = lines[j][1];
//      double mid = 0;
//      if (rho_i != rho_j) {
//        mid = (rho_i + rho_j) / 2;
//        rho_i = rho_i - mid;
//        rho_j = rho_j - mid;
//      }
      
      // Extended peaks must pass three conditions, vote threshold already passed
      if (abs(lines[i][1] - lines[j][1]) < t_theta && abs(rho_i + rho_j) < t_rho) {
        P.push_back(cv::Vec4f(abs(rho_i - rho_j) / 2, (lines[i][1] + lines[j][1]) / 2, i, j));
      }
    }
  }
  
  return P;
}

/*
 * Name: detectRectangle
 * PRECONDITION:  Valid extended peaks are given
 * POSTCONDITION: Rectangles are detected based on the relationship between extended peaks
 * Description: Modified implementation of "Rectangle Detection based on a Windowed Hough Transform"
 *              http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.59.4239&rep=rep1&type=pdf
 */
int detectRectangle(std::vector<cv::Vec4f> P, std::vector<cv::Vec2f> H, cv::Mat& image) {
  double c = 180/CV_PI;
  double t_alpha = 30 * (CV_PI/180);
  double t_xi    = 100;
  double e_min   = 9999;
  int min_k = 0, min_l = 0;
  int a = 1;
  int b = 4;
  int result = 0;
  
  NSLog(@"%lu", P.size());
  
  for (int k = 0; k < (int)P.size() - 1; k++) {
    for (int l = k + 1; l < (int)P.size() - 1; l++) {
      //      NSLog(@"alpha_i: %f alpha_j: %f", P[i][1], P[j][1]);
      double alpha = abs(abs(P[k][1] - P[l][1]) - CV_PI/2);
      if (alpha < t_alpha &&
          P[k][0] > t_xi && P[l][0] > t_xi &&
          abs(P[k][0] - P[l][0]) > 50) {
        
        // Calculate the error of the current rectangle
        int k_i = P[k][2];
        int k_j = P[k][3];
        int l_i = P[l][2];
        int l_j = P[l][3];
        double d_rho_k = abs(H[k_i][0] + H[k_j][0]);
        double d_rho_l = abs(H[l_i][0] + H[l_j][0]);
        double d_theta_k = abs(H[k_i][1] - H[k_j][1]);
        double d_theta_l = abs(H[l_i][1] - H[l_j][1]);
        double error = sqrt(a*(pow(c*d_theta_k, 2) + pow(c*d_theta_l, 2) + pow(c*alpha, 2)) + b*(pow(d_rho_k, 2) + pow(d_rho_l, 2)));
        if (error < e_min) {
          e_min = error;
          min_k = k;
          min_l = l;
        }
      }
    }
  }
  
  if (e_min < 9999) {
    NSLog(@"error: %f", e_min);
    NSLog(@"P_k: (%f, %f)", P[min_k][0], P[min_k][1]);
    NSLog(@"P_l: (%f, %f)", P[min_l][0], P[min_l][1]);
    
    // Draw a rectangle on the image
    int mid_x = image.size().width / 2;
    int mid_y = image.size().height / 2;
    int p1_x = mid_x - P[min_k][0];
    int p1_y = mid_y - P[min_l][0];
    int p2_x = mid_x + P[min_k][0];
    int p2_y = mid_y + P[min_l][0];
    
    cv::Point p1 = cv::Point(p1_x, p1_y);
    cv::Point p2 = cv::Point(p2_x, p2_y);
    
    cv::rectangle(image, p1, p2, cv::Scalar( 0, 255, 0 ), 10);
    
    result = 1;
  }
  
  return result;
}

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
  self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
  if (self) {
    // Custom initialization
  }
  return self;
}

- (void)viewDidLoad
{
  [super viewDidLoad];
  // Do any additional setup after loading the view.
  FrontCamera = NO;
  _imageView.hidden = YES;
  
}

- (void)viewDidAppear:(BOOL)animated {
  [self initializeCamera];
//  UIImage *testImage = [UIImage imageNamed:@"check.jpg"];
//  [self processImage:testImage];
}

- (void)didReceiveMemoryWarning
{
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

- (void)didRotateFromInterfaceOrientation:(UIInterfaceOrientation)fromInterfaceOrientation {
}

/*****************************
 * OpenCV Conversion Methods *
 *****************************/

// Convert from UIImage
- (cv::Mat)cvMatFromUIImage:(UIImage *)image
{
  CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
  CGFloat cols = image.size.width;
  CGFloat rows = image.size.height;
  
  cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
  
  CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                  cols,                       // Width of bitmap
                                                  rows,                       // Height of bitmap
                                                  8,                          // Bits per component
                                                  cvMat.step[0],              // Bytes per row
                                                  colorSpace,                 // Colorspace
                                                  kCGImageAlphaNoneSkipLast |
                                                  kCGBitmapByteOrderDefault); // Bitmap info flags
  
  CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
  CGContextRelease(contextRef);
  
  return cvMat;
}

// Convert UIImage to grayscale cvMat format
- (cv::Mat)cvMatGrayFromUIImage:(UIImage *)image
{
  CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceGray();
  CGFloat cols = image.size.width;
  CGFloat rows = image.size.height;
  
  cv::Mat cvMat(rows, cols, CV_8UC1); // 8 bits per component, 1 channels
  
  CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to data
                                                  cols,                       // Width of bitmap
                                                  rows,                       // Height of bitmap
                                                  8,                          // Bits per component
                                                  cvMat.step[0],              // Bytes per row
                                                  colorSpace,                 // Colorspace
                                                  kCGImageAlphaNone |
                                                  kCGBitmapByteOrderDefault); // Bitmap info flags
  
  CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
  CGContextRelease(contextRef);
  CGColorSpaceRelease(colorSpace);
  
  return cvMat;
}

// Converts cvMat object back to UIImage
- (UIImage *)UIImageFromCVMat:(cv::Mat)cvMat
{
  NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize()*cvMat.total()];
  CGColorSpaceRef colorSpace;
  
  if (cvMat.elemSize() == 1) {
    colorSpace = CGColorSpaceCreateDeviceGray();
  } else {
    colorSpace = CGColorSpaceCreateDeviceRGB();
  }
  
  CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
  
  // Creating CGImage from cv::Mat
  CGImageRef imageRef = CGImageCreate(cvMat.cols,                                 //width
                                      cvMat.rows,                                 //height
                                      8,                                          //bits per component
                                      8 * cvMat.elemSize(),                       //bits per pixel
                                      cvMat.step[0],                            //bytesPerRow
                                      colorSpace,                                 //colorspace
                                      kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                      provider,                                   //CGDataProviderRef
                                      NULL,                                       //decode
                                      false,                                      //should interpolate
                                      kCGRenderingIntentDefault                   //intent
                                      );
  
  
  // Getting UIImage from CGImage
  UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
  CGImageRelease(imageRef);
  CGDataProviderRelease(provider);
  CGColorSpaceRelease(colorSpace);
  
  return finalImage;
}

- (AVCaptureVideoPreviewLayer *) previewLayer
{
  if(!_previewLayer)
  {
    _previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession: self.captureSession];
    
    [_previewLayer setVideoGravity:AVLayerVideoGravityResizeAspectFill];
    
    _previewLayer.frame = self.imageView.bounds; // Assume you want the preview layer to fill the view.
    
    [_previewLayer setPosition:CGPointMake(0,0)];
    
    if (UIDeviceOrientationLandscapeLeft == [[UIDevice currentDevice] orientation]) {
      _previewLayer.transform = CATransform3DMakeRotation(-M_PI/2, 0, 0, 1);
    }
    else if (UIDeviceOrientationLandscapeRight == [[UIDevice currentDevice] orientation])
    {
      _previewLayer.transform = CATransform3DMakeRotation(M_PI/2, 0, 0, 1);
    }
  }
  
  return _previewLayer;
}

- (void) initializeCamera {
  _captureSession = [[AVCaptureSession alloc] init];
	_captureSession.sessionPreset = AVCaptureSessionPresetPhoto;
  
	AVCaptureVideoPreviewLayer *captureVideoPreviewLayer = [self previewLayer];
	[self.imagePreview.layer addSublayer:captureVideoPreviewLayer];
  
  UIView *view = [self imagePreview];
  CALayer *viewLayer = [view layer];
  [viewLayer setMasksToBounds:YES];
  
  CGRect bounds = [view bounds];
  [captureVideoPreviewLayer setFrame:bounds];
  
  NSLog(@"bounds: %@", NSStringFromCGRect(bounds));
  
  NSArray *devices = [AVCaptureDevice devices];
  AVCaptureDevice *frontCamera;
  AVCaptureDevice *backCamera;
  
  for (AVCaptureDevice *device in devices) {
    
    NSLog(@"Device name: %@", [device localizedName]);
    
    if ([device hasMediaType:AVMediaTypeVideo]) {
      
      if ([device position] == AVCaptureDevicePositionBack) {
        NSLog(@"Device position : back");
        backCamera = device;
      }
      else {
        NSLog(@"Device position : front");
        frontCamera = device;
      }
    }
  }
  
  if (!FrontCamera) {
    NSError *error = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:backCamera error:&error];
    if (!input) {
      NSLog(@"ERROR: trying to open camera: %@", error);
    }
    [_captureSession addInput:input];
  }
  
  if (FrontCamera) {
    NSError *error = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:frontCamera error:&error];
    if (!input) {
      NSLog(@"ERROR: trying to open camera: %@", error);
    }
    [_captureSession addInput:input];
  }
  
  _stillImageOutput = [[AVCaptureStillImageOutput alloc] init];
  NSDictionary *outputSettings = [[NSDictionary alloc] initWithObjectsAndKeys: AVVideoCodecJPEG, AVVideoCodecKey, nil];
  [_stillImageOutput setOutputSettings:outputSettings];
  
  [_captureSession addOutput:_stillImageOutput];
  
	[_captureSession startRunning];
}

- (void)processImage:(UIImage *)testImage {
  NSLog(@"Starting sequence");
  //  UIImage *testImage = [UIImage imageNamed:@"check18.jpg"];
  double dilation_size = 3.0;
  cv::Mat src = [self cvMatFromUIImage:testImage];
  cv::Mat test;
  cv::Mat grayMat;
  
  double width = _imageView.frame.size.width;
  double width_ratio = src.size().width / width;
  double new_height = src.size().height / width_ratio;
  
  // Resize the image
  cv::resize(src, test, cv::Size(width, new_height));
  
  // Convert to grayscale
  cv::cvtColor(test, grayMat, CV_BGR2GRAY);
  
  // blur the image for Canny
  cv::GaussianBlur(grayMat, grayMat, cv::Size(3, 3), 10.0);
  
  cv::Scalar mean, stdDev;
  cv::meanStdDev(grayMat, mean, stdDev);
  
  NSLog(@"mean: %f, std: %f", mean[0], stdDev[0]);
  
  // Apply a threshold
  cv::Mat element = getStructuringElement( 0,
                                          cv::Size( 2*dilation_size + 1, 2*dilation_size + 1 ),
                                          cv::Point( dilation_size, dilation_size ) );
  
  // calculate threshold from linear regression results
  double threshold = 188.360499 + (stdDev[0] * -1.001518);
  cv::threshold(grayMat, grayMat, threshold, 255.0, 0);
  
  // Dilate to remove unconnected dark spots
  cv::dilate(grayMat, grayMat, element);
  
  float t1 = 20.0;
  cv::Canny(grayMat, grayMat,
            t1,
            2*t1);
  
  // Hough Transform to detect the lines
  std::vector<cv::Vec2f> lines;
  
  // We cannot get the exact accumulator count, but having this threshold will
  // satisfy the third condition of our test
  int hough_threshold = 100;
  cv::HoughLines(grayMat, lines, 1, CV_PI/180, hough_threshold);
  
  // Convert these lines
  for (int i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    double x0 = cvRound(cos(theta) * rho), y0 = cvRound(sin(theta) * rho);
    double t_w = width / 2, t_h = new_height / 2;
    double delta_x = x0 - t_w, delta_y = y0 - t_h;
    float rho_m = delta_x * cos(theta) + delta_y * sin(theta);
    
    lines[i][0] = rho_m;
  }
  
  int result = detectRectangle(findExtendedPeaks(lines), lines, test);
  
  if (result) {
    [self.outputLabel setText:@"Possible Cheque Found"];
  } else {
    [self.outputLabel setText:@"No Cheque Detected"];
  }
  
  cv::Mat dst = test.clone();

  [_imageView setImage:[self UIImageFromCVMat:test]];
  NSLog(@"Ending sequence");
  
}

- (void)capImage { //method to capture image from AVCaptureSession video feed
  AVCaptureConnection *videoConnection = nil;
  for (AVCaptureConnection *connection in _stillImageOutput.connections) {
    for (AVCaptureInputPort *port in [connection inputPorts]) {
      
      if ([[port mediaType] isEqual:AVMediaTypeVideo] ) {
        videoConnection = connection;
        //        videoConnection.videoOrientation = AVCaptureVideoOrientationPortrait;
        break;
      }
    }
    
    if (videoConnection) {
      break;
    }
  }
  
  [_stillImageOutput captureStillImageAsynchronouslyFromConnection:videoConnection completionHandler: ^(CMSampleBufferRef imageSampleBuffer, NSError *error) {
    
    if (imageSampleBuffer != NULL) {
      NSData *imageData = [AVCaptureStillImageOutput jpegStillImageNSDataRepresentation:imageSampleBuffer];
      [self processImage:[UIImage imageWithData:imageData]];
    }
  }];
}

- (IBAction)snapImage:(id)sender {
  if (!haveImage) {
    _imageView.image = nil; //remove old image from view
    _imageView.hidden = NO; //show the captured image view
    _imagePreview.hidden = YES; //hide the live video feed
    [self capImage];
  }
  else {
    _imageView.hidden = YES;
    _imagePreview.hidden = NO;
    haveImage = NO;
  }
}

- (IBAction)clearImage:(id)sender {
  _imageView.image = nil; //remove old image from view
  _imageView.hidden = YES; //hide the captured image view
  _imagePreview.hidden = NO; //show the live video feed
}
@end
