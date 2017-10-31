#include <iostream>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;
using namespace std;

int main()
{
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    cv::VideoCapture cap("/home/alex504/img_video_file/kalman/road_view.mp4");
    // cv::VideoCapture cap("/home/alex/img_video_file/road_view.mp4");

    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
		cout << "Cannot open the video file. \n";
		return -1;
    }

    // count time 
    std::time_t timeBegin = std::time(0);
    long frameCounter = 0;
    int tick = 0;

    // OpenCL related
    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;
    /*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    ocl::setUseOpenCL(true);    

    // sobel filter
    int nBlurs = 50;    
        
    cv::UMat frame, frameGray, frameSobelx, frameSobely, frameSobel, blurredSobel;
    
    // thresh
    int thresh_val = 125, max_BINARY_value = 256, threshold_type = THRESH_TOZERO_INV;
    
    cv::UMat thresh;

    while (1)
    {
        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }
        
        // RGB to GRAY
        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
        // Sobel
        cv::Sobel(frameGray, frameSobelx, frameGray.depth(), 1, 0, 3);
        cv::Sobel(frameGray, frameSobely, frameGray.depth(), 0, 1, 3);
        cv::bitwise_or(frameSobelx, frameSobely, frameSobel);
        for (int n = 0; n < nBlurs; n++)
            cv::blur(frameSobel, blurredSobel, cv::Size(3,3));
        // Threshold
        threshold( blurredSobel, thresh, thresh_val, max_BINARY_value, threshold_type );

        cv::imshow("Original Frame", frame);            
        cv::imshow("Sobel blurred Frame", blurredSobel);
        cv::imshow("Threshold", thresh);
        
        cv::waitKey(1);
        
        // calculate time 
        frameCounter++;
        std::time_t timeNow = std::time(0) - timeBegin;
        if (timeNow - tick >= 1)
        {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            frameCounter = 0;
        }
    }

    return 0;
}