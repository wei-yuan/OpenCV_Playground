#include <iostream>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/ocl.hpp>


using namespace cv;
using namespace std;

int main()
{
    // Global variables
    int threshold_value = 125;
    int threshold_type = 3;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;

    // cap is the object of class video capture that tries to capture Bumpy.mp4
    VideoCapture cap("../../test1_720p.mp4");
	
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
		cout << "Cannot open the video file. \n";
		return -1;
    }

    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;

/*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    cv::ocl::setUseOpenCL(true); //-> doesn't work?

    long frameCounter = 0;

    std::time_t timeBegin = std::time(0);
    int tick = 0;

    cv::UMat frame;
    cv::UMat frame_gray;
    cv::UMat d_frame;    

    while (1)
    {

        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }

        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        threshold( frame_gray, d_frame, threshold_value, max_BINARY_value, threshold_type );

        cv::imshow("Original", frame);
        cv::imshow("Threshold", d_frame);

        if (cv::waitKey(3) > 0)
            break;
    
        frameCounter++;

        std::time_t timeNow = std::time(0) - timeBegin;

        if (timeNow - tick >= 1)
        {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            frameCounter = 0;
        }
    }

    cap.release();

    return 0;
}