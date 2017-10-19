#include <iostream>
#include <ctime>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/ocl.hpp"

using namespace cv;
using namespace std;

int main()
{
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    VideoCapture cap("/home/alex504/img_video_file/kalman/road_view.mp4");
	
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
		cout << "Cannot open the video file. \n";
		return -1;
    }

    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;

/*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    ocl::setUseOpenCL(true);
/*                                      */
    long frameCounter = 0;
    std::time_t timeBegin = std::time(0);
    int tick = 0;

    cv::UMat frame;
    cv::UMat frameGray;
    cv::UMat Thresh;

    while (1)
    {
        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }

        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

        // Kalman filter for tracking
        

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