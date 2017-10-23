#include <iostream>
#include <vector>
#include <ctime>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/ocl.hpp"

#include "hough_lane_detector.hpp"
#include "kalman_lane_tracker.hpp"

using namespace cv;
using namespace std;

int main()
{    
    // cap is the object of class video capture that tries to capture video file
    VideoCapture cap("/home/alex504/img_video_file/kalman/road_view.mp4");
    //VideoCapture cap("/home/alex/img_video_file/road_view.mp4");        

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
    
    HoughLaneDetector hdetector(180);

    while (1)
    {
        cv::UMat frame;
        //cv::UMat frameGray;
        //cv::UMat Thresh;
        cap >> frame;
        
        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }
                    
        // Image Filtering ex. sobel and threshold
        // cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY); 
        // Hough lane detector
        hdetector._standard_hough(frame); // draw lines

        // Kalman filter for tracking
        
        imshow( "Frame", frame );
        frameCounter++;

        std::time_t timeNow = std::time(0) - timeBegin;

        if (timeNow - tick >= 1)
        {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            frameCounter = 0;
        }        
        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
            break;
    }
    // When everything done, release the video capture object
    cap.release();
    // Closes all the frames
    destroyAllWindows();

    return 0;    
}
