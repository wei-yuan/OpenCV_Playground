#include <iostream>
#include <vector>
#include <ctime>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/ocl.hpp"

#include "hough_lane_detector.hpp"
#include "kalman_lane_tracker.hpp"

using namespace std;

int main()
{
    // cap is the object of class video capture that tries to capture video file
    cv::VideoCapture cap("/home/alex504/img_video_file/kalman/road_view.mp4");
    // cv::VideoCapture cap("/home/alex/img_video_file/road_view.mp4");

    if (!cap.isOpened()) // isOpened() returns true if capturing has been initialized.
    {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;
    /*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    cv::ocl::setUseOpenCL(true);

    long        frameCounter = 0;
    std::time_t timeBegin    = std::time(0);
    int         tick         = 0;
    float ktick = 0;

    // Hough transform related
    float             init_vote = 150; // what's this for?    
    HoughLaneDetector hdetector(180); // road horizon: 180 degree 
    std::pair<my::Line, my::Line> lanes;    

    // Kalman filter for tracking
    std::vector<my::Line> predicted;
    KalmanLaneTracker KTracker(2, 0.1, 500);    

    while (true) {
        cv::UMat src, croppedImg, dst, cdst;
        // input image and check
        cap >> src;
        if (!cap.read(src)) { // if not success, break loop
            cout << "\n Cannot read the video file. \n";
            break;
        }
        // kalman filter
        float precTick = ktick;
        ktick = cv::getTickCount();
        float dt = (ktick - precTick) / cv::getTickFrequency();
        // crop input image
        // cv::Rect roi(xMin,yMin,xMax-xMin,yMax-yMin);
        cv::Rect myROI(500, 300, 100 * 4, 100 * 3);
        cv::UMat croppedRef(src, myROI);
        croppedRef.copyTo(croppedImg);
        // edge detection
        cv::Canny(croppedImg, dst, 50, 200, 3);
        // detected lines        
//        predicted = KTracker.predict(dt);
        lanes = hdetector.detect(croppedImg);

        // prepare the color canvas for output image
        cv::cvtColor(dst, cdst, CV_GRAY2BGR);
        // draw line                        
//        cv::line(cdst, predicted[0].beg, predicted[0].end, cvScalar(0, 0, 255), 3, cv::LINE_AA);
//        cv::line(cdst, predicted[1].beg, predicted[1].end, cvScalar(0, 0, 255), 3, cv::LINE_AA);

        // update here
//        KTracker.update(lanes.first);

        // image check
        cv::imshow("source", src);
//        cv::imshow("Frame", cdst);     

        // Calculate frame per second
        frameCounter++;
        std::time_t timeNow = std::time(0) - timeBegin;
        if (timeNow - tick >= 1) {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            frameCounter = 0;
        }
        // Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(25);
        if (c == 27) break;
        if (cv::waitKey(10) == 32) break;        
    }
    // When everything done, release the video capture object
    cap.release();
    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
