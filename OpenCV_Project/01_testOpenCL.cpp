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
    VideoCapture cap("../test2_720p.mp4");
	
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
		cout << "Cannot open the video file. \n";
		return -1;
    }

    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;

/*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    ocl::setUseOpenCL(true);
/*                                      */
    int nBlurs = 50;

    long frameCounter = 0;

    std::time_t timeBegin = std::time(0);
    int tick = 0;

    cv::UMat frame;
    cv::UMat frameGray;
    cv::UMat frameSobelx;
    cv::UMat frameSobely;

    cv::UMat frameSobel;
    cv::UMat blurredSobel;
    /*cv::Mat frame;
    cv::Mat frameGray;
    cv::Mat frameSobelx;
    cv::Mat frameSobely;

    cv::Mat frameSobel;
    cv::Mat blurredSobel;*/
    //cv::UMat Thresh;

    while (1)
    {
        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }

        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

        cv::Sobel(frameGray, frameSobelx, frameGray.depth(), 1, 0, 3);
        cv::Sobel(frameGray, frameSobely, frameGray.depth(), 0, 1, 3);

        cv::bitwise_or(frameSobelx, frameSobely, frameSobel);

        for (int n = 0; n < nBlurs; n++)
            cv::blur(frameSobel, blurredSobel, cv::Size(3,3));

        //cv::imshow("Original Frame", frame);            
        cv::imshow("Sobel blurred Frame", blurredSobel);
        cv::waitKey(1);

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