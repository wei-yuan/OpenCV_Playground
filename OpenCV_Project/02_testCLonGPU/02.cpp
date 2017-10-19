#include <iostream>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include <CL/cl.h>

using namespace cv;
using namespace std;

int main()
{
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    VideoCapture cap("../../../img_video_file/test1_720p.mp4");
	
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
		cout << "Cannot open the video file. \n";
		return -1;
    }

    cout << "Have OpenCL?: " << cv::ocl::haveOpenCL() << endl;

/*  SWITCH OPENCL ON/OFF IN LINE BELLOW */
    cv::ocl::setUseOpenCL(false); //-> doesn't work?

 /*   cv::Mat image = cv::imread("lena.bmp", CV_LOAD_IMAGE_COLOR);
    cv::Mat imageRGBA;

    width = image.rows;
    height = image.cols;

    cv::cvtColor(image, imageRGBA, CV_BGR2RGBA);
    char *buffer = reinterpret_cast<char *>(imageRGBA.data);*/

/*    cl::Image2D clImage(context,
                            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                            cl::ImageFormat(CL_RGBA, CL_UNORM_INT8),
                            width,
                            height,
                            0,
                            buffer);

    return clImage;*/

    long frameCounter = 0;

    std::time_t timeBegin = std::time(0);
    int tick = 0;

    //cv::UMat frame;
    cv::Mat frame;
    //cv::UMat d_frame;    

    while (1)
    {
        //tm.reset(); tm.start();

        if (!cap.read(frame)){ // if not success, break loop
            cout<<"\n Cannot read the video file. \n";
            break;
        }
        //cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

        cv::imshow("CPU", frame);
        //cv::imshow("GPU", d_frame);

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
