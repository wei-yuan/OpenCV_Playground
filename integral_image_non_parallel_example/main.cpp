#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>\

int main()
{    
    cv::Mat src = cv::imread("720p.jpg", CV_32F);
    cv::Mat src_sum, src_sqsum;
    
    cv::UMat usrc, usrc_sum, usrc_sqsum;
    src.copyTo(usrc);

    // Time measurement
    int sdepth = -1;
    double mat_t1 = (double)cv::getTickCount();
        cv::integral(src, src_sum, src_sqsum, sdepth);
    double mat_t2 = (double)cv::getTickCount();

    double mat_time = (mat_t1 - mat_t2)/ (double)cv::getTickFrequency();

    std::cout << "mat_time: " << mat_time << "sec" << std::endl;

    cv::imshow("720p", src);
    //cv::waitKey();
    return 0;
}