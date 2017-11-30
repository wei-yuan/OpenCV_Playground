#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>\

int main()
{    
    cv::Mat src = cv::imread("/home/alex/img_video_file/720p.jpg", CV_32F);
    cv::Mat src_sum, src_sqsum;
    
    cv::UMat usrc, usrc_sum, usrc_sqsum;
    src.copyTo(usrc);

    // Time measurement
    int sdepth = -1;
    time_t mat_t1 = cv::getTickCount();
        cv::integral(src, src_sum, src_sqsum, sdepth);
    time_t mat_t2 = cv::getTickCount();

    time_t mat_time = (mat_t1 - mat_t2)/ cv::getTickFrequency();

    time_t umat_t1 = cv::getTickCount();
        cv::integral(usrc, usrc_sum, usrc_sqsum, sdepth);
    time_t umat_t2 = cv::getTickCount();

    time_t umat_time = (umat_t1 - umat_t2)/ cv::getTickFrequency();

    std::cout << "mat_time: " << mat_time << "sec" << "\numat_time" << umat_time << "sec" << std::endl;

    cv::imshow("720p", src);
    cv::waitKey();
    return 0;
}