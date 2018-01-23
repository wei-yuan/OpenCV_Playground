//--------------------------------------------------------------
/*
Author: Wei-Yuan Alex Hsu
Date:   2018/1/19 Fri
Target: implement non-parallel and parallel( OpenCL based ) histogram equalization
Reference: 
[1] Histogram Calculation - https://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html#code
[2] How to scan images, lookup tables and time measurement with OpenCV - https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv

*/
//--------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <string>
#include "main.hpp"
// opencv library
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

void equalOfHist( cv::Mat& src);
int main()
{
        
    std::string INPUT_FILE_PATH ("/home/alex504/img_and_video_data_set/image");
    std::string INPUT_FILENAME ("720p.jpg");
    std::string INPUT_FILE = INPUT_FILE_PATH + '/' + INPUT_FILENAME;

    // mat type is CV_8U
    cv::Mat src = cv::imread(INPUT_FILE);       
    cv::Mat grey;
    cv::UMat umat_grey;
    cv::cvtColor(src, grey, cv::COLOR_BGR2GRAY);    
    cv::cvtColor(src, umat_grey, cv::COLOR_BGR2GRAY);
 
    // mini test
    // cv::Mat input_grey = (cv::Mat_<float>(5,5) << 4, 4, 4, 4, 4,
    //                                             3, 5, 5, 5, 3,
    //                                             3, 4, 5, 4, 3,
    //                                             4, 4, 4, 4, 4);
    // cv::Mat grey_resize;
    // input_grey.convertTo(grey_resize, CV_8U);
    // std::cout <<"grey_resize type: "<< grey_resize.type() << std::endl;    
    
    //---------------------------------------// 
    // histogram equalization
    //---------------------------------------//           
    double equalOfHist_t1 = cv::getTickCount();    
    equalOfHist(grey);
    double equalOfHist_t2 = cv::getTickCount();

    double equalOfHist_time = (equalOfHist_t2 - equalOfHist_t1)/ cv::getTickFrequency();    
    std::cout <<"equalOfHist_time: "<< equalOfHist_time << std::endl;
    // waitKey(0) will display the window infinitely 
    // until any keypress (it is suitable for image display)
    //cv::waitKey(0);

    // Time measurement    
    cv::Mat eqOutput;
    double opencv_equalizeHist_t1 = cv::getTickCount();
    cv::equalizeHist(grey, eqOutput);
    double opencv_equalizeHist_t2 = cv::getTickCount();        

    double opencv_equalizeHist_time = (opencv_equalizeHist_t2 - opencv_equalizeHist_t1)/ cv::getTickFrequency();    
    std::cout <<"opencv_equalizeHist_time: "<< opencv_equalizeHist_time << std::endl;

    cv::UMat umat_eqOutput;
    double opencv_umat_equalizeHist_t1 = cv::getTickCount();
    cv::equalizeHist(umat_grey, umat_eqOutput);
    double opencv_umat_equalizeHist_t2 = cv::getTickCount();

    double opencv_umat_equalizeHist_time = (opencv_umat_equalizeHist_t2 - opencv_umat_equalizeHist_t1)/ cv::getTickFrequency();    
    std::cout <<"opencv_umat_equalizeHist_time: "<< opencv_umat_equalizeHist_time << std::endl;    
    
    return 0;
}