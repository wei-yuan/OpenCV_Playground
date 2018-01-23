//--------------------------------------------------------------
/*
Author: Wei-Yuan Alex Hsu
Date:   2018/1/19 Fri
Target: implement non-parallel and parallel( OpenCL based ) histogram equalization
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
    cv::Mat grey, grey_resize;
    cv::cvtColor(src, grey, cv::COLOR_BGR2GRAY);    
    cv::Size size(1000, 1000);
    cv::resize(grey, grey_resize, size);//resize image
    //std::cout <<"grey_resize: "<< grey_resize << std::endl;
    
    cv::imshow("resize", grey_resize);
    //cv::waitKey(0);

    // for debugging image resizing
   
/* 
    // mini test
    cv::Mat input_grey = (cv::Mat_<float>(5,5) << 4, 4, 4, 4, 4, 
                                                3, 4, 5, 4, 3,
                                                3, 5, 5, 5, 3,
                                                3, 4, 5, 4, 3,
                                                4, 4, 4, 4, 4);
 
    cv::Mat grey_resize;
    input_grey.convertTo(grey_resize, CV_8U);
    std::cout <<"grey_resize type: "<< grey_resize.type() << std::endl;    
*/   
    equalOfHist(grey_resize);
    cv::waitKey(0); // wait until any key press

/*
    // Time measurement
    int sdepth = -1;
    double umat_t1 = cv::getTickCount();
        //cv::integral(usrc, usrc_sum, usrc_sqsum, sdepth);
    double umat_t2 = cv::getTickCount();

    double umat_time = (umat_t1 - umat_t2)/ cv::getTickFrequency();    

    //cv::waitKey();
*/        
    return 0;
}