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
#include <numeric>
#include <vector>  /* std container - vector*/
// opencv library
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

void equalOfHist( cv::Mat& src);
int main()
{
    /*    
    std::string INPUT_FILE_PATH ("/home/alex504/img_and_video_data_set/image");
    std::string INPUT_FILENAME ("720p.jpg");
    cv::Mat src = cv::imread(INPUT_FILE);       
    */
    std::string INPUT_FILE_PATH ("/home/alex504/img_and_video_data_set/video/1min");
    //std::string INPUT_FILENAME ("car.mp4");     
    std::string INPUT_FILENAME ("1440p_1min.mp4"); 
       
    std::string INPUT_FILE = INPUT_FILE_PATH + '/' + INPUT_FILENAME;
 
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
    // double equalOfHist_t1 = cv::getTickCount();    
    // equalOfHist(grey);
    // double equalOfHist_t2 = cv::getTickCount();

    // double equalOfHist_time = (equalOfHist_t2 - equalOfHist_t1)/ cv::getTickFrequency();    
    // std::cout <<"equalOfHist_time: "<< equalOfHist_time << std::endl;
    
    // waitKey(0) will display the window infinitely 
    // until any keypress (it is suitable for image display)
    //cv::waitKey(0);
    
    // mat type is CV_8U
    cv::VideoCapture video(INPUT_FILE);    
    // check video file
    if( !video.isOpened() )
    {
        std::cout << "Could not initialize capturing..." << std::endl;
        return -1;
    }    

    int frameNumbers = video.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "frameNumbers: " << frameNumbers << std::endl;
    
    std::vector<double> mat_time(frameNumbers);
    std::vector<double> umat_time(frameNumbers);

    int cnt = 0;
    cv::Mat frame, grey;    
    // non-opencl time measurement    
    double mat_eH_outside_t1 = cv::getTickCount();    
    for(;;)
    {
        video >> frame; // get a new frame from video    

        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);    
        cv::Mat eqOutput;
        //double mat_eh_inside_t1 = cv::getTickCount();
        cv::equalizeHist(grey, eqOutput);
        //double mat_eh_inside_t2 = cv::getTickCount();                

        // Time result
        //double mat_eH_time = (mat_eh_inside_t2 - mat_eh_inside_t1)/ cv::getTickFrequency();
        //mat_time.push_back(mat_eH_time);                  
        
        cnt++;
        if(cnt == frameNumbers) break;
    }
    double mat_eH_outside_t2 = cv::getTickCount();    
    // total time
    double mat_total_time = (mat_eH_outside_t2 - mat_eH_outside_t1) / cv::getTickFrequency();
    // fps
    int fps = frameNumbers / mat_total_time;     
    
    std::cout <<"mat_total_time: "<< mat_total_time << std::endl;
    std::cout <<"EqualizeHist() - fps of Mat: "<< fps << std::endl;

    // input video file again
    cv::VideoCapture cap(INPUT_FILE);    

    cnt = 0;
    cv::Mat uframe, umat_grey;
    // with opencl time measurement    
    double umat_eH_outside_t1 = cv::getTickCount();    
    for(;;)
    {
        cap >> uframe;    

        cv::cvtColor(uframe, umat_grey, cv::COLOR_BGR2GRAY);
        // Time measurement    
        cv::UMat umat_eqOutput;
        //double opencv_umat_equalizeHist_t1 = cv::getTickCount();
        cv::equalizeHist(umat_grey, umat_eqOutput);
        //double opencv_umat_equalizeHist_t2 = cv::getTickCount();

        //double opencv_umat_equalizeHist_time = (opencv_umat_equalizeHist_t2 - opencv_umat_equalizeHist_t1)/ cv::getTickFrequency();    
        //umat_time.push_back(opencv_umat_equalizeHist_time);

        cnt++;
        if(cnt == frameNumbers) break; 
    }
    double umat_eH_outside_t2 = cv::getTickCount();    
    // total time
    double umat_total_time = (umat_eH_outside_t2 - umat_eH_outside_t1) / cv::getTickFrequency();
    // fps
    fps = frameNumbers / umat_total_time; 
    
    std::cout <<"umat_total_time: "<< umat_total_time << std::endl;
    std::cout <<"EqualizeHist() - fps of UMat: "<< fps << std::endl;
    
    return 0;
}