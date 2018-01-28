//--------------------------------------------------------------
/*
Author: Wei-Yuan Alex Hsu
Date:   2018/1/19 Fri
Target: implement non-parallel and parallel( OpenCL based ) histogram equalization
Reference:
[1] Histogram Calculation -
https://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html#code
[2] How to scan images, lookup tables and time measurement with OpenCV -
https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv

*/
//--------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <vector>
#include <unistd.h>
//#include <chrono>  // c++11

// OpenCV Library
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "main.hpp"

namespace CPU
{

float run_equalizeHist(cv::VideoCapture video)
{
    video.set(cv::CAP_PROP_POS_FRAMES, 0);

    //auto start = std::chrono::system_clock::now();
    double start = (double)cv::getTickCount();
    for (;;) {
        cv::Mat frame, grey;
        cv::Mat eqOutput;
        if (video.read(frame) == false) break;

        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(grey, eqOutput);
    }
    //auto end = std::chrono::system_clock::now();
    double end = (double)cv::getTickCount();

    //std::chrono::duration<double> elapsed_seconds = end - start;
    double elapsed_seconds = (end - start) / cv::getTickFrequency();

/*
    std::cout << "Elapse Time: " << elapsed_seconds.count() << std::endl;
    std::cout << "FPS " << video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds.count()
              << std::endl;
*/              
    std::cout << "Elapse Time: " << elapsed_seconds << std::endl;
    std::cout << "FPS " << video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds
              << std::endl;

    return video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds;
}
}

namespace GPU
{

float run_equalizeHist(cv::VideoCapture video)
{
    video.set(cv::CAP_PROP_POS_FRAMES, 0);

    //auto start = std::chrono::system_clock::now();
    double start = (double)cv::getTickCount();

    int frameNumber = video.get(cv::CAP_PROP_FRAME_COUNT);
    #pragma omp parallel for // OpenMP
    for (int i = 0; i < frameNumber; i++) {
        cv::UMat frame, grey;
        cv::UMat eqOutput;
        #pragma omp critical
        {
            video.read(frame);
        }
        //if (video.read(frame) == false) break;

        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(grey, eqOutput);
    }
    //auto end = std::chrono::system_clock::now();
    double end = (double)cv::getTickCount();

    double elapsed_seconds = (end - start) / cv::getTickFrequency();
/*
    std::cout << "Elapse Time: " << elapsed_seconds.count() << std::endl;
    std::cout << "FPS " << video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds.count()
              << std::endl;
*/
    std::cout << "Elapse Time: " << elapsed_seconds << std::endl;
    std::cout << "FPS " << video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds
              << std::endl;               

    return video.get(cv::CAP_PROP_FRAME_COUNT) / elapsed_seconds;
}
}

int main(int argc, char *argv[])
{
    std::string file_path ("../../img_and_video_data_set/video/1min");
    std::string file_name ("720p_1min.mp4");
    std::string input_file_video = file_path + '/' + file_name;

    std::string input_file = (argc == 2) ? std::string(argv[1]) : input_file_video;

    cv::VideoCapture video(input_file);

    // check video file
    if (!video.isOpened()) {
        std::cout << "File not found: " << input_file << std::endl;
        exit(1);
    }

    std::cout << "frameNumbers: " << video.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;

    float cpu_fps = 0, gpu_fps = 0;
    cpu_fps = CPU::run_equalizeHist(video);
    gpu_fps = GPU::run_equalizeHist(video);

    // speed up
    std::cout << "Speed-up of OpenCL: " << gpu_fps / cpu_fps << std::endl;

    return 0;
}
