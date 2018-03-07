#include <assert.h> // assert()
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

int main(int argc, char **argv)
{
    std::string home_path         = std::getenv("HOME");
    std::cout << "home_path: " << home_path << std::endl;
    
    std::string default_file_path = home_path + "/img_and_video_data_set/video/1min/", file_name = "720p_1min.mp4",
                default_input_file = default_file_path + file_name;

    std::string input_file = argv[1] != NULL ? argv[1] : default_input_file;
    std::cout << "input_file: " << input_file << std::endl;

    // video
    cv::VideoCapture capture(input_file);
    if (!capture.isOpened())
    {
        std::cout << "capture not opened..." << std::endl;
        return -1;
    }

    cv::Mat input, five;
    capture >> input;
    cv::cvtColor(input, five, CV_BGR2GRAY);

    cv::Mat *src[5];

    int    num_of_image_per_batch = 5, basic_mem_size = five.rows * five.cols * sizeof(uchar);
    uchar *data = (uchar *)malloc(num_of_image_per_batch * basic_mem_size);

    for (int i = 0; i < num_of_image_per_batch; i++)
    {        
        capture >> input;        
        src[i] = new cv::Mat(five.rows, five.cols, CV_8UC1, &data[i * basic_mem_size]);;
        cv::cvtColor(input, *(src[i]), CV_BGR2GRAY);
    }

    for (int i = 0; i < num_of_image_per_batch; i++)
    {
        std::cout << "i: " << i << std::endl;
        //cv::waitKey(0);        
        cv::imshow("PEWDS YOLO", *(src[i]));
    }

    for (int i = 0; i < num_of_image_per_batch; i++)
    {
        assert( (*src[i]).data == &data[i * basic_mem_size] );        
    }

    // release resource    
    for (int i = 0; i < num_of_image_per_batch; i++)
    {    
        (*src[i]).release();
    }
    free(data);

    return 0;
}