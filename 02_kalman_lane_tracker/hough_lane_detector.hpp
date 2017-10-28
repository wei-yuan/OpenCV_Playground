#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

#include <vector>
#include <utility>
#include <iostream>
#include <exception>
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class my_logic_exception: public std::exception
{    
};

class HoughLaneDetector
{
protected:
    bool  _prob_hough;
    float _roi_theta = 0.3;
    float _road_horizon; // degree, ex 180
    int   _vote;

public:
    // constrctor & destructor
    HoughLaneDetector(float intput_road_horizon)
    {
        bool  _prob_hough   = true;
        float _roi_theta    = 0.3;
        float _road_horizon = intput_road_horizon;
        float _vote         = 150;
    }
    // Hough Trnasform wrapper to return a series of point
    std::pair<cv::Point, cv::Point> _standard_hough(cv::UMat& frame, float initial_vote)
    {
        // input argument of HoughLines(dst, lines, rho, theta, threshold, srn ,stn)
        // -dst: Output of the edge detector
        // -lines: A vector that will store the parameters (rho, theta) of the detected lines
        // -rho : The resolution of the parameter \r in pixels. we can use 1
        // -theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
        // -threshold: The minimum number of intersections to “detect” a line
        // -srn and -stn: Default parameters to zero. Check OpenCV reference for more info.
        float                  init_vote = initial_vote; // 150, but what's this for?
        std::vector<cv::Vec2f> lines;                
        cv::HoughLines(frame, lines, 1, CV_PI / 180, init_vote, 0, 0);

        cv::Point pt1, pt2;
        auto      l = {pt1, pt2};

        for (auto& line : lines) {
            float rho   = line[0];
            float theta = line[1];

            double x0 = cos(theta) * rho;
            double y0 = sin(theta) * rho;

            pt1.x = cvRound(x0 + 1000 * (-sin(theta)));
            pt1.y = cvRound(y0 + 1000 * (cos(theta)));
            pt2.x = cvRound(x0 - 1000 * (-sin(theta)));
            pt2.y = cvRound(y0 - 1000 * (cos(theta)));
        }
        std::cout << "Member function _standard_hough(). first point: "<< pt1 
        <<  ", second point: "<< pt2 << std::endl;
        return {pt1, pt2};
    }

    float _base_distance(float x1, float y1, float x2, float y2, float width)
    {
        if (x2 == x1) {
            return (width * 0.5) - x1;
        }

        float m          = (y2 - y1) / (x2 - x1);
        float c          = y1 - m * x1;
        float base_cross = -c / m;

        if ((x2 - x1) != 0){
            return (width * 0.5) - base_cross;
        }            
        else
        {
            // do nothing
        }            
    }

    std::pair<cv::Point, cv::Point> _scale_line(float x1, float y1, float x2, float y2, float frame_height)
    {
        float     m; // slope value m
        cv::Point pt1, pt2;

        if (pt1.x == pt1.x) {
            if (pt1.y < y2) {
                pt1.y = _road_horizon;
                pt2.y = frame_height;
                return {pt1, pt2};
            } else {
                pt1.y = frame_height;
                pt2.y = _road_horizon;
                return {pt1, pt2};
            }
        } else {
            // do nothing
        }

        if (pt1.y < y2) {
            if ((pt1.x - pt2.x) != 0) // Denominator shouldn't be zero
            {
                m     = (pt1.y - pt2.y) / (pt1.x - pt2.x);
                pt1.x = ((_road_horizon - pt1.y) / m) + pt1.x;
                pt1.y = _road_horizon;
                pt2.x = ((frame_height - y2) / m) + pt2.x;
                pt2.y = frame_height;
            }
        } else // pt1.y > pt2.y
        {
            if ((pt1.x - pt2.x) != 0) // Denominator shouldn't be zero
            {
                m     = (pt1.y - y2) / (pt1.x - pt2.x);
                pt1.x = ((frame_height - pt1.y) / m) + pt1.x;
                pt1.y = frame_height;
                pt2.x = ((_road_horizon - pt2.y) / m) + pt2.x;
                pt2.y = _road_horizon;
            }
        }
        return {pt1, pt2};
    }

    void detect(cv::UMat& frame)
    {
        std::cout << "HoughLaneDetector detect" << std::endl;        
        
        cv::UMat gray, croppedImg, dst, cdst, blur, contour;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // w = frame.shape[0], h = frame.shape[1]
        int roiy_end = frame.rows;
        int roix_end = frame.cols;

        // crop input image
        // cv::Rect roi(xMin,yMin,xMax-xMin,yMax-yMin);
        cv::Rect myROI(500, 300, 100 * 4, 100 * 3);
        cv::UMat croppedRef(frame, myROI);
        croppedRef.copyTo(croppedImg);
        // edge detection
        cv::Canny(croppedImg, dst, 50, 200, 3);
        std::cout << "Canny" << std::endl;

/*        cv::Rect roi(_road_horizon, 0, (roiy_end - _road_horizon), (roix_end - 0));
        cv::UMat croppedRef(gray, roi);
        croppedRef.copyTo(croppedImg);
        // blurring
        int ksize = 5;
        medianBlur(croppedImg, blur, ksize);
        // edge detection
        cv::Canny(blur, contour, 60, 120);                
        cv::imshow("Canny in class", contour);        
*/        
        std::vector<cv::Vec2f> lines;
        std::vector<cv::Vec4f> plines;
        std::pair<cv::Point, cv::Point> houghlines;                
        
        _prob_hough = true;        
        if (_prob_hough) // true
        {            
            std::cout << "_prob_hough: " << _prob_hough << std::endl;
            // void HoughLinesP(InputArray image, OutputArray lines, 
            //                  double rho, double theta, int threshold, 
            //                  double minLineLength=0, double maxLineGap=0)
            // type of lines = std::vector<cv::Vec2f>, i.e. vector with 2 float .element            
            cv::HoughLinesP(contour, plines, 1, CV_PI / 180, _vote, 30, 100);
           
            cv::Point pt1, pt2;
            auto      l = {pt1, pt2};
    
            for (auto& line : lines) {
                float rho   = line[0];
                float theta = line[1];
    
                double x0 = cos(theta) * rho;
                double y0 = sin(theta) * rho;
    
                pt1.x = cvRound(x0 + 1000 * (-sin(theta)));
                pt1.y = cvRound(y0 + 1000 * (cos(theta)));
                pt2.x = cvRound(x0 - 1000 * (-sin(theta)));
                pt2.y = cvRound(y0 - 1000 * (cos(theta)));                
            }
            cv::cvtColor(dst, cdst, CV_GRAY2BGR);
            // draw line
            cv::line(cdst, houghlines.first, houghlines.second, cvScalar(0, 0, 255), 3, cv::LINE_AA);
            std::cout << houghlines.first << houghlines.second << std::endl;
            // generate line with std::pair<cv::Point, cv::Point> format
            std::cout << "class member function detect. first point: "<< pt1 
            <<  ", second point: "<< pt2 << std::endl;
            cv::imshow("line using HoughLineP", cdst);
            cv::waitKey();
        } 
        else 
        {
            // self defined function, return std::pair<cv::Point, cv::Point>
            houghlines = _standard_hough(dst, _vote);
        }

        //     
        /*if(houghlines)
        {
            auto l = {pt1, pt2};
            for(auto& line : lines) {

            }
        }
        else
        {
            throw HoughLaneDetector;
        }*/
        

//        return {left_bound, right_bound};
    }
};
#endif