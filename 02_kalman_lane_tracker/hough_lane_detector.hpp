#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

// cpp header
#include <exception>
#include <iostream>
#include <math.h>    /* atan2 */
#include <tuple>
#include <utility>
#include <vector>
// OpenCV header
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
        //auto      l = {pt1, pt2};

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

    std::vector<cv::Vec4f> _Vec4f_copy_to_2dvector(std::vector<cv::Vec4f>& one_side_of_lane_bound, std::vector<std::vector<float>>& lane_pair_point)
    {
        // copy value back for return
        for (size_t i = 0; i < one_side_of_lane_bound.size(); i++) 
        {
            for (int j = 0; j < 4; j++) 
            {
                one_side_of_lane_bound[j] = lane_pair_point[i][j];
            }
        }
        return one_side_of_lane_bound;
    }

    std::vector<cv::Vec4f> _scale_line(std::vector<cv::Vec4f>& one_side_of_lane_bound, float frame_height)
    {
        std::cout << "_scale_line" << std:: endl;
        float m; // slope value m
        // x1: one_side_of_lane_bound[0], y1: one_side_of_lane_bound[1]
        // x2: one_side_of_lane_bound[2], y2: one_side_of_lane_bound[3]        
        
        std::vector<std::vector<float>> lane_pair_point(1, std::vector<float>(4));        
        // copy Vec4f value for comparing
        for(size_t i=0; i < one_side_of_lane_bound.size(); i++)
        {    
            const cv::Vec4f& one = one_side_of_lane_bound[i];        
            for(int j=0; j<4; j++)
            {                
                lane_pair_point[i][j] = one[j];            
            }            
        }

        // convert, for readable
        float x1, y1, x2, y2;
        x1 = lane_pair_point[0][0]; y1 = lane_pair_point[0][1];
        x2 = lane_pair_point[0][2]; y2 = lane_pair_point[0][3];

        if (x1 == x2) {
            if (y1 < y2) {
                lane_pair_point[0][1] = _road_horizon;
                lane_pair_point[0][3] = frame_height;                
                one_side_of_lane_bound = _Vec4f_copy_to_2dvector(one_side_of_lane_bound, lane_pair_point);

                for (int j = 0; j < 4; j++) 
                {
                    std::cout << "x1 == x2, y1 < y2" << one_side_of_lane_bound[j] << std:: endl; 
                }

                return one_side_of_lane_bound;
            } 
            else 
            {
                lane_pair_point[0][1] = frame_height;
                lane_pair_point[0][3] = _road_horizon;
                one_side_of_lane_bound = _Vec4f_copy_to_2dvector(one_side_of_lane_bound, lane_pair_point);
                
                for (int j = 0; j < 4; j++) 
                {
                    std::cout << "x1 == x2, y1 > y2" << one_side_of_lane_bound[j] << std:: endl; 
                }

                return one_side_of_lane_bound;
            }
        } 
        else 
        {
            // do nothing
        }

        if (y1 < y2) 
        {
            if ((x1 - x2) != 0) // Denominator shouldn't be zero
            {
                m     = (y1 - y2) / (x1 - x2);
                lane_pair_point[0][0] = ((_road_horizon - y1) / m) + x1; //x1
                lane_pair_point[0][1] = _road_horizon;                   //y1
                lane_pair_point[0][2] = ((frame_height - y2) / m) + x2;  //x2
                lane_pair_point[0][3] = frame_height;                    //y2 
            }
        } 
        else // y1 > y2
        {
            if ((x1 - x2) != 0) // Denominator shouldn't be zero
            {
                m     = (y1 - y2) / (x1 - x2);
                lane_pair_point[0][0] = ((frame_height - y1) / m) + x1;
                lane_pair_point[0][1] = frame_height;
                lane_pair_point[0][2] = ((_road_horizon - y2) / m) + x2;
                lane_pair_point[0][3] = _road_horizon;
            }
        }
        // copy to lane(Vec4f)
        one_side_of_lane_bound = _Vec4f_copy_to_2dvector(one_side_of_lane_bound, lane_pair_point);

        for (int j = 0; j < 4; j++) 
        {
            std::cout << one_side_of_lane_bound[j] << std:: endl; 
        }

        return one_side_of_lane_bound;
    }

    std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> detect(cv::UMat& frame)
    {                
        cv::UMat gray, croppedImg, dst, cdst, blur, contour;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // w = frame.shape[0], h = frame.shape[1]
        int roiy_end = frame.rows;
        int roix_end = frame.cols;

        // crop input image
        // cv::Rect roi(xMin,yMin,xMax-xMin,yMax-yMin);
        //cv::Rect roi(_road_horizon, 0, (roiy_end - _road_horizon), (roix_end - 0));
        cv::Rect myROI(500, 300, 100 * 4, 100 * 3);
        cv::UMat croppedRef(frame, myROI);
        croppedRef.copyTo(croppedImg);
        // blurring
        int ksize = 5;
        cv::medianBlur(croppedImg, blur, ksize);
        // edge detection
        cv::Canny(blur, contour, 60, 120);                        
        
        //std::vector<cv::Vec2f> lines;
        std::vector<cv::Vec4f> plines;
        std::pair<cv::Point, cv::Point> houghlines;                
        
        /* switch HoughlineP() or Houghline() here */
        //_prob_hough = true;        
        if (_prob_hough) // true
        {                        
            // void HoughLinesP(InputArray image, OutputArray lines, 
            //                  double rho, double theta, int threshold, 
            //                  double minLineLength=0, double maxLineGap=0)
            // type of lines = std::vector<cv::Vec4f>, i.e. vector with 4 float element            
            cv::HoughLinesP(contour, plines, 1, CV_PI / 180, _vote, 0, 0);
           
            cv::Point pt1, pt2;
            //auto      l = {pt1, pt2};
    
            for (auto& line : plines) {
                float rho   = line[0];
                float theta = line[1];
    
                double x0 = cos(theta) * rho;
                double y0 = sin(theta) * rho;
    
                pt1.x = cvRound(x0 + 1000 * (-sin(theta)));
                pt1.y = cvRound(y0 + 1000 * (cos(theta)));
                pt2.x = cvRound(x0 - 1000 * (-sin(theta)));
                pt2.y = cvRound(y0 - 1000 * (cos(theta)));                
            }
            
            // prepare the color canvas for output image 
            cv::cvtColor(contour, cdst, CV_GRAY2BGR);            
            // draw line
            houghlines = {pt1, pt2};            
            cv::line(cdst, houghlines.first, houghlines.second, cvScalar(0, 0, 255), 3, cv::LINE_AA);                        
            // generate line with std::pair<cv::Point, cv::Point> format
            std::cout << "class member function detect. first point: "<< pt1 
            <<  ", second point: "<< pt2 << std::endl;
            //cv::imshow("line using HoughLineP", cdst);
            //cv::waitKey();
        } 
        else 
        {
            // self defined function, return std::pair<cv::Point, cv::Point>
            houghlines = _standard_hough(dst, _vote);
        } 
        
        // !!find nearest lines to center!!
//        lines = lines+np.array([0, self.road_horizon, 0, self.road_horizon]).reshape((1, 1, 4))
        // scale points from ROI coordinates to full frame coordinates
        std::vector<cv::Vec4f> left_bound, right_bound, scale_pair_left, scale_pair_right;
        // find the rightmost line of the left half of the frame
        // and the leftmost line of the right half
            
        float theta, left_dist, right_dist;
        for (auto& line : plines)
        {
            theta = std::abs(atan2((houghlines.second.y - houghlines.first.y), (houghlines.second.x-houghlines.first.x)));
            if (theta > _roi_theta)
            {
                float dist = _base_distance(houghlines.first.x, houghlines.first.y, houghlines.second.x, houghlines.second.y, frame.cols);
                if(dist < 0)
                {
                    left_bound[0] = houghlines.first.x;
                    left_bound[1] = houghlines.first.y;
                    left_bound[2] = houghlines.second.x;
                    left_bound[3] = houghlines.second.y;
                    left_dist = dist;
                }
                else if(dist > 0)
                {
                    right_bound[0] = houghlines.first.x;
                    right_bound[1] = houghlines.first.y;
                    right_bound[2] = houghlines.second.x;
                    right_bound[3] = houghlines.second.y;
                    right_dist = dist;
                }
                else if(0 > dist > left_dist)
                {
                    left_bound[0] = houghlines.first.x;
                    left_bound[1] = houghlines.first.y;
                    left_bound[2] = houghlines.second.x;
                    left_bound[3] = houghlines.second.y;
                    left_dist = dist;
                }
                else if(0 > dist > right_dist)
                {
                    right_bound[0] = houghlines.first.x;
                    right_bound[1] = houghlines.first.y;
                    right_bound[2] = houghlines.second.x;
                    right_bound[3] = houghlines.second.y;
                    right_dist = dist;
                }
            }
        }    
        
        for (int j = 0; j < 4; j++) 
        {
            std::cout << "left_bound[" << j << "]: " << left_bound[j] << std:: endl; 
        }

        // _scale_line return std::pair<cv::Point, cv::Point>
        scale_pair_left  = _scale_line(left_bound, frame.rows);
        scale_pair_right = _scale_line(right_bound, frame.rows);

        return {scale_pair_left, scale_pair_right};
    }    
};

// Exception
class my_logic_exception: public std::exception
{    
};
#endif