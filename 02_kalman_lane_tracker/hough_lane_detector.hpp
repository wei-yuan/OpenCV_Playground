#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

#include <vector>
#include <utility>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class HoughLaneDetector
{
protected:
    bool  prob_hough;
    float roi_theta = 0.3;
    float road_horizon; // degree, ex 180
    int   vote;

public:
    // constrctor & destructor
    HoughLaneDetector(float intput_road_horizon)
    {
        prob_hough   = true;
        roi_theta    = 0.3;
        road_horizon = intput_road_horizon;
        vote         = 50;
    }
    // Hough Trnasform wrapper to return a series of point
    std::pair<cv::Point, cv::Point> _standard_hough(cv::UMat& frame /*, int init_vote*/)
    {
        // input argument of HoughLines(dst, lines, rho, theta, threshold, srn ,stn)
        // -dst: Output of the edge detector
        // -lines: A vector that will store the parameters (rho, theta) of the detected lines
        // -rho : The resolution of the parameter \r in pixels. we can use 1
        // -theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
        // -threshold: The minimum number of intersections to “detect” a line
        // -srn and -stn: Default parameters to zero. Check OpenCV reference for more info.
        int                init_vote = 150; // what's this for?
        std::vector<cv::Vec2f> lines;
        HoughLines(frame, lines, 1, CV_PI/180, init_vote, 0, 0);

        cv::Point pt1, pt2;
        auto l = {pt1, pt2};

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

        return (width * 0.5) - base_cross;
    }
    /*
    float _scale_line(float x1, float y1, float x2, float y2, float frame_height)
    {
        float m; // slope value m
        if (x1 == x2)
        {
            if (y1 < y2)
            {
                //return x1, x2, y1, y2;
            }
            else
            {
                //return x1, x2, y1, y2;
            }
        }
        if (y1 < y2)
        {
            if ((x1 - x2) != 0)
            {
                m = (y1 - y2) / (x1 - x2);
            }
        else // y1 > y2
        {
            if ((x1 - x2) != 0)
            {
                m = (y1 - y2) / (x1 - x2);
            }
        }
        return x1, x2, y1, y2;
    }
    float detect(frame)
    {
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        int roiy_end = frame.shape[0];
        int roix_end = frame.shape[1];
        //roi = img[self.road_horizon:roiy_end, 0:roix_end]

    }
    */
};
#endif