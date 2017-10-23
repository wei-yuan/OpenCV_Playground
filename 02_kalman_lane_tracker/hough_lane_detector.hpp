#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

#include <vector>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class HoughLaneDetector
{
  protected:
    bool prob_hough;    
    float roi_theta = 0.3;
    float road_horizon;    // degree, ex 180
    int vote;

  public:
    // constrctor & destructor
    HoughLaneDetector(float intput_road_horizon)
    {
        prob_hough = true;
        roi_theta = 0.3;
        road_horizon = intput_road_horizon;
        vote = 50;
    }
    //
    std::vector<std::vector<float>> _standard_hough(cv::UMat & frame/*, int init_vote*/)
    {
        // Hough Trnasform wrapper to return a series of point
        UMat dst;
        std::vector<std::vector<float>> vec_points(0); // dynamic allocation
        std::vector<float> pts(0);
        std::vector<Vec2f> lines;
        
        // input argument of HoughLines(dst, lines, rho, theta, threshold, srn ,stn)
        // -dst: Output of the edge detector
        // -lines: A vector that will store the parameters (rho, theta) of the detected lines
        // -rho : The resolution of the parameter \r in pixels. we can use 1
        // -theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
        // -threshold: The minimum number of intersections to “detect” a line
        // -srn and -stn: Default parameters to zero. Check OpenCV reference for more info.
        int init_vote = 50; // what's this for?
        HoughLinesP(frame, lines, 1, CV_PI/180, init_vote, 50, 10);
        
        for(size_t i = 0; i < lines.size(); i++)
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;

            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));

            pts[0] = pt1.x;
            pts[1] = pt1.y;
            pts[2] = pt2.x;
            pts[3] = pt2.y;                
            
            line( dst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
            //vec_points[i].push_back(pts);
            vec_points.push_back(pts);
        }        
        return vec_points;
    }
    // float _base_distance(float x1, float y1, float x2, float y2, float width) {}
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