#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

#include <iostream>
#include <vector>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class HoughLaneDetector
{
  protected:
    bool prob_hough;    
    float roi_theta = 0.3;
    float road_horizon;
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
    std::vector<float> _standard_hough(cv::UMat & frame, int init_vote)
    {
        // Hough Trnasform wrapper to return a series of point
        std::vector<float> vec_points = {0.0, 0.0, 0.0, 0.0};
        std::vector<Vec2f> lines;
        //HoughLines(frame, 1, CV_PI/180, init_vote);

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