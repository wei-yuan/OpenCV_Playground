#ifndef KALMAN_LANE_TRACKER_H
#define KALMAN_LANE_TRACKER_H

#include <vector>
#include <string.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "hough_lane_detector.hpp"

// performance boost: Matrix access
class KalmanLaneTracker
{
protected:
    int  n_lanes;
    int  meas_size;
    int  state_size;
    int  contr_size = 0;
    bool first_detected;
    
public:
    cv::KalmanFilter kf;
    cv::Mat meas, state;

    // constructor
    KalmanLaneTracker(int n_lanes, float proc_noise_scale, float meas_noise_scale)
    {
        float process_cov_parallel = 0;
        char  proc_noise_type[]    = "white";
        this->meas_size            = 4 * this->n_lanes;
        this->state_size           = 2 * this->meas_size;

        std::cout << "n_lanes: " << n_lanes << ", meas_size: " << meas_size << ", state_size: " << state_size
                  << ", _contr_size: " << contr_size << std::endl;                

        kf.transitionMatrix  = cv::Mat::eye(state_size, state_size, CV_32F); // dtype = 32 bit float
        kf.measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);

        for (int i = 0; i < meas_size; i++) {
            kf.measurementMatrix.at<float>(i, i * 2) = 1; // access point (i, i*2) and assign it to 1;
        }

        if (strncmp(proc_noise_type, "white", 2) == 0) // 2 -> compare two elements
        {
            int     size_block = 2;
            cv::Mat block      = (cv::Mat_<float>(size_block, size_block) << 0.25, 0.5, 0.5, 1);
            std::cout << "block: \n" << block << std::endl;

            // assignment of kf.processNoiseCov matrix
            kf.processNoiseCov = cv::Mat::zeros(size_block * meas_size, size_block * meas_size, CV_32F) * proc_noise_scale;
            for (int i = 0; i < meas_size *2 ; i+=2) // 0, 2, 4, 6
            {                
                for (int j = 0; j < 2; j++) // 0 ,1
                {             
                    for(int k = 0; k < 2; k++) // 0 ,1
                    {                                         
                        kf.processNoiseCov.at<float>(i+k,i+j) = block.at<float>(k, j);
                    }                    
                }
            }            
        }
        if (strncmp(proc_noise_type, "identity", 2) == 0) {
            kf.processNoiseCov = cv::Mat::eye(state_size, state_size, CV_32F);
        }
        for (int i = 0; i < meas_size; i += 2) 
        {
            for (int j = 1; j < n_lanes; j++) // 1 ???
            {                
                kf.processNoiseCov.at<float>(i, i+(j*8)) = process_cov_parallel; 
                kf.processNoiseCov.at<float>(i+(j*8), i) = process_cov_parallel;                 
            }
        }        

        kf.measurementNoiseCov = cv::Mat::eye(meas_size, meas_size, CV_32F);
        kf.errorCovPre         = cv::Mat::eye(state_size, state_size, CV_32F);

        cv::Mat meas  = cv::Mat::zeros(meas_size, 1, CV_32F);
        cv::Mat state = cv::Mat::zeros(state_size, 1, CV_32F);

        first_detected = false;
    }
    
    void update_dt( float dt)
    {
        for (int i = 0; i < state_size; i+=2) {
            kf.transitionMatrix.at<float>(i, i+1) = dt;
        }
    }
    // Mat state is defined in constructor
    float first_detect( my::Line lines )
    {
        //for(auto& l : lines) // lines -> (x1, y1) and (x2, y2)
        //{            
            state.at<float>(0, 0) = lines.beg.x;            
            state.at<float>(2, 0) = lines.beg.y;            
            state.at<float>(4, 0) = lines.end.x;            
            state.at<float>(6, 0) = lines.end.y;            
        //}                    
        
        kf.statePost = state;
        this->first_detected = true;
    }
    
    void update( my::Line lines )
    {
        if( this->first_detected == true ) 
        {
            //for (auto& l : lines) // lines -> (x1, y1) and (x2, y2)
            //{
                meas.at<float>(0, 0) = lines.beg.x;
                meas.at<float>(1, 0) = lines.beg.y;
                meas.at<float>(2, 0) = lines.end.x;
                meas.at<float>(3, 0) = lines.end.y;
            //}
            kf.correct(meas);
        }
        else
        {
            int x1 = lines.beg.x;
            int y1 = lines.beg.y;
            int x2 = lines.end.x;
            int y2 = lines.end.x;
            if( ((x1 == 0) && (y1 == 0)) && ((x2 == 0) && (y2 == 0)) )
            {
                first_detect( lines );
            }
        }
    }
    std::vector<my::Line> predict( float dt)
    {
        if( this->first_detected == true )
        {
            update_dt(dt);
            state = kf.predict();
            
            std::vector<my::Line> lines;                        
            lines.resize(100);                        

            for (int i=0; i < 8; i++) 
            {
                cv::Point2f beg = {state.at<float>(i, 0), state.at<float>(i + 2, 0)};
                cv::Point2f end   = {state.at<float>(i + 4, 0), state.at<float>(i + 6, 0)};
                lines.push_back({ beg, end });
                std::cout << "begin (x, y)= " << "(" << beg.x << "," << beg.y << ")" << std::endl;
                std::cout << "End (x, y)= " << "(" << end.x << "," << end.y << ")" << std::endl;
            }
            return lines;
        }                
    }    
};
#endif