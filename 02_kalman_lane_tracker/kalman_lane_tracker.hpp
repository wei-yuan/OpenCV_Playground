# ifndef KALMAN_LANE_TRACKER_H
# define KALMAN_LANE_TRACKER_H

#include <vector>
#include <string.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class KalmanLaneTracker
{
protected:
    int _n_lanes;
    int _meas_size;
    int _state_size;
    int _contr_size = 0;
    bool _first_detected;

public:
    KalmanLaneTracker(int _n_lanes, float proc_noise_scale, float meas_noise_scale)
    {
        float process_cov_parallel = 0.0;
        char proc_noise_type[] = "white";
        _meas_size  = 4 * _n_lanes;
        _state_size = 2 * _meas_size;
        
        std::cout << "n_lanes: " << _n_lanes << ", meas_size: "  << _meas_size << 
        ", state_size: " << _state_size << ", _contr_size: "  << _contr_size << std::endl;
        
        cv::KalmanFilter kf(_state_size, _meas_size, _contr_size);        
        
        kf.transitionMatrix = cv::Mat::eye(_state_size, _state_size, CV_32F); // dtype = 32 bit float        
        kf.measurementMatrix = cv::Mat::zeros(_meas_size, _state_size, CV_32F);
                        
        for(int i = 0; i < sizeof(_meas_size); i++)
        {
            kf.measurementMatrix.at<float>(i, i*2) = 1; // access point (i, i*2) and assign it to 1;
        }                
        
        if( strncmp( proc_noise_type, "white", 2) == 0) // 2 -> compare two elements
        {
            cv::Mat block = (cv::Mat_<float>(2,2) << 0.25, 0.5, 0.5, 1);
            //kf.processNoiseCov = block_diag(*([block] * _meas_size)) * proc_noise_scale
        }
        if( strncmp( proc_noise_type, "identity", 2) == 0)
        {
            kf.processNoiseCov = cv::Mat::eye(_state_size, _state_size, CV_32F);
        }
        for(int i=0; i< _meas_size; i+=2)
        {
            for(int j=0; j< _n_lanes; j++)
            {
                //kf.processNoiseCov ...
            }
        }        

 /*       kf.measurementNoiseCov = cv::Mat::eye(_meas_size, _meas_size, CV_32F);
        kf.errorCovPre = cv::Mat::eye(_state_size, _state_size, CV_32F);
        
        cv::Mat meas = cv::Mat::zeros(_meas_size, 1, CV_32F);        
        cv::Mat state = cv::Mat::zeros(_state_size, 1, CV_32F);        

        _first_detected = false;
*/        
    }
    /*
    float _update_dt(float dt)
    {
        for (int i = 0; i < state_size; i+=2) {
            kf.measurementMatrix.at<float>(i, i+1) = dt;
        }
    }
    float _first_detect(int lanes) 
    {
        for l, i in zip(lanes, range(0, state_size, 8)):
            state[i:i+8:2, 0] = l
        kf.statePost = state;
        first_detected = true;
    }
    float update(int lanes) 
    {
        if(first_detected) // if first_detected = true
        {
            for l, i in zip(lanes, range(0, self.meas_size, 4))
            {
                if(l != NULL)
                    meas[i:i+4, 0] = l;
            }            
            kf.correct(meas);
        }
    }
    float predict(float dt) 
    {
        if(first_detected)
        {
            _update_dt(dt);
            state = kf.predict();
            std::vector<cv::Vec2f> lines;
            for i in range(0, len(state), 8):
                lanes.append((state[i], state[i+2], state[i+4], state[i+6]))
            return lanes
        }
        else
        {
            return NULL;
        }
    }
    */
};
# endif