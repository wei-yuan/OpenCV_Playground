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
    int  _n_lanes;
    int  _meas_size;
    int  _state_size;
    int  _contr_size = 0;
    bool _first_detected;

public:
    // constructor
    KalmanLaneTracker(int _n_lanes, float proc_noise_scale, float meas_noise_scale)
    {
        float process_cov_parallel = 0;
        char  proc_noise_type[]    = "white";
        _meas_size                 = 4 * _n_lanes;
        _state_size                = 2 * _meas_size;

        std::cout << "n_lanes: " << _n_lanes << ", meas_size: " << _meas_size << ", state_size: " << _state_size
                  << ", _contr_size: " << _contr_size << std::endl;

        cv::KalmanFilter kf(_state_size, _meas_size, _contr_size);

        kf.transitionMatrix  = cv::Mat::eye(_state_size, _state_size, CV_32F); // dtype = 32 bit float
        kf.measurementMatrix = cv::Mat::zeros(_meas_size, _state_size, CV_32F);

        for (int i = 0; i < _meas_size; i++) {
            kf.measurementMatrix.at<float>(i, i * 2) = 1; // access point (i, i*2) and assign it to 1;
        }

        if (strncmp(proc_noise_type, "white", 2) == 0) // 2 -> compare two elements
        {
            int     size_block = 2;
            cv::Mat block      = (cv::Mat_<float>(size_block, size_block) << 0.25, 0.5, 0.5, 1);
            std::cout << "block: \n" << block << std::endl;

            // assignment of kf.processNoiseCov matrix
            kf.processNoiseCov = cv::Mat::zeros(size_block * _meas_size, size_block * _meas_size, CV_32F) * proc_noise_scale;
            for (int i = 0; i < _meas_size *2 ; i+=2) // 0, 2, 4, 6
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
            kf.processNoiseCov = cv::Mat::eye(_state_size, _state_size, CV_32F);
        }
        for (int i = 0; i < _meas_size; i += 2) 
        {
            for (int j = 1; j < _n_lanes; j++) // 1 ???
            {                
                kf.processNoiseCov.at<float>(i, i+(j*8)) = process_cov_parallel; 
                kf.processNoiseCov.at<float>(i+(j*8), i) = process_cov_parallel;                 
            }
        }
        std::cout << kf.processNoiseCov << std::endl;

        kf.measurementNoiseCov = cv::Mat::eye(_meas_size, _meas_size, CV_32F);
        kf.errorCovPre         = cv::Mat::eye(_state_size, _state_size, CV_32F);

        cv::Mat meas  = cv::Mat::zeros(_meas_size, 1, CV_32F);
        cv::Mat state = cv::Mat::zeros(_state_size, 1, CV_32F);

        _first_detected = false;
    }
    
    void _update_dt(cv::KalmanFilter& kf, float dt)
    {
        for (int i = 0; i < _state_size; i+=2) {
            kf.transitionMatrix.at<float>(i, i+1) = dt;
        }
    }
    // Mat state is defined in constructor
    float _first_detect(cv::KalmanFilter& kf, std::vector<my::Line> lines)
    {
        for(auto& l : lines)
        {
            kf.state[i:i+8:2, 0] = l;
        }
        for lane, i in zip(lanes, range(0, _state_size, 8)): // lanes -> (x1, y1) and (x2, y2)
            
        
        kf.statePost = state;
        _first_detected = true;
    }
    
    void update(cv::KalmanFilter& kf, cv::Mat& meas, int lanes)
    {
        if(_first_detected == true ) 
        {            
/*            for lane, i in zip(lanes, range(0, self.meas_size, 4))
            {
                if(lane != NULL)
                    meas[i:i+4, 0] = lane;
            }*/
            kf.correct(meas);
        }
    }
    float predict(float dt)
    {
        if(_first_detected)
        {
/*            _update_dt(dt);
            state = kf.predict();
            std::vector<cv::Vec2f> lines;
            for i in range(0, len(state), 8):
                lanes.append((state[i], state[i+2], state[i+4], state[i+6]))
            return lanes
*/            
        }
        else
        {
            return NULL;
        }        
    }    
};
#endif