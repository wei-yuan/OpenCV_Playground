# ifndef KALMAN_LANE_TRACKER_H
# define KALMAN_LANE_TRACKER_H

#include <iostream>
#include <vector>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class KalmanLaneTracker
{
protected:
    int n_lanes;
    int meas_size  = 4 * n_lanes;
    int state_size = 2 * meas_size;
    int contr_size = 0;

public:
    KalmanLaneTracker(int n_lanes, float proc_noise_scale, float meas_noise_scale, float process_cov_parallel, int proc_noise_type)
    {
        
        cv::KalmanFilter kf(state_size, meas_size, contr_size);        
        kf.transitionMatrix = cv::Mat::eye(state_size, state_size, CV_32F); // dtype = 32 bit float
        kf.measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);
        /* Need transformation from numpy to OpenCV format
        for(int i = 0; i < sizeof(meas_size); i++)
        {
            kf.transitionMatrix = (Mat_<CV_32F>( , )  << 1);
        }
        */
        if (proc_noise_type == 'white')
        {
            //cv::UMat block = {{0.25, 0.5]},{0.5, 1}};
            //kf.processNoiseCov = ;
        }
        if(proc_noise_type == 'identity')
        {
            kf.processNoiseCov = cv::Mat::eye(state_size, state_size, CV_32F);
        }
        for(int i=0; i< meas_size; i+=2)
        {
            for(int j=0; j< n_lanes; j++)
            {
                //kf.processNoiseCov ...
            }
        }
        kf.measurementNoiseCov = cv::Mat::eye(meas_size, meas_size, CV_32F);
    }
    /*
    float _update_dt(float dt)
    {
        for (int i = 0; i < state_size; i++) {
            // kf.transitionMatrix(i, i+1) = dt
        }
    }
    float _first_detect() {}
    float update() {}
    float predict() {}
    */
};
# endif