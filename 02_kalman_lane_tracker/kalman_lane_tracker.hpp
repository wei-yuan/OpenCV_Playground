# ifndef KALMAN_LANE_TRACKER_H
# define KALMAN_LANE_TRACKER_H

#include <iostream>
#include <vector>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

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
        //KalmanFilter kf(state_size, meas_size, contr_size);
        //kf.transitionMatrix = ? ;
    }
    ~KalmanLaneTracker() {}
    float _update_dt(float dt)
    {
        for (int i = 0; i < state_size; i++) {
            // kf.transitionMatrix(i, i+1) = dt
        }
    }
    float _first_detect() {}
    float update() {}
    float predict() {}
};
# endif