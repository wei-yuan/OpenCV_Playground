#include <iostream>
#include <vector>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;


struct mouse_info_struct { int x,y; };
struct mouse_info_struct mouse_info = {-1,-1}, last_mouse;

vector<Point> mousev,kalmanv;
int trackbarProcessNoiseCov = 10, trackbarMeasurementNoiseCov = 10, trackbarStateEstimationErrorCov = 10;

float processNoiseCov=10, measurementNoiseCov = 1000, stateEstimationErrorCov = 50;
int trackbarProcessNoiseCovMax=10000, trackbarMeasurementNoiseCovMax = 10000,
      trackbarStateEstimationErrorCovMax = 5000;

float processNoiseCovMin=0, measurementNoiseCovMin = 0,
      stateEstimationErrorCovMin = 0;
float processNoiseCovMax=100, measurementNoiseCovMax = 5000,
      stateEstimationErrorCovMax = 5000;

int nStates = 5, nMeasurements = 2, nInputs = 1;
KalmanFilter KF(nStates, nMeasurements, nInputs, CV_64F);

void on_mouse(int event, int x, int y, int flags, void* param)
{
    last_mouse = mouse_info;
    mouse_info.x = x;
    mouse_info.y = y;
}

void on_trackbarProcessNoiseCov( int, void* )
{
    processNoiseCov = processNoiseCovMin +
        (trackbarProcessNoiseCov * (processNoiseCovMax-processNoiseCovMin)/trackbarProcessNoiseCovMax);
    setIdentity(KF.processNoiseCov, Scalar::all(processNoiseCov));
    std::cout << "\nProcess Noise Cov:     " << processNoiseCov;
    std::cout << "\nMeasurement Noise Cov: " << measurementNoiseCov << std::endl;
}

void on_trackbarMeasurementNoiseCov( int, void* )
{
    measurementNoiseCov = measurementNoiseCovMin +
(trackbarMeasurementNoiseCov * (measurementNoiseCovMax-measurementNoiseCovMin)/trackbarMeasurementNoiseCovMax);
    setIdentity(KF.measurementNoiseCov, Scalar::all(measurementNoiseCov));
    std::cout << "\nProcess Noise Cov:     " << processNoiseCov;
    std::cout << "\nMeasurement Noise Cov: " << measurementNoiseCov << std::endl;
}

int main (int argc, char * const argv[])
{
    Mat img(500, 1000, CV_8UC3);
    Mat state(nStates, 1, CV_64F);/* (x, y, Vx, Vy, a) */
    Mat measurementNoise(nMeasurements, 1, CV_64F), processNoise(nStates, 1, CV_64F);
    Mat measurement(nMeasurements,1,CV_64F); measurement.setTo(Scalar(0.0));
    Mat noisyMeasurement(nMeasurements,1,CV_64F); noisyMeasurement.setTo(Scalar(0.0));
    Mat prevMeasurement(nMeasurements,1,CV_64F); prevMeasurement.setTo(Scalar(0.0));
    Mat prevMeasurement2(nMeasurements,1,CV_64F); prevMeasurement2.setTo(Scalar(0.0));
    int key = -1, dt=50, T=1000;


    namedWindow("Mouse-Kalman");
    setMouseCallback("Mouse-Kalman", on_mouse, 0);
    createTrackbar( "Process Noise Cov", "Mouse-Kalman", &trackbarProcessNoiseCov,
            trackbarProcessNoiseCovMax, on_trackbarProcessNoiseCov );
    createTrackbar( "Measurement Noise Cov", "Mouse-Kalman", &trackbarMeasurementNoiseCov,
            trackbarMeasurementNoiseCovMax, on_trackbarMeasurementNoiseCov );

    on_trackbarProcessNoiseCov( trackbarProcessNoiseCov, 0 );
    on_trackbarMeasurementNoiseCov( trackbarMeasurementNoiseCov, 0 );

    //while ( (char)(key=cv::waitKey(100)) != 'q' )
    {
    /// A (TRANSITION MATRIX INCLUDING VELOCITY AND ACCELERATION MODEL)
    KF.transitionMatrix.at<double>(0,0) = 1;
    KF.transitionMatrix.at<double>(0,1) = 0;
    KF.transitionMatrix.at<double>(0,2) = (dt/T);
    KF.transitionMatrix.at<double>(0,3) = 0;
    KF.transitionMatrix.at<double>(0,4) = 0.5*(dt/T)*(dt/T);

    KF.transitionMatrix.at<double>(1,0) = 0;
    KF.transitionMatrix.at<double>(1,1) = 1;
    KF.transitionMatrix.at<double>(1,2) = 0;
    KF.transitionMatrix.at<double>(1,3) = (dt/T);
    KF.transitionMatrix.at<double>(1,4) = 0.5*(dt/T)*(dt/T);

    KF.transitionMatrix.at<double>(2,0) = 0;
    KF.transitionMatrix.at<double>(2,1) = 0;
    KF.transitionMatrix.at<double>(2,2) = 1;
    KF.transitionMatrix.at<double>(2,3) = 0;
    KF.transitionMatrix.at<double>(2,4) = (dt/T);

    KF.transitionMatrix.at<double>(3,0) = 0;
    KF.transitionMatrix.at<double>(3,1) = 0;
    KF.transitionMatrix.at<double>(3,2) = 0;
    KF.transitionMatrix.at<double>(3,3) = 1;
    KF.transitionMatrix.at<double>(3,4) = (dt/T);

    KF.transitionMatrix.at<double>(4,0) = 0;
    KF.transitionMatrix.at<double>(4,1) = 0;
    KF.transitionMatrix.at<double>(4,2) = 0;
    KF.transitionMatrix.at<double>(4,3) = 0;
    KF.transitionMatrix.at<double>(4,4) = 1;


    /// Initial estimate of state variables
    KF.statePost = cv::Mat::zeros(nStates, 1,CV_64F);
    KF.statePost.at<double>(0) = mouse_info.x;
    KF.statePost.at<double>(1) = mouse_info.y;
    KF.statePost.at<double>(2) = 0.1;
    KF.statePost.at<double>(3) = 0.1;
    KF.statePost.at<double>(4) = 0.1;

    KF.statePre = KF.statePost;
    state = KF.statePost;

    /// Ex or Q (PROCESS NOISE COVARIANCE)
    setIdentity(KF.processNoiseCov, Scalar::all(processNoiseCov));


    /// Initial covariance estimate Sigma_bar(t) or P'(k)
    setIdentity(KF.errorCovPre, Scalar::all(stateEstimationErrorCov));

    /// Sigma(t) or P(k) (STATE ESTIMATION ERROR COVARIANCE)
    setIdentity(KF.errorCovPost, Scalar::all(stateEstimationErrorCov));

    /// B (CONTROL MATRIX)
    KF.controlMatrix = cv::Mat(nStates, nInputs,CV_64F);
    KF.controlMatrix.at<double>(0,0) = /*0.5*(dt/T)*(dt/T);//*/0;
    KF.controlMatrix.at<double>(1,0) = /*0.5*(dt/T)*(dt/T);//*/0;
    KF.controlMatrix.at<double>(2,0) = 0;
    KF.controlMatrix.at<double>(3,0) = 0;
    KF.controlMatrix.at<double>(4,0) = 1;

    /// H (MEASUREMENT MATRIX)
    KF.measurementMatrix = cv::Mat::eye(nMeasurements, nStates, CV_64F);

    /// Ez or R (MEASUREMENT NOISE COVARIANCE)
    setIdentity(KF.measurementNoiseCov, Scalar::all(measurementNoiseCov));


    while (mouse_info.x < 0 || mouse_info.y < 0)
    {
        imshow("Mouse-Kalman", img);
        waitKey(30);
        continue;
    }

    prevMeasurement.at<double>(0,0) = 0;
    prevMeasurement.at<double>(1,0) = 0;
    prevMeasurement2 = prevMeasurement;

    while ( (char)key != 's' )
    {
        /// STATE UPDATE
        Mat prediction = KF.predict();

        /// MAKE A MEASUREMENT
        measurement.at<double>(0) = mouse_info.x;
        measurement.at<double>(1) = mouse_info.y;

        /// MEASUREMENT NOISE SIMULATION
        randn( measurementNoise, Scalar(0),
          Scalar::all(sqrtf(measurementNoiseCov)));
        noisyMeasurement = measurement + measurementNoise;

        /// MEASUREMENT UPDATE
        Mat estimated = KF.correct(noisyMeasurement);

        cv::Mat u(nInputs,1,CV_64F);
        u.at<double>(0,0) = 0.0 * sqrtf(pow((prevMeasurement.at<double>(0) - measurement.at<double>(0)),2)
                    + pow((prevMeasurement.at<double>(1) - measurement.at<double>(1)),2));

        /// STORE ALL DATA
        Point noisyPt(noisyMeasurement.at<double>(0),noisyMeasurement.at<double>(1));
        Point estimatedPt(estimated.at<double>(0),estimated.at<double>(1));
        Point measuredPt(measurement.at<double>(0),measurement.at<double>(1));


        /// PLOT POINTS
#define drawCross( center, color, d )                                 \
        line( img, Point( center.x - d, center.y - d ),                \
        Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
        line( img, Point( center.x + d, center.y - d ),                \
        Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

        /// DRAW ALL ON IMAGE
        img = Scalar::all(0);
        drawCross( noisyPt, Scalar(255,255,255), 9 );     //WHITE
        drawCross( estimatedPt, Scalar(0,0,255), 6 );       //RED
        drawCross( measuredPt, Scalar(0,255,0), 3 );        //GREEN


        line( img, estimatedPt, measuredPt, Scalar(100,255,255), 3, CV_AA, 0 );
        line( img, estimatedPt, noisyPt, Scalar(0,255,255), 3, CV_AA, 0 );

        imshow( "Mouse-Kalman", img );
        key=cv::waitKey(dt);
        prevMeasurement = measurement;
    }
    }

    return 0;
}