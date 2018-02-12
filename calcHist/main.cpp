#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/**
 * @function main
 */
int main(int argc, char **argv)
{
    //Mat src, dst;
    Mat src, dst;

    /// Load image
    // getUMat() faster than copyTo()
    src = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    if (!src.data)
    {
        return -1;
    }

    UMat usrc = src.getUMat(ACCESS_READ);

    /// Separate the image in 3 places ( B, G and R )
    vector<Mat> bgr_planes;
    split(usrc, bgr_planes);

    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float        range[]   = {0, 256};
    const float *histRange = {range};

    bool uniform    = true;
    bool accumulate = false;

    UMat ub_hist, ug_hist, ur_hist;

    /// Compute the histograms:
    calcHist(&bgr_planes[0], 1, 0, Mat(), ub_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), ug_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), ur_hist, 1, &histSize, &histRange, uniform, accumulate);

    // print result
    cout << "ub_hist: " << endl;
    cout << ub_hist << endl;
    cout << "ug_hist: " << endl;
    cout << ug_hist << endl;    

    // Draw the histograms for B, G and R
    int hist_w = 512;
    int hist_h = 400;
    int bin_w  = cvRound((double)hist_w / histSize);

    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    Mat b_hist, g_hist, r_hist;
    b_hist = ub_hist.getMat(ACCESS_READ);
    g_hist = ug_hist.getMat(ACCESS_READ);
    r_hist = ur_hist.getMat(ACCESS_READ);

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());        

    /// Draw for each channel
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
             Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
             Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
             Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255), 2, 8, 0);
    }

    /// Display
    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
    imshow("calcHist Demo", histImage);

    waitKey(0);

    return 0;
}