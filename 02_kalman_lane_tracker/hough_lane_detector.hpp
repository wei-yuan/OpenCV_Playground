#ifndef HOUGH_LANE_DETECTOR_H
#define HOUGH_LANE_DETECTOR_H

// cpp header
#include <algorithm>
#include <exception>
#include <iostream>
#include <math.h> /* atan2 */
#include <tuple>
#include <utility>
#include <vector>
#include <map>
// OpenCV header
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef FIL_MAX
#define FLT_MAX 3.402823466e+38F /* max value */
#define FLT_MIN 1.175494351e-38F /* min positive value */
#endif

namespace my
{
class Line
{
public:
    Line() {}
    Line(cv::Point beg, cv::Point end)
    {
        this->beg = beg;
        this->end = end;
    }

    void swap()
    {
        cv::Point tmp = beg;
        beg           = end;
        end           = tmp;
    }

    inline bool operator==(const Line& rhs)
    {
        return (((this->beg.x == rhs.beg.x) || (this->beg.y == rhs.beg.y))
                || ((this->end.x == rhs.end.x) || (this->end.y == rhs.end.y)) == true);
    }
    inline bool operator!=(const Line& rhs)
    {
        return (((this->beg.x != rhs.beg.x) || (this->beg.y != rhs.beg.y))
                || ((this->end.x != rhs.end.x) || (this->end.y != rhs.end.y)) == true);
    }

    cv::Point beg = {};
    cv::Point end = {};
};
}

class HoughLaneDetector
{
public:
    // constrctor & destructor
    HoughLaneDetector(float intput_road_horizon)
    {
        prob_hough   = true;
        roi_theta    = 0.3;
        road_horizon = intput_road_horizon;
        vote         = 150;
    }

    // std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> detect(cv::UMat& frame)
    std::pair<my::Line, my::Line> detect(cv::UMat& frame)
    {
        cv::UMat gray, dst, cdst, blur, contour;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // w = frame.shape[0], h = frame.shape[1]
        int roiy_end = frame.rows;
        int roix_end = frame.cols;

        // blurr
        int ksize = 5;
        cv::medianBlur(gray, blur, ksize);
        // edge detection
        cv::Canny(blur, contour, 60, 120);
        imshow("canny in detect", contour);

        std::vector<my::Line> lines;
        /* switch HoughlineP() or Houghline() here */
        if (this->prob_hough == true) {
            lines = this->houghp(contour, this->vote);
        } else {
            lines = this->standard_hough(contour, this->vote);
        }

        imshow("dst in detect", contour);
        // prepare the color canvas for output image
        cv::cvtColor(contour, cdst, CV_GRAY2BGR);
        // draw line
        for (auto& l : lines) {
            cv::line(cdst, l.beg, l.end, cvScalar(0, 0, 255), 3, cv::LINE_AA);
        }
        // generate line with std::pair<cv::Point, cv::Point> format
        cv::imshow("line using HoughLineP", cdst);
        // cv::waitKey();left

        my::Line left_bound, right_bound;
        float    left_bound_dist = FLT_MIN, right_bound_dist = FLT_MAX;
        for (auto& l : lines) {
            float theta = std::abs(atan2((l.end.y - l.beg.y), (l.end.x - l.beg.x)));
            // Skip lines with small angle WRT horizon
            if (theta <= this->roi_theta) continue;
            // Calculate the distance WRT center of the input image
            float dist = base_distance(l, frame.cols);
            if (left_bound_dist < dist && dist < 0) {
                left_bound_dist = dist;
                left_bound      = l;
            } else if (dist > 0 && right_bound_dist > dist) {
                right_bound_dist = dist;
                right_bound      = l;
            }
        }

        // If the region is found
        if (left_bound != right_bound) {
            left_bound  = this->scale_line(left_bound, frame.rows);
            right_bound = this->scale_line(right_bound, frame.rows);
        }

        return std::make_pair(left_bound, right_bound);
    }

protected:
    bool  prob_hough   = true;
    float roi_theta    = 0.3;
    int   road_horizon = 180; // degree, ex 180
    int   vote         = 150;

private:
    std::vector<cv::Point> cv_vec2f_to_point(std::vector<cv::Vec2f> input)
    {
        std::vector<cv::Point> output;
        for (auto& e : input) {
            cv::Point p(e[0], e[1]);
            output.push_back(p);
        }
        return output;
    }

    std::vector<my::Line> cv_vec4f_to_line(std::vector<cv::Vec4f> input)
    {
        std::vector<my::Line> output;
        for (auto& e : input) {
            cv::Point2f beg = {e[0], e[1]};
            cv::Point2f end = {e[2], e[3]};
            output.push_back({beg, end});
        }
        return output;
    }

    // Hough Trnasform wrapper to return a series of point
    std::vector<my::Line> standard_hough(cv::UMat& frame, float initial_vote)
    {
        // input argument of HoughLines(dst, lines, rho, theta, threshold, srn ,stn)
        // -dst: Output of the edge detector
        // -lines: A vector that will store the parameters (rho, theta) of the detected lines
        // -rho : The resolution of the parameter \r in pixels. we can use 1
        // -theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
        // -threshold: The minimum number of intersections to “detect” a line
        // -srn and -stn: Default parameters to zero. Check OpenCV reference for more info.
        float init_vote = initial_vote; // 150, but what's this for?

        std::vector<cv::Vec2f> raw_lines;
        cv::HoughLines(frame, raw_lines, 1, CV_PI / 180, init_vote, 0, 0);
        // why this line is here...
        auto lines = cv_vec2f_to_point(raw_lines);

        std::vector<my::Line> cvt_lines;
        for (auto& line : lines) {
            float rho   = line.x;
            float theta = line.y;

            double x0 = cos(theta) * rho;
            double y0 = sin(theta) * rho;

            my::Line new_line;
            new_line.beg.x = cvRound(x0 + 1000 * (-sin(theta)));
            new_line.beg.y = cvRound(y0 + 1000 * (cos(theta)));
            new_line.end.x = cvRound(x0 - 1000 * (-sin(theta)));
            new_line.end.y = cvRound(y0 - 1000 * (cos(theta)));
            cvt_lines.push_back(new_line);
        }

        return cvt_lines;
    }

    // Hough Trnasform wrapper to return a series of point
    std::vector<my::Line> houghp(cv::UMat& frame, float initial_vote)
    {
        std::vector<cv::Vec4f> plines;
        // type of lines cv::Vec4fneed to be std::vector<cv::Vec4f>, i.e. vector with 4 float element
        cv::HoughLinesP(frame, plines, 1, CV_PI / 180, initial_vote, 0, 0);
        return cv_vec4f_to_line(plines);
    }

    float base_distance(my::Line line, float width)
    {
        if (line.beg.x == line.end.x) {
            return (width * 0.5) - line.beg.x;
        }

        float m = (line.end.y - line.beg.y) / (line.end.x - line.beg.x);
        float c = line.beg.y - m * line.beg.x;

        float base_cross = -c / m;
        return (width * 0.5) - base_cross;
    }

    // scale the farthest point of the segment to be on the drawing horizon
    my::Line scale_line(my::Line line, int frame_height)
    {
        if (line.beg.x == line.end.x) {
            if (line.beg.y < line.end.y) {
                line.beg.y = this->road_horizon;
                line.end.y = frame_height;
            } else {
                line.beg.y = frame_height;
                line.end.y = this->road_horizon;
            }
            return line;
        }

        if (line.beg.y >= line.end.y) {
            line.swap();
        }
        float m = (line.end.y - line.beg.y) / (line.end.x - line.beg.x);
        line.end.x += (this->road_horizon - line.end.y) / m;
        line.end.y = this->road_horizon;
        line.beg.x += (frame_height - line.beg.y) / m;
        line.beg.y = frame_height;

        return line;
    }
};
#endif