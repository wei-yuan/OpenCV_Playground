#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <utility>
#include <vector>

using namespace std;

std::pair<cv::Point, cv::Point> polar_to_card(cv::Vec2f line)
{
    float rho   = line[0];
    float theta = line[1];

    double x0 = cos(theta) * rho;
    double y0 = sin(theta) * rho;

    cv::Point pt1, pt2;
    pt1.x = cvRound(x0 + 1000 * (-sin(theta)));
    pt1.y = cvRound(y0 + 1000 * (cos(theta)));
    pt2.x = cvRound(x0 - 1000 * (-sin(theta)));
    pt2.y = cvRound(y0 - 1000 * (cos(theta)));

    return {pt1, pt2};
}

int main()
{
    // cap is the object of class video capture that tries to capture video file
    cv::VideoCapture cap("/home/alex504/img_video_file/kalman/road_view.mp4");
    // VideoCapture cap("/home/alex/img_video_file/road_view.mp4");

    if (!cap.isOpened()) {
        cout << "\nCannot open video camera";
    } else {
        // CAPTURING FRAMES FROM CAMERA
        while (true) {
            cv::UMat src, croppedImg, dst, cdst;
            cap >> src;

            // crop input image
            // cv::Rect roi(xMin,yMin,xMax-xMin,yMax-yMin); 
            cv::Rect myROI(500, 300, 100*4, 100*3);            
            cv::UMat croppedRef(src, myROI);
            croppedRef.copyTo(croppedImg);
            
            cv::Canny(croppedImg, dst, 50, 200, 3);

            vector<cv::Vec2f> lines;
            // detect lines
            cv::HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);

            // prepare the color canvas for output image
            cv::cvtColor(dst, cdst, CV_GRAY2BGR);
            // draw lines
            for (auto& line : lines) {                                
                auto l = polar_to_card(line);
                cv::line(cdst, l.first, l.second, cv::Scalar(0, 0, 255), 3, CV_AA);
                break;
            }

            cv::imshow("source", src);
            cv::imshow("detected lines", cdst);

            if (cv::waitKey(10) == 32) break;
        }
    }

    return 0;
}