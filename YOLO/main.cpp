#include <cstdio>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace cv::dnn;

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
using namespace std;

const size_t network_width = 416;
const size_t network_height = 416;

const char* about = "This sample uses You only look once (YOLO)-Detector "
                    "(https://arxiv.org/abs/1612.08242) "
                    "to detect objects on camera/video/image.\n"
                    "Models can be downloaded here: "
                    "https://pjreddie.com/darknet/yolo/\n"
                    "Default network is 416x416.\n"
                    "Class names can be downloaded here: "
                    "https://github.com/pjreddie/darknet/tree/master/data\n";

const char* params
    = "{ help           | false | print usage         }"
      "{ cfg            |       | model configuration }"
      "{ model          |       | model weights       }"
      "{ camera_device  | 0     | camera device number}"
      "{ video          |       | video or image for detection}"
      "{ min_confidence | 0.24  | min confidence      }"
      "{ class_names    |       | class names         }";

int main(int argc, char** argv)
{
    
    CommandLineParser parser(argc, argv, params);

    if (parser.get<bool>("help"))
    {
        cout << about << endl;
        parser.printMessage();
        return 0;
    }

    String modelConfiguration = parser.get<String>("cfg");
    String modelBinary = parser.get<String>("model");

    //! [Initialize network]
    dnn::Net net = readNetFromDarknet(modelConfiguration, modelBinary);
    //! [Initialize network]

    if (net.empty())
    {
        cerr << "Can't load network by using the following files: " << endl;
        cerr << "cfg-file:     " << modelConfiguration << endl;
        cerr << "weights-file: " << modelBinary << endl;
        cerr << "Models can be downloaded here:" << endl;
        cerr << "https://pjreddie.com/darknet/yolo/" << endl;
        exit(-1);
    }
/*
    // cap is the object of class video capture that tries to capture video file
    cv::VideoCapture cap("/home/alex504/img_video_file/car.mp4");
    // cv::VideoCapture cap("/home/alex/img_video_file/road_view.mp4");

    if (!cap.isOpened()) // isOpened() returns true if capturing has been initialized.
    {
        cout << "Cannot open the video file. \n";
        return -1;
    }
*/
    VideoCapture cap;    
    if (parser.get<String>("video").empty())
    {
        int cameraDevice = parser.get<int>("camera_device");
        cap = VideoCapture(cameraDevice);
        if(!cap.isOpened())
        {
            cout << "Couldn't find camera: " << cameraDevice << endl;
            return -1;
        }
    }
    else
    {
        cap.open(parser.get<String>("video"));
        if(!cap.isOpened())
        {
            cout << "Couldn't open image or video: " << parser.get<String>("video") << endl;
            return -1;
        }
    }

    Size videoSize = Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH),(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));    
    VideoWriter writer;
    writer.open("YOLO_processed.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);

    vector<string> classNamesVec;
    ifstream classNamesFile(parser.get<String>("class_names").c_str());
    if (classNamesFile.is_open())
    {
        string className = "";
        while (classNamesFile >> className)
            classNamesVec.push_back(className);
    }

    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera/video or read image

        if (frame.empty())
        {
            waitKey();
            break;
        }

        if (frame.channels() == 4)
            cvtColor(frame, frame, COLOR_BGRA2BGR);

        //! [Resizing without keeping aspect ratio]
        Mat resized;
        resize(frame, resized, Size(network_width, network_height));
        //! [Resizing without keeping aspect ratio]

        //! [Prepare blob]
        Mat inputBlob = blobFromImage(resized, 1 / 255.F); //Convert Mat to batch of images
        //! [Prepare blob]

        //! [Set input blob]
        net.setInput(inputBlob, "data");                   //set the network input
        //! [Set input blob]

        //! [Make forward pass]
        Mat detectionMat = net.forward("detection_out");   //compute output
       //! [Make forward pass]

       vector<double> layersTimings;
       double freq = getTickFrequency() / 1000;
       double time = net.getPerfProfile(layersTimings) / freq;
       ostringstream ss;
       ss << "FPS: " << 1000/time << " ; time: " << time << " ms";
       putText(frame, ss.str(), Point(20,20), 0, 0.5, Scalar(0,0,255));

        float confidenceThreshold = parser.get<float>("min_confidence");
        for (int i = 0; i < detectionMat.rows; i++)
        {
            const int probability_index = 5;
            const int probability_size = detectionMat.cols - probability_index;
            float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);

            size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
            float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);

            if (confidence > confidenceThreshold)
            {
                float x = detectionMat.at<float>(i, 0);
                float y = detectionMat.at<float>(i, 1);
                float width = detectionMat.at<float>(i, 2);
                float height = detectionMat.at<float>(i, 3);
                int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
                int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
                int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
                int yRightTop = static_cast<int>((y + height / 2) * frame.rows);

                Rect object(xLeftBottom, yLeftBottom,
                            xRightTop - xLeftBottom,
                            yRightTop - yLeftBottom);

                rectangle(frame, object, Scalar(0, 255, 0));

                if (objectClass < classNamesVec.size())
                {
                    ss.str("");
                    ss << confidence;
                    String conf(ss.str());
                    String label = String(classNamesVec[objectClass]) + ": " + conf;
                    int baseLine = 0;
                    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
                                          Size(labelSize.width, labelSize.height + baseLine)),
                              Scalar(255, 255, 255), CV_FILLED);
                    putText(frame, label, Point(xLeftBottom, yLeftBottom),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
                }
                else
                {
                    cout << "Class: " << objectClass << endl;
                    cout << "Confidence: " << confidence << endl;
                    cout << " " << xLeftBottom
                         << " " << yLeftBottom
                         << " " << xRightTop
                         << " " << yRightTop << endl;
                }
            }
        }

        writer.write(frame);
        imshow("detections", frame);
        if (waitKey(1) >= 0) break;
    }
    
    return 0;
} // main