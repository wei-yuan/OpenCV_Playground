//--------------------------------------------------------------
/*
Author: Wei-Yuan Alex Hsu
Date:   2018/1/19 Fri
Target: implement non-parallel and parallel( OpenCL based ) histogram equalization
*/
//--------------------------------------------------------------
#include <math.h>
#include <stdio.h>
#include <stdlib.h> /* abs */
#include <limits.h> /* MAX */
// opencv library
#include "opencv2/opencv.hpp"

void cumulative_Mat_Value(cv::Mat& src, cv::Mat& dst)
{
    int cuSum = 0;
    for(int i=0; i < src.rows; i++)
    {
        for(int j=0; j < src.cols; j++)
        {
            cuSum += src.at<int>(i,j);
            dst.at<int>(i,j) = cuSum;  // assign cumulative value
        }
    }
}

//void equalOfHist( cv::Mat& src, cv::Mat& dst )
void equalOfHist( cv::Mat& src) // src type: CV_8U
{        
    int matSize = src.rows * src.cols;
    // std::cout <<"matSize:\n" << matSize << std::endl;
    //---------------------------------------// 
    // Calculate histogram --> count pixel number is faster in CPU
    //---------------------------------------//     
    int num_bins = 256, idx = 0;
    cv::Mat hist = cv::Mat::zeros(1, num_bins, CV_32SC1); // CV_32SC1: int type

    for(int i=0; i < src.rows; i++)
    {    
        for(int j=0; j < src.cols; j++)        
        {            
            idx = src.at<unsigned char>(i,j);       // value of src = cnt of hist                                    
            hist.at<int>(0, idx) += 1;    // only one row in hist
        }        
    }    
    std::cout <<"matSize:\n" << matSize << std::endl;

    // create the Mat objects to save our histograms
    cv::Mat hist_img;        

    // Draw the histograms     
    int hist_w = 640; int hist_h = 480;
    int bin_w = cvRound( (double) hist_w/num_bins );

    cv::Mat histImage( hist_h, hist_w, CV_32F, cv::Scalar(0) );    
    // Normalize the result to [ 0, histImage.rows ]
    normalize(hist_img, hist_img, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    int thickness = 3;
    for( int i = 1; i < num_bins; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<unsigned char>(i-1)) ) ,
                        cv::Point( bin_w*(i), hist_h - cvRound(hist.at<unsigned char>(i)) ),
                        cv::Scalar( 255, 0, 0), 3, 8, 0  );
    }
    /// Display    
    cv::imshow("hist Demo", histImage ); 

    //---------------------------------------// 
    // Calculate cumulative histogram
    //---------------------------------------// 
    cv::Mat cu_hist = cv::Mat::zeros(hist.size(), CV_32SC1);
    
    cumulative_Mat_Value(hist, cu_hist);

    //---------------------------------------// 
    // new pixel frequency
    //---------------------------------------//         
    
    int num_equal_freq = floor(matSize / num_bins);
    int num_res_pixel = matSize - num_equal_freq * num_bins;
    
    std::cout << "num_equal_freq: " << num_equal_freq << std::endl;
    std::cout << "num_res_pixel: " << num_res_pixel << std::endl;

    cv::Mat equalFreq = cv::Mat::zeros(hist.size(), CV_32SC1);

    for(int i=0; i < hist.rows; i++)
    {
        //std::cout << "i: " << i << std::endl;
        for(int j=0; j < hist.cols; j++)        
        {   
            //std::cout << "j: " << j << std::endl;
            if( j < hist.cols -1)
            {
                equalFreq.at<int>(i,j) = num_equal_freq;  // assign cumulative value                
            }                                     
            else
            {
                equalFreq.at<int>(i,j) = num_equal_freq + num_res_pixel;
            }                
        }
    }

    //---------------------------------------// 
    // cumulative of new pixel frequency
    //---------------------------------------// 
    cv::Mat cu_equalFreq = cv::Mat::zeros(hist.size(), CV_32SC1);

    cumulative_Mat_Value(equalFreq, cu_equalFreq);

    //---------------------------------------// 
    // calculate mapping table
    //---------------------------------------//    
    int TEMP_DIFF = 0;    
    int DIFF = std::numeric_limits<int>::max();

    // mapping: grey level mapping value
    cv::Mat mapping = cv::Mat::zeros(hist.size(), CV_32SC1);
    for(int i=0; i < hist.cols; i++)
    {        
        for(int j=0; j < hist.cols; j++)        
        {   
            if(cu_hist.at<int>(0, i) == 0) continue;
            
            TEMP_DIFF = abs( cu_hist.at<int>(0, i) - cu_equalFreq.at<int>(0, j) ); 
            if(TEMP_DIFF <= DIFF)
            {
                DIFF = TEMP_DIFF;
                mapping.at<int>(0, i) = j;
            }
        }
        DIFF = std::numeric_limits<int>::max(); // reset
    }

    //---------------------------------------// 
    // mapping image to new grey scale value
    //---------------------------------------//
    cv::Mat output = cv::Mat::zeros(src.size(), src.type());  
    for(int i=0; i < src.rows; i++)
    {        
        for(int j=0; j < src.cols; j++)        
        {   
            int index = src.at<unsigned char>(i, j);
            //assign new pixel value  
            output.at<unsigned char>(i, j) =  mapping.at<int>(0, index);
        }        
    }

    //---------------------------------------// 
    // show new histogram
    //---------------------------------------//    
    cv::Mat map_hist_img = cv::Mat::zeros(1, num_bins, CV_32SC1); // CV_32SC1: int type
    for(int i=0; i < src.rows; i++)
    {    
        for(int j=0; j < src.cols; j++)        
        {            
            idx = output.at<unsigned char>(i,j); // value of src = cnt of hist                                    
            map_hist_img.at<int>(0, idx) += 1;        // only one row in hist
        }        
    } 
    
    cv::Mat eqHistImage( hist_h, hist_w, CV_32F, cv::Scalar(0) );
    // Normalize the result to [ 0, histImage.rows ]
    normalize(map_hist_img, map_hist_img, 0, eqHistImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    for( int i = 1; i < num_bins; i++ )
    {
        line( eqHistImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                            cv::Scalar( 255, 0, 0), thickness, 8, 0  );
    }
    // Display
    cv::imshow("eqhist Demo", eqHistImage );
/*
    std::cout <<"matSize:\n" << matSize << std::endl;
    std::cout <<"histogram:\n" << hist << std::endl;
    std::cout <<"cu_hist:\n" << cu_hist << std::endl;
    std::cout <<"equalFreq:\n" << equalFreq << std::endl;
    std::cout <<"cu_equalHist:\n" << cu_equalFreq << std::endl;
    std::cout <<"mapping:\n" << mapping << std::endl;
    std::cout <<"output:\n" << output << std::endl;
*/
    cv::imshow("output", output);
    
}

