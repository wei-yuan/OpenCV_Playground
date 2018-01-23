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
//    std::cout <<"matSize:\n" << matSize << std::endl;
/*
    //---------------------------------------// 
    // Draw the histograms     
    //---------------------------------------//          
    cv::Mat hist_plot = cv::Mat::zeros(hist.size(), hist.type());
    hist.copyTo(hist_plot);

    int hist_w = 640; int hist_h = 480;
    int bin_w = cvRound( (double) hist_w/num_bins );    

    cv::Mat histImage( hist_h, hist_w, CV_32F, cv::Scalar(0) );    
    // Normalize the result to [ 0, histImage.rows ]

    normalize(hist_plot, hist_plot, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    int thickness = 2;
    for( int i = 1; i < num_bins; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist_plot.at<int>(i-1)) ) ,
                        cv::Point( bin_w*(i), hist_h - cvRound(hist_plot.at<int>(i)) ),
                        cv::Scalar( 255, 0, 0), thickness, 8, 0  );
    }
    /// Display    
    cv::imshow("hist Demo", histImage ); 
*/
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
    
//    std::cout << "num_equal_freq: " << num_equal_freq << std::endl;
//    std::cout << "num_res_pixel: " << num_res_pixel << std::endl;

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

//    cv::imshow("output", output);
    //---------------------------------------// 
    // show new histogram
    //---------------------------------------//
/*    
    cv::Mat eqhist_plot = cv::Mat::zeros(1, num_bins, CV_32SC1); // CV_32SC1: int type
    
    for(int i=0; i < src.rows; i++)
    {    
        for(int j=0; j < src.cols; j++)        
        {            
            idx = output.at<unsigned char>(i,j);    // value of src = cnt of hist                                    
            eqhist_plot.at<int>(0, idx) += 1;       // only one row in hist
        }        
    }         

    cv::Mat eqHistImage( hist_h, hist_w, CV_32F, cv::Scalar(0) );
    // Normalize the result to [ 0, histImage.rows ]
    normalize(eqhist_plot, eqhist_plot, 0, eqHistImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    for( int i = 1; i < num_bins; i++ )
    {
        line( eqHistImage, cv::Point( bin_w*(i-1), hist_h - cvRound(eqhist_plot.at<int>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(eqhist_plot.at<int>(i)) ),
                            cv::Scalar( 255, 0, 0), thickness, 8, 0  );
    }
    // Display
    cv::imshow("eqhist Demo", eqHistImage );
*/
/*
    std::cout <<"matSize:\n" << matSize << std::endl;
    std::cout <<"histogram:\n" << hist << std::endl;
    std::cout <<"cu_hist:\n" << cu_hist << std::endl;
    std::cout <<"equalFreq:\n" << equalFreq << std::endl;
    std::cout <<"cu_equalHist:\n" << cu_equalFreq << std::endl;
    std::cout <<"mapping:\n" << mapping << std::endl;
    std::cout <<"output:\n" << output << std::endl;
*/    
}

