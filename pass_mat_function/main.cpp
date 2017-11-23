#include <iostream>
#include <fstream>
#include <string>
//#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>

cv::UMat& opencl_mat_mul(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
{

    std::cout << "input_array1: \n" << input_array1 << std::endl;    
    std::cout << "input_array2: \n" << input_array2 << std::endl;
    std::cout << "output_array: \n" << output_array << std::endl;    

    // ---------------------------------------------------
    // OpenCL Initialization
    // ---------------------------------------------------
    // check OpenCL availability
    if (!cv::ocl::haveOpenCL())
    {
        std::cout << "OpenCL is not avaiable..." << std::endl;
        //return -1;
    }

    // create context
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        std::cout << "Failed creating the context..." << std::endl;
        //return -1;
    }

    // device detection
    std::cout << context.ndevices() << " GPU devices are detected." << std::endl;
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        std::cout << "name                 : " << device.name() << std::endl;
        std::cout << "available            : " << device.available() << std::endl;
        std::cout << "imageSupport         : " << device.imageSupport() << std::endl;
        std::cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << std::endl;
    }

    // Select the first device
    cv::ocl::Device(context.device(0));
/*
    // Transfer Mat data to the device
    cv::UMat uinput_array1(output_array.size(), output_array.type(), cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat uinput_array2(output_array.size(), output_array.type(), cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat uoutput_array(output_array.size(), output_array.type(), cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    uinput_array1 = input_array1;
    uinput_array2 = input_array2;
    uoutput_array = output_array;
*/    
    // ---------------------------------------------------
    // Kernel Initialization
    // ---------------------------------------------------
    // Read the OpenCL kernel code
    std::ifstream ifs("mat_mul.cl");
    if (ifs.fail()) 
        std::cout << "Failed to read opencl file" << std::endl; 
    std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    cv::ocl::ProgramSource programSource(kernelSource);

    // Compile the kernel code
    cv::String errmsg;
    // By setting program argument "-D xxx=yyy ", we can replace xxx with yyy in the kernel
    cv::String buildopt =""; 
    cv::ocl::Program program = context.getProg(programSource, buildopt, errmsg);    

    // ---------------------------------------------------
    // Create Kernel and run
    // ---------------------------------------------------    
    // create kernel
    cv::ocl::Kernel kernel_statePre("mat_mul", program);
    // kernel argument
    int Mat_type = CV_32F;    
    // use input UMat data directly
    kernel_statePre.args(Mat_type,  
                        cv::ocl::KernelArg::ReadOnlyNoSize(input_array1),
                        cv::ocl::KernelArg::ReadOnlyNoSize(input_array2),
                        cv::ocl::KernelArg::ReadWrite(output_array));                        
    // thread size
    size_t globalThreads[3] = { (size_t)input_array1.cols, (size_t)input_array1.rows, 1 };
    //size_t localThreads[3] = { 16, 16, 1 };

    bool success = kernel_statePre.run(3, globalThreads, NULL, true);
    if (!success){
        std::cout << "Failed running the kernel..." << std::endl;
        //return -1;
    }
   
    // ---------------------------------------------------
    // Copy data from device(GPU) to host(CPU)
    // ---------------------------------------------------     
    // Download the dst data from the device
    //output_array = uoutput_array;

    return output_array;
}

cv::UMat& opencl_mat_add(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
{
    std::cout << "input_array1: \n" << input_array1 << std::endl;    
    std::cout << "input_array2: \n" << input_array2 << std::endl;
    std::cout << "output_array: \n" << output_array << std::endl;    

    // ---------------------------------------------------
    // OpenCL Initialization
    // ---------------------------------------------------
    // check OpenCL availability
    if (!cv::ocl::haveOpenCL())
    {
        std::cout << "OpenCL is not avaiable..." << std::endl;
        //return -1;
    }

    // create context
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        std::cout << "Failed creating the context..." << std::endl;
        //return -1;
    }

    // device detection
    std::cout << context.ndevices() << " GPU devices are detected." << std::endl;
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        std::cout << "name                 : " << device.name() << std::endl;
        std::cout << "available            : " << device.available() << std::endl;
        std::cout << "imageSupport         : " << device.imageSupport() << std::endl;
        std::cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << std::endl;
    }

    // Select the first device
    cv::ocl::Device(context.device(0));

    // ---------------------------------------------------
    // Kernel Initialization
    // ---------------------------------------------------
    // Read the OpenCL kernel code
    std::ifstream ifs("mat_add.cl");
    if (ifs.fail()) 
        std::cout << "Failed to read opencl file" << std::endl; 
    std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    cv::ocl::ProgramSource programSource(kernelSource);

    // Compile the kernel code
    cv::String errmsg;
    // By setting program argument "-D xxx=yyy ", we can replace xxx with yyy in the kernel
    cv::String buildopt =""; 
    cv::ocl::Program program = context.getProg(programSource, buildopt, errmsg);    

    // ---------------------------------------------------
    // Create Kernel and run
    // ---------------------------------------------------    
    // create kernel
    cv::ocl::Kernel kernel_statePre("mat_add", program);
    // kernel argument
    int Mat_type = CV_32F;    
    // use input UMat data directly
    kernel_statePre.args(Mat_type,  
                        cv::ocl::KernelArg::ReadOnlyNoSize(input_array1),
                        cv::ocl::KernelArg::ReadOnlyNoSize(input_array2),
                        cv::ocl::KernelArg::ReadWrite(output_array));                        
    // thread size
    size_t globalThreads[3] = { (size_t)input_array1.cols, (size_t)input_array1.rows, 1 };
    //size_t localThreads[3] = { 16, 16, 1 };

    bool success = kernel_statePre.run(3, globalThreads, NULL, true);
    if (!success){
        std::cout << "Failed running the kernel..." << std::endl;
        //return -1;
    }
   
    // ---------------------------------------------------
    // Copy data from device(GPU) to host(CPU)
    // ---------------------------------------------------     
    // Download the dst data from the device

    return output_array;
}

cv::UMat& kalman_gain(cv::KalmanFilter& KF, cv::UMat& output_array)
{
    return output_array;
}


int main()
{
    /////////////////////////////////////////////////////////
    // Update Stage
    /////////////////////////////////////////////////////////
    // Kalman gain
    /////////////////////////////////////////////////////////
    
    /*******************************************************/
    // temp2 = H * Pk-
    /*******************************************************/    
    // ****** Data Initialization ********* //
    // Kalman filter here, DP = 16, MP = 8, CP = 0
    cv::KalmanFilter KF(16, 8, 0);
    // KF.transitionMatrix needs to be CV_32F or CV_64F
    std::cout << "****** input mat: \n" << std::endl;    
    // UMat: Transfer Mat data to the device
    cv::UMat H = cv::UMat::eye(KF.measurementMatrix.size(), CV_32F);     // 8 * 16
    cv::UMat Pkmo = cv::UMat::eye(16, 16, CV_32F);                       // 16 * 16
    cv::UMat HP = cv::UMat::eye(KF.measurementMatrix.size(), CV_32F);    // 8 * 16
    cv::UMat HP_dst = cv::UMat::zeros(KF.measurementMatrix.size(), CV_32F);// 8 * 16

    // matrix multiplication using opencl
    HP_dst = opencl_mat_mul(H, Pkmo, HP);

    std::cout << "\n****** temp2 = H * Pk-: "  << std::endl;
    std::cout << "\nHP_dst: \n" << HP_dst << std::endl;  
    
    /*******************************************************/
    // temp3 = temp2 * Ht + R
    /*******************************************************/    
    // ****** Data Initialization ********* //
   
    std::cout << "****** input mat: \n" << std::endl;
    cv::UMat Ht = H.t();  // 16 * 8 
    cv::UMat HPH = cv::UMat::eye(8, 8, CV_32F);
    cv::UMat HPH_dst = cv::UMat::eye(8, 8, CV_32F);

    cv::UMat R = cv::UMat::ones(8, 8, CV_32F);
/*  delete annotation mark if all process is ready  
    std::cout << "\nR: \n" << R << std::endl;    
    cv::randu(R, cv::Scalar::all((double)-500), cv::Scalar::all((double)500)); // dst , input1, input2, watch out for chnnel number
    std::cout << "\nR: \n" << R << std::endl;   
*/    
    cv::UMat HPHR = cv::UMat::zeros(8, 8, CV_32F); 

    // matrix multiplication using opencl
    HPH_dst = opencl_mat_mul(HP_dst, Ht, HPH);
    
    std::cout << "\n****** temp2 * Ht: "  << std::endl;
    std::cout << "\nHPH_dst: \n" << HPH_dst << std::endl;      
        
    // matrix add using opencl
    HPHR = opencl_mat_add(HPH_dst, R, HPH);    
    std::cout << "\n****** temp3 = temp2 * Ht + R: "  << std::endl;
    std::cout << "\nHPHR: \n" << HPHR << std::endl;      

    cv::UMat A = cv::UMat::ones(8, 8, CV_32F);
    cv::UMat B = cv::UMat::ones(8, 8, CV_32F);
    cv::UMat C = cv::UMat::ones(8, 8, CV_32F);
    cv::UMat C_dst = cv::UMat::zeros(8, 8, CV_32F);
    C_dst = opencl_mat_add(A, B, C);    
    std::cout << "\nC_dst: \n" << C_dst << std::endl;      

    /*******************************************************/
    // inv(temp3) * temp2
    /*******************************************************/        

    return 0;
}