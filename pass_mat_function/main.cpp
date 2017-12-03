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
   
    return output_array;
}

cv::UMat& opencl_mat_subtract(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
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
    std::ifstream ifs("mat_subtract.cl");
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
    cv::ocl::Kernel kernel_statePre("mat_subtract", program);
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
   
    return output_array;
}

cv::UMat& kalman_gain(cv::KalmanFilter& KF, cv::UMat& output_array)
{   
    /*******************************************************/
    // temp2 = H * Pk-
    /*******************************************************/    
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

    /*******************************************************/
    // temp4 = inv(temp3) * temp2
    /*******************************************************/
    std::cout << "\n****** temp4 = inv(temp3) * temp2: "  << std::endl;
    cv::UMat kt = cv::UMat::zeros(8, 16, CV_32F);
    cv::UMat kt_dst = cv::UMat::zeros(8, 16, CV_32F);
    cv::UMat inv = HPHR.inv();
    kt_dst = opencl_mat_mul(inv, HP_dst, kt_dst);
    std::cout << "\nkt_dst: \n" << kt_dst << std::endl;       

    /*******************************************************/
    // Kk = transpose of temp4 
    /*******************************************************/
    output_array = kt_dst.t();    
    std::cout << "\n****** Kalman gain = temp2 * Ht + R: "  << std::endl;
    std::cout << "\nKk: \n" << output_array << std::endl;

    KF.measurementMatrix = H.getMat(CV_32F);
    std::cout << "\nKF.measurementMatrix: \n" << KF.measurementMatrix << std::endl;
    
    return output_array;
}

cv::Mat& kalman_predict(cv::KalmanFilter& KF, const cv::Mat& control)
{
    //
    return ;
}

cv::Mat& kalman_correct(cv::KalmanFilter& KF, const cv::Mat& measurement)
{
    /*******************************************************/
    // Calculate Kalman gain
    /*******************************************************/      
    cv::UMat kgain, kgain_dst;

    kgain_dst = kalman_gain(KF, kgain);
    /*******************************************************/
    // Calculate statePost Xk hat
    /*******************************************************/    
    // tmp1 = H * X(k)-
    cv::UMat H = KF.measurementMatrix.getUMat(CV_32F);
    cv::UMat Xkm = KF.statePre.getUMat(CV_32F);
    cv::UMat HXkm, HXkm_dst;
    HXkm_dst = opencl_mat_mul(H, Xkm, HXkm);

    // tmp2 = zk - tmp1
    cv::UMat zk = measurement.getUMat(CV_32F);
    cv::UMat zH, zH_dst;
    zH_dst = opencl_mat_subtract(zk, HXkm_dst, zH);

    // tmp3 = Kk * tmp2
    cv::UMat kz, kz_dst;
    kz_dst = opencl_mat_mul(kgain_dst, zH_dst, kz);

    // statePost = X(k)- + tmp3
    cv::UMat statePost;
    KF.statePost = opencl_mat_add(Xkm, kz_dst, statePost).getMat(CV_32F);

    /*******************************************************/
    // Calculate errorCovPost
    /*******************************************************/    
    // tmp1 = Kk * H
    cv::UMat KH, KH_dst;
    KH_dst = opencl_mat_mul(kgain_dst, H, KH_dst);

    // tmp2 = I - tmp1
    cv::UMat IK, IK_dst;
    cv::UMat I = cv::UMat::eye(HXkm.size(), CV_32F);
    IK_dst = opencl_mat_subtract(I, H, IK);

    // errorCovPost = tmp2 * errorCovPre(from predict stage)
    cv::UMat IZ, IZ_dst;
    IZ_dst = opencl_mat_mul(IK_dst, zH_dst, IZ);
    KF.errorCovPost = IZ_dst.getMat(CV_32F);

    return KF.errorCovPost;
}

int main()
{
    /////////////////////////////////////////////////////////
    // Update Stage
    /////////////////////////////////////////////////////////
    // ****** Data Initialization ********* //
    // Kalman filter here, DP = 16, MP = 8, CP = 0
    cv::KalmanFilter KF(16, 8, 0);
    
    
    return 0;
}