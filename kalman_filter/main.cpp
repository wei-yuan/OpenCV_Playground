#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>

cv::UMat& opencl_mat_mul(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
{

    std::cout << "input_array1: \n" << input_array1 << std::endl;    
    std::cout << "input_array2: \n" << input_array2 << std::endl;
    //std::cout << "output_array: \n" << output_array << std::endl;    

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
    std::cout << "\nresult:\n" << output_array << std::endl;

    return output_array;
}

cv::UMat& opencl_mat_add(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
{
    std::cout << "input_array1: \n" << input_array1 << std::endl;    
    std::cout << "input_array2: \n" << input_array2 << std::endl;
    //std::cout << "output_array: \n" << output_array << std::endl;    

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
    
    std::cout << "\nresult:\n" << output_array << std::endl;
   
    return output_array;
}

cv::UMat& opencl_mat_sub(cv::UMat& input_array1, cv::UMat& input_array2, cv::UMat& output_array)
{
    std::cout << "input_array1: \n" << input_array1 << std::endl;    
    std::cout << "input_array2: \n" << input_array2 << std::endl;
    //std::cout << "output_array: \n" << output_array << std::endl;    

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
    std::ifstream ifs("mat_sub.cl");
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
    cv::ocl::Kernel kernel_statePre("mat_sub", program);
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
    
    std::cout << "\nresult:\n" << output_array << std::endl;
   
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

    H.copyTo(KF.measurementMatrix);
    std::cout << "\nKF.measurementMatrix: \n" << KF.measurementMatrix << std::endl;
    
    return output_array;
}

// B: controlcl;
void kalman_predict(cv::KalmanFilter& KF, const cv::UMat& control)
{
    /*******************************************************/
    // X(k) = AX(k-1) + Bu(k-1) + Q
    /*******************************************************/        
    std::cout << "****** X(k) = AX(k-1) + Bu(k-1) + Q" << std::endl;
    cv::UMat A = cv::UMat::ones(KF.transitionMatrix.size(), CV_32F);
    
    cv::UMat Xkmo = cv::UMat::ones(16, 1, CV_32F);
    cv::UMat Xk = cv::UMat::zeros(16, 1, CV_32F);
    cv::UMat Xk_dst(Xk.size(), CV_32F, cv::ACCESS_READ );
    Xk.copyTo(Xk_dst);
    
    Xk_dst = opencl_mat_mul(A, Xkmo, Xk);
    
    //KF.statePre = Xk_dst.getMat(cv::ACCESS_READ);
    Xk_dst.copyTo(KF.statePre);
    std::cout << "\nKF.statePre: \n" << KF.statePre << std::endl;
    
    // TODO: computation of Bu(k-1) + Q, if necessary

    /*******************************************************/
    // errorCovPre P(k) = AP(k-1)At + Q
    /*******************************************************/
    std::cout << "\n****** errorCovPre P(k) = AP(k-1)At + Q: \n" << std::endl;
    // errorCovPost = P(k-1)    
    cv::UMat Pkmo = cv::UMat::ones(KF.errorCovPost.size(), CV_32F);
    // temp1 = AP(k-1)
    cv::UMat AP(Pkmo.size(), CV_32F), AP_dst(Pkmo.size(), CV_32F);
    AP_dst = opencl_mat_mul(A, Pkmo, AP);
    std::cout << "\nAP_dst: \n" << AP_dst << std::endl;

    // temp2 = temp1 * transpose(A)    
    std::cout << "\n****** AP * transpose(A): \n"<< std::endl;
    cv::UMat At = A.t();
    cv::UMat At_dst(At.size(), CV_32F, cv::ACCESS_READ);
    At_dst = opencl_mat_mul(AP_dst, At, Xk);
    At_dst.copyTo(KF.errorCovPre);
    std::cout << "\nKF.errorCovPre: \n" << KF.errorCovPre << std::endl;

    // handle the case when there will be measurement before the next predict.    
    KF.statePre.copyTo(KF.statePost);
    KF.errorCovPre.copyTo(KF.errorCovPost);
    
    std::cout << "\nKF.statePre: \n" << KF.statePre << std::endl;
    std::cout << "\nKF.errorCovPre: \n" << KF.errorCovPre << std::endl;
}

// zk: measurement
void kalman_correct(cv::KalmanFilter& KF, const cv::UMat& measurement)
{
    /*******************************************************/
    // Calculate Kalman gain
    /*******************************************************/      
    cv::UMat kgain, kgain_dst;

    kgain_dst = kalman_gain(KF, kgain);
    /*******************************************************/
    // Calculate statePost Xk hat
    /*******************************************************/    
    std::cout << "\n****** statePost = statePre + K * (zk - H * statePre) \n"<< std::endl;
    
    // tmp1 = H * X(k)-, cannot use uninitialization UMat    
    std::cout << "\n****** tmp1 = H * X(k)- \n"<< std::endl;
    cv::UMat H(KF.measurementMatrix.size(), CV_32F);    
    cv::UMat Xkm(KF.statePre.size(), CV_32F);        
    cv::UMat HXkm(H.rows, Xkm.cols, CV_32F);
    cv::UMat HXkm_dst(HXkm.size(), CV_32F); 

    HXkm_dst = opencl_mat_mul(H, Xkm, HXkm);
    std::cout << "\nHXkm_dst: \n" << HXkm_dst << std::endl;

    // tmp2 = zk - tmp1
    std::cout << "\n****** tmp2 = zk - tmp1 \n"<< std::endl;
    cv::UMat zk = measurement;
    cv::UMat zH(HXkm_dst.size(), CV_32F);
    cv::UMat zH_dst(zH.size(), CV_32F);
//    zH_dst = opencl_mat_subtract(zk, HXkm_dst, zH);
    std::cout << "\nzH_dst:\n" << zH_dst << std::endl;

    // tmp3 = Kk * tmp2
    std::cout << "\n****** tmp3 = Kk * tmp2 \n"<< std::endl;
    cv::UMat kz(kgain_dst.rows, zH_dst.cols, CV_32F);
    cv::UMat kz_dst(kz.size(), CV_32F);
    kz_dst = opencl_mat_mul(kgain_dst, zH_dst, kz);
    std::cout << "\nkz_dst:\n" << kz_dst << std::endl;

    // statePost = X(k)- + tmp3
    std::cout << "\n****** X(k)- + tmp3 \n"<< std::endl;
    cv::UMat statePost(kz_dst.size(), CV_32F);
    cv::UMat statePost_dst(statePost.size(), CV_32F);
    statePost_dst = opencl_mat_add(Xkm, kz_dst, statePost); 
    statePost_dst.copyTo(KF.statePost);
    std::cout << "\nKF.statePost:\n" << KF.statePost << std::endl;

    /*******************************************************/
    // Calculate errorCovPost
    /*******************************************************/    
    std::cout << "\n****** errorCovPost P(k) = (I - K * H) * P(k)- \n"<< std::endl;
    // tmp1 = Kk * H    
    std::cout << "\n****** tmp1 = Kk * H \n"<< std::endl;
    cv::UMat KH(kgain_dst.rows, H.cols, CV_32F);
    cv::UMat KH_dst(KH.size(), CV_32F);
    KH_dst = opencl_mat_mul(kgain_dst, H, KH_dst);
    std::cout << "\nKH_dst:\n" << KH_dst << std::endl;

    //** tmp2 = I - tmp1 **
    std::cout << "\n****** tmp2 = I - tmp1 \n"<< std::endl;
    cv::UMat IK2 = cv::UMat::ones(KH_dst.size(), CV_32F);
    cv::UMat IK_dst2 = cv::UMat::ones(KH_dst.size(), CV_32F);    
    cv::UMat I2 = cv::UMat::ones(KH_dst.size(), CV_32F);
    IK_dst2 = opencl_mat_add(I2, KH_dst, IK2);
    std::cout << "\nIK_dst2:\n" << IK_dst2 << std::endl;
    

    cv::UMat IK(KH_dst.size(), CV_32F);
    cv::UMat IK_dst(IK.size(), CV_32F);
    cv::UMat I = cv::UMat::eye(KH_dst.size(), CV_32F);
    IK_dst = opencl_mat_add(I, KH_dst, IK);

    // errorCovPost = tmp2 * errorCovPre(from predict stage)
    cv::UMat IZ(IK_dst.rows, KF.errorCovPre.cols, CV_32F);
    cv::UMat IZ_dst(IZ.size(), CV_32F);
    IZ_dst = opencl_mat_mul(IK_dst, zH_dst, IZ);
    IZ_dst.copyTo(KF.errorCovPost);
    std::cout << "\nKF.errorCovPost: \n" << KF.errorCovPost << std::endl;

}

int main()
{
    // ****** Data Initialization ********* //
    // Kalman filter here, DP = 16, MP = 8, CP = 0
    int DP = 16, MP = 8, CP = 0;
    cv::KalmanFilter KF(DP, MP, CP);

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

    /////////////////////////////////////////////////////////
    // Predict Stage
    /////////////////////////////////////////////////////////    
    
    cv::UMat measurement = cv::UMat::ones(DP, 1, CV_32F);
    kalman_predict(KF, measurement);

    
    /////////////////////////////////////////////////////////
    // Update Stage
    /////////////////////////////////////////////////////////    
    
    cv::UMat control = cv::UMat::ones(MP, 1, CV_32F);
    kalman_correct(KF, control);    

    return 0;
}