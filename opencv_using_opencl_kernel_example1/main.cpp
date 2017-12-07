#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main()
{
    // check OpenCL availability
    if (!cv::ocl::haveOpenCL())
    {
        cout << "OpenCL is not avaiable..." << endl;
        return -1;
    }

    // create context
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        cout << "Failed creating the context..." << endl;
        return -1;
    }

    // device detection
    cout << context.ndevices() << " GPU devices are detected." << endl;
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name                 : " << device.name() << endl;
        cout << "available            : " << device.available() << endl;
        cout << "imageSupport         : " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;        
    }

    // Select the first device
    cv::ocl::Device(context.device(0));

    // Transfer Mat data to the device
    cv::Mat mat_src = cv::imread("test_img.jpg", cv::IMREAD_GRAYSCALE);
    cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat umat_dst(mat_src.size(), mat_src.type(), cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    
    string ty =  type2str( mat_src.type() );
    printf("Matrix: %s %dx%d \n", ty.c_str(), mat_src.cols, mat_src.rows );
    ty =  type2str( umat_src.type() );
    printf("Matrix: %s %dx%d \n", ty.c_str(), umat_src.cols, umat_src.rows );
    ty =  type2str( umat_dst.type() );
    printf("Matrix: %s %dx%d \n", ty.c_str(), umat_dst.cols, umat_dst.rows );

    // Read the OpenCL kernel code
    std::ifstream ifs("neg.cl");
    if (ifs.fail()) 
        return -1;
    std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    cv::ocl::ProgramSource programSource(kernelSource);

    // Compile the kernel code
    cv::String errmsg;
    cv::String buildopt = ""; // By setting "-D xxx=yyy ", we can replace xxx with yyy in the kernel
    cv::ocl::Program program = context.getProg(programSource, buildopt, errmsg);

    // create kernel
    cv::ocl::Kernel kernel("neg", program);
    // kernel argument
    kernel.args(cv::ocl::KernelArg::ReadOnlyNoSize(umat_src), cv::ocl::KernelArg::ReadWrite(umat_dst));

    size_t globalThreads[3] = { (size_t)mat_src.cols, (size_t)mat_src.rows, 1 };
    //size_t localThreads[3] = { 16, 16, 1 };

    bool success = kernel.run(3, globalThreads, NULL, true);
    if (!success){
        cout << "Failed running the kernel..." << endl;
        return -1;
    }

    // Download the dst data from the device (?)
    cv::Mat mat_dst = umat_dst.getMat(cv::ACCESS_READ);
    std::cout << mat_dst << std::endl;

    //cv::imshow("src", mat_src);
    //cv::imshow("dst", mat_dst);
    //cv::waitKey();
    cout << "Succeed" << endl;

    return 0;
}~