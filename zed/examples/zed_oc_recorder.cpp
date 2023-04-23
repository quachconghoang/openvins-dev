// ----> Includes
#include "videocapture.hpp"
#include "sensorcapture.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
// <---- Includes

// ----> Functions
// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap);

void getImagesThreadFunc();

// <---- Functions

// ----> Global variables
std::mutex imuMutex;
std::string imuTsStr;
std::string imuAccelStr;
std::string imuGyroStr;

//std::string format_account_number(int acct_no) {
//    char buffer[6];
//    std::snprintf(buffer, sizeof(buffer), "%08d", acct_no);
//    return buffer;
//}

bool sensThreadStop = false;
bool imgsThreadStop = false;
bool recording = false;

const double deg2rad = M_PI/180;
uint64_t mcu_sync_ts=0;
std::queue<cv::Mat> buff_raw_queue;
std::vector<std::vector<uchar>> buff_encode;
std::vector<uint64_t> buff_timestamp;

std::vector<std::string> buff_imu;

// <---- Global variables

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

//    sl_oc::sensors::SensorCapture::resetSensorModule();
//    sl_oc::sensors::SensorCapture::resetVideoModule();

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::ERROR;

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_60;
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    sl_oc::video::VideoCapture videoCap(params);
    if( !videoCap.initializeVideo(-1) ){
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;
        return EXIT_FAILURE;
    }

    // Serial number of the connected camera
    int camSn = videoCap.getSerialNumber();
    std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sl_oc::sensors::SensorCapture sensCap(verbose);
    if( !sensCap.initializeSensors(camSn) ){
        std::cerr << "Cannot open sensors capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber() << std::endl;

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    // <---- Create Sensors Capture
    std::thread sensThread(getSensorThreadFunc,&sensCap);
    std::thread imgsThread(getImagesThreadFunc);

    // ----> Enable video/sensors synchronization
    videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Init OpenCV RGB frame
    int w,h;
    videoCap.getFrameSize(w,h);
    videoCap.setAutoWhiteBalance(true);

    cv::Size display_resolution(1024, 576);

    switch(params.res)
    {
        default:
        case sl_oc::video::RESOLUTION::VGA:
            display_resolution.width = w;
            display_resolution.height = h;
            break;
        case sl_oc::video::RESOLUTION::HD720:
            display_resolution.width = w*0.6;
            display_resolution.height = h*0.6;
            break;
        case sl_oc::video::RESOLUTION::HD1080:
        case sl_oc::video::RESOLUTION::HD2K:
            display_resolution.width = w*0.4;
            display_resolution.height = h*0.4;
            break;
    }

    int h_data = 70;
    int h_action = 40;
    cv::Mat frameDisplay(display_resolution.height + h_data + h_action, display_resolution.width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat frameData = frameDisplay(cv::Rect(0,0, display_resolution.width, h_data));
    cv::Mat frameBGRDisplay = frameDisplay(cv::Rect(0,h_data, display_resolution.width, display_resolution.height));
    cv::Mat frameBGR(h, w, CV_8UC3, cv::Scalar(0,0,0));

    cv::Mat frameAction = frameDisplay(cv::Rect(0,h_data + display_resolution.height, display_resolution.width, h_action));
//    cv::circle(frameAction,cv::Point(display_resolution.width - 35, 35), 28, cv::Scalar(0,0,255),-1);
    // <---- Init OpenCV RGB frame

    uint64_t last_timestamp = 0;
    uint64_t update_timestamp = 49*1e6;
    float frame_fps=0;

    // Infinite grabbing loop
    while (1)
    {
        // ----> Get Video frame
        // Get last available frame
        const sl_oc::video::Frame frame = videoCap.getLastFrame(1);

        if(frame.data!=nullptr && frame.timestamp!=last_timestamp && (frame.timestamp-last_timestamp)> update_timestamp)
        {
            frame_fps = 1e9/static_cast<float>(frame.timestamp-last_timestamp);
            last_timestamp = frame.timestamp;
            cv::Mat frameYUV( frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV,frameBGR, cv::COLOR_YUV2BGR_YUYV);
            if(recording)
            {
                buff_timestamp.push_back(frame.timestamp);
                buff_raw_queue.push(frameBGR);

            }
        }
        // <---- Get Video frame

        // ----> Video Debug information
        std::stringstream videoTs;
        videoTs << std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_timestamp)/1e9<< " sec" ;
        if( last_timestamp!=0 )
            videoTs << std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]";
        // <---- Video Debug information

        if(frame.data!=nullptr)
        {
            frameData.setTo(0);

            if(recording) frameAction.setTo(128);
            else frameAction.setTo(0);

            // Video info
            cv::putText( frameData, videoTs.str(), cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(241,240,236));
            // IMU info
            imuMutex.lock();
            cv::putText( frameData, imuTsStr, cv::Point(10, 35),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(241,240,236));

            // Timestamp offset info
            std::stringstream offsetStr;
            double offset = (static_cast<double>(frame.timestamp)-static_cast<double>(mcu_sync_ts))/1e9;
            offsetStr << std::fixed << std::setprecision(9) << std::showpos << "Timestamp offset: " << offset << " sec [video-sensors]";
            cv::putText( frameData, offsetStr.str().c_str(), cv::Point(10, 50),cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(241,240,236));

            // Average timestamp offset info (we wait at least 200 frames to be sure that offset is stable)
            if( frame.frame_id>200 )
            {
                static double sum=0;
                static int count=0;
                sum += offset;
                double avg_offset=sum/(++count);
                std::stringstream avgOffsetStr;
                avgOffsetStr << std::fixed << std::setprecision(9) << std::showpos << "Avg timestamp offset: " << avg_offset << " sec";
                cv::putText( frameData, avgOffsetStr.str().c_str(), cv::Point(10,62),cv::FONT_HERSHEY_SIMPLEX,0.35, cv::Scalar(241, 240,236));
            }

            // IMU values
            cv::putText( frameData, "Inertial sensor data:", cv::Point(display_resolution.width/2,20),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            cv::putText( frameData, imuAccelStr, cv::Point(display_resolution.width/2+15,42),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            cv::putText( frameData, imuGyroStr, cv::Point(display_resolution.width/2+15, 62),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            imuMutex.unlock();

            // Resize Image for display
            cv::resize(frameBGR, frameBGRDisplay, display_resolution);
            // Display image
            cv::imshow( "Capture-Apps", frameDisplay);
        }

        // ----> Keyboard handling
        int key = cv::waitKey(1);

        if( key != -1 )
        {
            if(key =='r')   {
                while(!buff_raw_queue.empty()) buff_raw_queue.pop();
                buff_encode.clear();
                buff_imu.clear();
                buff_timestamp.clear();

                recording = true;
            }

            if(key =='s')   recording = false;

            // Quit
            if(key=='q' || key=='Q'|| key==27)
            {
                recording = false;
                sensThreadStop=true;    sensThread.join();
                imgsThreadStop=true;    imgsThread.join();
                break;
            }
        }
        // <---- Keyboard handling
    }

    videoCap.~VideoCapture();

    return EXIT_SUCCESS;
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap)
{
    // Flag to stop the thread
    sensThreadStop = false;

    // Previous IMU timestamp to calculate frequency
    uint64_t last_imu_ts = 0;

    // Infinite data grabbing loop
    while(!sensThreadStop)
    {
        // ----> Get IMU data
        const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

        // Process data only if valid
        if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
        {
            // ----> Data info to be displayed
            std::stringstream timestamp;
            std::stringstream accel;
            std::stringstream gyro;

            timestamp << std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" ;
            if(last_imu_ts!=0)
                timestamp << std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]";
            last_imu_ts = imuData.timestamp;

            accel << std::fixed << std::showpos << std::setprecision(4) << " * Accel: " << imuData.aX << " " << imuData.aY << " " << imuData.aZ << " [m/s^2]";
            gyro << std::fixed << std::showpos << std::setprecision(4) << " * Gyro: " << imuData.gX*deg2rad << " " << imuData.gY*deg2rad << " " << imuData.gZ*deg2rad << " [rad/s]";
            // <---- Data info to be displayed

            // Mutex to not overwrite data while diplaying them
            imuMutex.lock();

            imuTsStr = timestamp.str();
            imuAccelStr = accel.str();
            imuGyroStr = gyro.str();

            // ----> Timestamp of the synchronized data
            if(imuData.sync)
            {
                mcu_sync_ts = imuData.timestamp;
            }
            // <---- Timestamp of the synchronized data

            imuMutex.unlock();
        }
        // <---- Get IMU data
    }

    std::cout << "End IMU thread\n";
}

void getImagesThreadFunc()
{
    imgsThreadStop = false;
    std::vector<int> encode_param;
    encode_param.push_back(cv::IMWRITE_JPEG_QUALITY) ;
    encode_param.push_back(95);

    std::queue<int> xxx;

    while (!imgsThreadStop){
        while (recording){
            if(buff_raw_queue.size() > 0){
                std::vector<uchar> buff;
                cv::Mat m = buff_raw_queue.front();
                cv::imencode(".jpg",m,buff, encode_param);
                buff_raw_queue.pop();
                buff_encode.push_back(buff);
            }
            std::this_thread::sleep_for(std::chrono::microseconds (1));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "End Image thread\n";

    double dumb_size = 0;
    for(int i=0; i<buff_encode.size(); i++){
        double img_size = double(buff_encode[i].size())/1048576;
        dumb_size+=img_size;

        // FILE Dumb
        if(std::FILE * f1 = std::fopen("test.jpg", "wb")) {
            std::fwrite(buff_encode[i].data(), sizeof(uchar), buff_encode[i].size(), f1);
            std::fclose(f1);
        }
    }

    for(int i=0; i<buff_imu.size(); i++){

    }
    std::cout << "Total: " << dumb_size << " MB; " << buff_timestamp.size() << " images.\n";
}