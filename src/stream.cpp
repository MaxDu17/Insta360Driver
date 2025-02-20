#include <iostream>
#include <thread>
#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include <regex>

#include <opencv2/opencv.hpp>
// #include "opencv4"

#include <netinet/in.h>
#include <thread>
#include <chrono>

#include <fstream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


#ifdef WIN32
#include <conio.h>
#include <windows.h>
#include <direct.h>
#endif

// Define constants


// int sock; 
 // Create socket




class TestStreamDelegate : public ins_camera::StreamDelegate {
public:
    TestStreamDelegate() {
        std::string SERVER_IP = "127.0.0.1";  // Replace with your receiver's IP address
        int SERVER_PORT = 8080;

        file1_ = fopen("./01.h264", "wb");
        file2_ = fopen("./02.h264", "wb");

        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation failed" << std::endl;
            return;
        }

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(SERVER_PORT);

        // Convert IP address from string to binary form
        if (inet_pton(AF_INET, SERVER_IP.c_str(), &serverAddr.sin_addr) <= 0) {
            std::cerr << "Invalid address or address not supported" << std::endl;
            close(sock);
            return;
        }

        // Connect to the Python server
        if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Connection failed" << std::endl;
            close(sock);
            return;
        }
    }
    ~TestStreamDelegate() {
        fclose(file1_);
        fclose(file2_);
        close(sock); 
    }


    void sendImage(const uint8_t* imageData, size_t imageSize) {
        // Send the size of the image first
        ssize_t sent = send(sock, &imageSize, sizeof(imageSize), 0);
        std::cout << imageSize << std::endl; 
        if (sent < 0) {
            std::cerr << "Failed to send image size" << std::endl;
            close(sock);
            return;
        }

        // Send the image data
        sent = send(sock, imageData, imageSize, 0);
        if (sent < 0) {
            std::cerr << "Failed to send image data" << std::endl;
            close(sock);
            return;
        }

        std::cout << "Image sent successfully!" << std::endl;

    // close(sock); 
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        // std::cout << "on video frame:" << size << ";" << timestamp << std::endl;
        if (stream_index == 0) {
            sendImage(data, size); 
            // fwrite(data, sizeof(uint8_t), size, file1_);
        }
        // if (stream_index == 1) {
        //     fwrite(data, sizeof(uint8_t), size, file2_);
        // }
    }
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        //for (auto& gyro : data) {
        //	if (gyro.timestamp - last_timestamp > 2) {
        //		fprintf(file1_, "timestamp:%lld package_size = %d  offtimestamp = %lld gyro:[%f %f %f] accel:[%f %f %f]\n", gyro.timestamp, data.size(), gyro.timestamp - last_timestamp, gyro.gx, gyro.gy, gyro.gz, gyro.ax, gyro.ay, gyro.az);
        //	}
        //	last_timestamp = gyro.timestamp;
        //}
    }
    void OnExposureData(const ins_camera::ExposureData& data) override {
        //fprintf(file2_, "timestamp:%lld shutter_speed_s:%f\n", data.timestamp, data.exposure_time);
    }

private:
    FILE* file1_;
    FILE* file2_;
    int64_t last_timestamp = 0;
    int sock; 
};

std::shared_ptr<ins_camera::Camera> cam; 

void onExit(){
   if (cam->StopLiveStreaming()) {
            std::cout << "Successfully closed stream!" << std::endl;
        }
        else {
            std::cerr << "failed to stop live." << std::endl;
        }
    cam->Close();
}


int main(int argc, char* argv[]) {
    // std::atexit(onExit); 
    std::cout << "begin open camera" << std::endl;
    ins_camera::DeviceDiscovery discovery;
    auto list = discovery.GetAvailableDevices();
    for (int i = 0; i < list.size(); ++i) {
        auto desc = list[i];
        std::cout << "serial:" << desc.serial_number << "\t"
            << "camera type:" << int(desc.camera_type) << "\t"
            << "lens type:" << int(desc.lens_type) << std::endl;
    }

    if (list.size() <= 0) {
        std::cerr << "no device found." << std::endl;
        return -1;
    }

    cam = std::make_shared<ins_camera::Camera>(list[0].info);
    //ins_camera::Camera cam(list[0].info);
    if (!cam->Open()) {
        std::cerr << "failed to open camera" << std::endl;
        return -1;
    }

    //std::cout << "http base url:" << cam->GetHttpBaseUrl() << std::endl;

    std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>();
    cam->SetStreamDelegate(delegate);

    discovery.FreeDeviceDescriptors(list);

    std::cout << "Succeed to open camera..." << std::endl;

    auto camera_type = cam->GetCameraType();

    auto start = time(NULL);
    cam->SyncLocalTimeToCamera(start);

    ins_camera::LiveStreamParam param;
    param.video_resolution = ins_camera::VideoResolution::RES_720_360P30;
    param.lrv_video_resulution = ins_camera::VideoResolution::RES_720_360P30;
    param.video_bitrate = 1024 * 1024 / 2;
    param.enable_audio = false;
    param.using_lrv = false;
    if (cam->StartLiveStreaming(param)) {
        std::cout << "successfully started live stream" << std::endl;
    }
    while(true); //hang until we quit 

    return 0;
}