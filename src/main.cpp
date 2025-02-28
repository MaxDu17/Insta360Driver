﻿#include <iostream>
#include <thread>
#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include <regex>

#include <thread>
#include <chrono>

#ifdef WIN32
#include <conio.h>
#include <windows.h>
#include <direct.h>
#endif

class TestStreamDelegate : public ins_camera::StreamDelegate {
public:
    TestStreamDelegate() {
        file1_ = fopen("./01.h264", "wb");
        file2_ = fopen("./02.h264", "wb");
    }
    ~TestStreamDelegate() {
        fclose(file1_);
        fclose(file2_);
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        // std::cout << "on video frame:" << size << ";" << timestamp << std::endl;
        if (stream_index == 0) {
            fwrite(data, sizeof(uint8_t), size, file1_);
        }
        if (stream_index == 1) {
            fwrite(data, sizeof(uint8_t), size, file2_);
        }
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
};

int main(int argc, char* argv[]) {
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

    std::shared_ptr<ins_camera::Camera> cam = std::make_shared<ins_camera::Camera>(list[0].info);
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

    std::cout << "Usage" << std::endl;
    std::cout << "1: take photo" << std::endl;
    std::cout << "2: get serial number" << std::endl;
    std::cout << "3: get file list(only video and photo)" << std::endl;
    std::cout << "4: delete file" << std::endl;
    std::cout << "5: download file" << std::endl;
    std::cout << "6: start recording" << std::endl;
    std::cout << "7: stop recording" << std::endl;
    std::cout << "8: test set exposure settings:" << std::endl;
    std::cout << "9: test set capture settings:" << std::endl;
    std::cout << "10: start preview live streaming:" << std::endl;
    std::cout << "11: stop preview live streaming:" << std::endl;
    std::cout << "16: get uuid " << std::endl;
    std::cout << "17: test take photo and download " << std::endl;
    std::cout << "18: get current capture status " << std::endl;
    std::cout << "19: start timelapse " << std::endl;
    std::cout << "20: stop timelapse " << std::endl;
    std::cout << "21: get batty " << std::endl;
    std::cout << "22: get storage info " << std::endl;
    std::cout << "23: get recording file " << std::endl;
    std::cout << "30: batch download list files " << std::endl;
    std::cout << "31: delete camera all files " << std::endl;
    std::cout << "32: auto test timelapse " << std::endl;
    std::cout << "33: get file list count " << std::endl;
    std::cout << "35: upload fileware " << std::endl;
    std::cout << "0: exit" << std::endl;

    auto camera_type = cam->GetCameraType();

    auto start = time(NULL);
    cam->SyncLocalTimeToCamera(start);

    /*************************demo for 普通照片72MP和 HDR录制5.7K******************/
    //cam->SetPhotoSize(ins_camera::FUNCTION_MODE_NORMAL_IMAGE, ins_camera::PhotoSize::Size_6912_3456);
    //ins_camera::RecordParams record_params_test;
    //record_params_test.resolution = ins_camera::VideoResolution::RES_2880_2880P30;
    //cam->SetVideoCaptureParams(record_params_test, ins_camera::CameraFunctionMode::FUNCTION_MODE_HDR_VIDEO);
    /*************************demo for 普通照片72MP和 HDR录制5.7K******************/

    int option;
    while (true) {
        std::cout << "please enter index: ";
        std::cin >> option;
        if (option < 0 || option > 35) {
            std::cout << "Invalid index" << std::endl;
            continue;
        }

        if (option == 0) {
            break;
        }

        // take photo
        if (option == 1) {
            bool ret = cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE);
            if (!ret) {
                std::cout << "change submode failed!" << std::endl;
                continue;
            }

            const auto url = cam->TakePhoto();
            if (!url.IsSingleOrigin() || url.Empty()) {
                std::cout << "failed to take picture" << std::endl;
                break;
            }
            std::cout << "Take picture done: " << url.GetSingleOrigin() << std::endl;
        }

        // get serial number from camera
        if (option == 2) {
            const auto serial_number = cam->GetSerialNumber();
            if (serial_number.empty()) {
                std::cout << "failed to get serial number" << std::endl;
                break;
            }
            std::cout << "serial number: " << serial_number << std::endl;
        }

        // get camera file list(only video and photo for now)
        if (option == 3) {
            const auto file_list = cam->GetCameraFilesList();
            for (const auto& file : file_list) {
                std::cout << "File: " << file << std::endl;
            }
        }

        // delete file from camera
        if (option == 4) {
            std::string file_to_delete;
            std::cout << "please input full file path to delete: ";
            std::cin >> file_to_delete;
            const auto ret = cam->DeleteCameraFile(file_to_delete);
            if (ret) {
                std::cout << "Deletion succeed" << std::endl;
            }
        }

        // download file from camera
        if (option == 5) {
            std::string file_to_download;
            std::string file_to_save;
            std::cout << "please input full file path to download: ";
            std::cin >> file_to_download;
            std::cout << "please input full file path to save: ";
            std::cin >> file_to_save;

            const auto ret = cam->DownloadCameraFile(file_to_download,
                file_to_save, [](int64_t current, int64_t total_size) {
                //std::cout << "current :" << current << "; total_size: " << total_size << std::endl;
            });
            if (ret) {
                std::cout << "Download " << file_to_download << " succeed!!!" << std::endl;
            }
            else {
                std::cout << "Download " << file_to_download << " failed!!!" << std::endl;
            }
        }

        if (option == 6) {
            bool ret = cam->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_NORMAL);
            if (!ret) {
                std::cout << "change submode failed!" << std::endl;
                continue;
            }

            ins_camera::RecordParams record_params;
            record_params.resolution = ins_camera::VideoResolution::RES_2880_2880P60;
            record_params.bitrate = 1024 * 1024 * 10;
            if (!cam->SetVideoCaptureParams(record_params, ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_VIDEO)) {
                std::cerr << "failed to set capture settings." << std::endl;
            }
            else {
                auto ret = cam->StartRecording();
                if (ret) {
                    std::cerr << "success!" << std::endl;
                }
                else {
                    std::cerr << "failed to start recording" << std::endl;
                }
            }
        }

        if (option == 7) {
            auto url = cam->StopRecording();
            if (url.Empty()) {
                std::cerr << "stop recording failed" << std::endl;
                continue;
            }
            auto& origins = url.OriginUrls();
            std::cout << "stop recording success" << std::endl;
            for (auto& origin_url : origins) {
                std::cout << "url:" << origin_url << std::endl;
            }
        }

        if (option == 8) {
            auto exposure_settings = cam->GetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE);
            if (exposure_settings) {
                std::cout << "EVBias : " << exposure_settings->EVBias() << std::endl;
                std::cout << "ISO    : " << exposure_settings->Iso() << std::endl;
                std::cout << "speed  : " << exposure_settings->ShutterSpeed() << std::endl;
            }

            int bias;
            std::cout << "please enter EVBIOS: ";
            std::cin >> bias;
            auto exposure = std::make_shared<ins_camera::ExposureSettings>();
            auto exposure_mode = ins_camera::PhotographyOptions_ExposureMode::PhotographyOptions_ExposureOptions_Program_AUTO;
            if (camera_type == ins_camera::CameraType::Insta360X3 || camera_type == ins_camera::CameraType::Insta360X4) {
                exposure_mode = ins_camera::PhotographyOptions_ExposureMode::PhotographyOptions_ExposureOptions_Program_FULL_AUTO;
            }
            exposure->SetExposureMode(exposure_mode);
            exposure->SetEVBias(bias); // range -80 ~ 80, default 0, step 1
            exposure->SetIso(800);
            exposure->SetShutterSpeed(1.0 / 120.0);
            auto ret = cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, exposure);
            if (ret) {
                auto exposure_settings = cam->GetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE);
                std::cout << "success! ISO " << exposure_settings->Iso() << ", WB = " << exposure_settings->ShutterSpeed() << ", ExposureMode = " << exposure_settings->ExposureMode() << std::endl;
            }
            else {
                std::cerr << "failed to set exposure" << std::endl;
            }
        }

        if (option == 9) {
            auto settings = std::make_shared<ins_camera::CaptureSettings>();
            settings->SetValue(ins_camera::CaptureSettings::CaptureSettings_Saturation, 0);
            settings->SetWhiteBalance(ins_camera::PhotographyOptions_WhiteBalance_WB_4000K);
            settings->SetValue(ins_camera::CaptureSettings::CaptureSettings_Brightness, 100);
            auto ret = cam->SetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, settings);
            if (ret) {
                auto capture_settings = cam->GetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE);
                std::cout << "success!" << std::endl;
            }
            else {
                std::cerr << "failed to set capture settings" << std::endl;
            }
        }

        if (option == 10) {
            ins_camera::LiveStreamParam param;
            param.video_resolution = ins_camera::VideoResolution::RES_720_360P30;
            param.lrv_video_resulution = ins_camera::VideoResolution::RES_720_360P30;
            param.video_bitrate = 1024 * 1024 / 2;
            param.enable_audio = false;
            param.using_lrv = false;
            if (cam->StartLiveStreaming(param)) {
                std::cout << "successfully started live stream" << std::endl;
            }
        }

        if (option == 11) {
            if (cam->StopLiveStreaming()) {
                std::cout << "success!" << std::endl;
            }
            else {
                std::cerr << "failed to stop live." << std::endl;
            }
        }

        if (option == 16) {
            const auto str_uuid = cam->GetCameraUUID();
            if (str_uuid.empty()) {
                std::cerr << "failed to get uuid" << std::endl;
                return -1;
            }
            std::cout << "uuid : " << str_uuid << std::endl;
        }

        if (option == 17) {
            const auto url = cam->TakePhoto();
            if (!url.IsSingleOrigin() || url.Empty()) {
                std::cout << "failed to take picture" << std::endl;
                return -1;
            }

            std::string download_url = url.GetSingleOrigin();
            std::string save_path = "C:/Users/admin/Desktop/test" + download_url;
            const auto ret = cam->DownloadCameraFile(download_url, save_path);
            if (ret) {
                std::cout << "Download " << download_url << " succeed!!!" << std::endl;
            }
            else {
                std::cout << "Download " << download_url << " failed!!!" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        if (option == 35) {
            std::string loacl_path;
            std::string remoteFilePath;
            std::cout << "please input full file path to upload: ";
            std::cin >> loacl_path;
            //std::string loacl_path = "D:/Insta360X4FW_.bin";
            const auto ret = cam->UploadFile(loacl_path, "Insta360X4FW.bin",
                [](int64_t current, int64_t total_size) {
                std::cout << "current :" << current << "; total_size: " << total_size << std::endl;
            });
            if (ret) {
                std::cout << "Upload " << loacl_path << " succeed!!!" << std::endl;
            }
            else {
                std::cout << "Upload " << loacl_path << " failed!!!" << std::endl;
            }
            cam->Close();
            return 0;
        }

        if (option == 18) {
            auto ret = cam->GetCaptureCurrentStatus();
            if (ret == ins_camera::CaptureStatus::NOT_CAPTURE) {
                std::cout << "current statue : not capture" << std::endl;;
            }
            else {
                std::cout << "current statue : capture" << std::endl;
            }
        }

        if (option == 19) {
            bool ret = cam->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_TIMELAPSE);
            if (!ret) {
                std::cout << "change submode failed!" << std::endl;
                continue;
            }

            // 以RS一英寸为例 4k对应的分辨率是RES_3920_1920P30 6k对应的分辨率是RES_3920_1920P30
            // 4k为例
            ins_camera::RecordParams record_params;
            record_params.resolution = ins_camera::VideoResolution::RES_5120_5120P30;
            if (!cam->SetVideoCaptureParams(record_params, ins_camera::CameraFunctionMode::FUNCTION_MODE_MOBILE_TIMELAPSE)) {
                std::cerr << "failed to set capture settings." << std::endl;
                break;
            }

            //mode 是你相机所支持的模式
            ins_camera::TimelapseParam param = { ins_camera::CameraTimelapseMode::MOBILE_TIMELAPSE_VIDEO, 10,1000,5 };
            if (!cam->SetTimeLapseOption(param)) {
                std::cerr << "failed to set capture settings." << std::endl;
            }
            else {
                auto ret = cam->StartTimeLapse(param.mode);
                if (ret) {
                    std::cerr << "success!" << std::endl;
                }
                else {
                    std::cerr << "failed to start timelapse" << std::endl;
                }
            }
        }

        if (option == 20) {
            auto url = cam->StopTimeLapse(ins_camera::CameraTimelapseMode::MOBILE_TIMELAPSE_VIDEO);
            if (url.Empty()) {
                std::cerr << "stop timelapse failed" << std::endl;
                continue;
            }
            auto& origins = url.OriginUrls();
            std::cout << "stop timelapse success" << std::endl;
            for (auto& origin_url : origins) {
                std::cout << "url:" << origin_url << std::endl;
            }
        }

        if (option == 21) {
            if (!cam->IsConnected()) {
                std::cout << "device is offline" << std::endl;
                break;
            }

            ins_camera::BatteryStatus status;
            bool ret = cam->GetBatteryStatus(status);
            if (!ret) {
                std::cerr << "GetBatteryStatus failed" << std::endl;
                continue;
            }
            std::cout << "PowerType : " << status.power_type << std::endl;
            std::cout << "battery_level : " << status.battery_level << std::endl;
            std::cout << "battery_scale : " << status.battery_scale << std::endl;
        }

        if (option == 22) {
            ins_camera::StorageStatus status;
            bool ret = cam->GetStorageState(status);
            if (!ret) {
                std::cerr << "GetBatteryStatus failed" << std::endl;
                continue;
            }
            std::cout << "free_space : " << status.free_space << std::endl;
            std::cout << "total_space : " << status.total_space << std::endl;
            std::cout << "state : " << status.state << std::endl;
        }

        if (option == 23) {
            std::vector<std::string> file_list;
            bool ret = cam->GetRecordingFiles(file_list);
            if (!ret) {
                std::cerr << "GetRecordingFiles failed" << std::endl;
                continue;
            }
            for (auto& file : file_list) {
                std::cout << file << std::endl;
            }
        }

        if (option == 24) {
            bool is_video = true;
            bool ret;
            if (is_video) {
                ret = cam->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_NORMAL);
            }
            else {
                ret = cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE);
            }
            if (ret) {
                std::cout << "Change Submode Succeed!" << std::endl;
                continue;
            }
            else {
                std::cout << "Change Submode Failed!" << std::endl;
                continue;
            }
        }

        if (option == 30) {
            std::vector<std::string> file_lists = cam->GetCameraFilesList();
            if (file_lists.size() > 0) {
                std::string file_to_save;
                std::cout << "please input full file path to save: ";
                std::cin >> file_to_save;

                int begin, end;
                std::cout << "please enter begin index: ";
                std::cin >> begin;
                std::cout << "please enter end index: ";
                std::cin >> end;
                int i = begin;

                if (begin < 0) {
                    begin = 0;
                }
                if (end > file_lists.size() - 1) {
                    end = file_lists.size() - 1;
                }

                for (i = begin; i <= end; i++) {
                    auto file = file_lists[i];
                    std::string fileName = file.substr(file.find_last_of("/"));
                    std::string save_path = file_to_save + fileName;
                    int ret = cam->DownloadCameraFile(file, save_path);
                    if (ret) {
                        std::cout << "Download " << file << " succeed!!!" << std::endl;
                    }
                    else {
                        std::cout << "Download " << file << " failed!!!" << std::endl;
                    }
                }
            }
            continue;
        }

        if (option == 31) {
            const auto file_list = cam->GetCameraFilesList();
            for (const auto& file : file_list) {
                const auto ret = cam->DeleteCameraFile(file);
                if (ret) {
                    std::cout << file << " Deletion succeed" << std::endl;
                }
            }
        }

        if (option == 32) {
#ifdef WIN32
            while (true) {
                ins_camera::RecordParams record_params;
                record_params.resolution = ins_camera::VideoResolution::RES_2944_2880P30;
                record_params.bitrate = { 1024 * 1024 * 10 };
                cam->SetVideoCaptureParams(record_params, ins_camera::CameraFunctionMode::FUNCTION_MODE_STATIC_TIMELAPSE);
                ins_camera::TimelapseParam param;
                param.mode = ins_camera::CameraTimelapseMode::MOBILE_TIMELAPSE_VIDEO;
                param.duration = -1;
                param.lapseTime = 500;
                param.accelerate_fequency = 5;
                cam->SetTimeLapseOption(param);

                auto ret = cam->StartTimeLapse(ins_camera::CameraTimelapseMode::MOBILE_TIMELAPSE_VIDEO);
                if (ret) {
                    std::cout << "Start timeLapse succeed, Type q or Q exit!" << std::endl;
                    bool exit_flag = false;

                    std::chrono::seconds sleepDuration(60 * 12);
                    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
                    // 启动一个线程进行睡眠
                    std::thread sleepThread([&] {while (true) {
                        // 检查键盘输入
                        if (_kbhit()) {
                            char ch = _getch();
                            if (ch == 'q' || ch == 'Q') {
                                exit_flag = true;
                                break;
                            }
                        }

                        // 检查睡眠是否完成
                        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
                        std::chrono::seconds elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
                        if (elapsedTime >= sleepDuration) {
                            break;
                        }
                    }});

                    // 等待线程完成
                    sleepThread.join();

                    auto ret1 = cam->StopTimeLapse(ins_camera::CameraTimelapseMode::MOBILE_TIMELAPSE_VIDEO);
                    if (!ret1.Empty()) {
                        std::cout << "Stop timeLapse succeed!" << std::endl;
                    }
                    else {
                        std::cout << "Stop timeLapse failed!" << std::endl;
                    }

                    if (exit_flag) {
                        break;
                    }
                }
                else {
                    std::cout << "Start timeLapse failed!" << std::endl;
                }
            }
#endif
        }

        if (option == 33) {
            int count = 0;
            auto ret = cam->GetCameraFilesCount(count);
            if (ret) {
                std::cout << "The count of files is:" << count << std::endl;
            }
            else {
                std::cout << "get files count failed!!!" << std::endl;
            }
        }
    }

    cam->Close();

    return 0;
}