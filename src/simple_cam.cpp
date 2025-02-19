#include <iostream>
#include <thread>

#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include "camera/ins_types.h"

#include "stream/stream_delegate.h"
#include "stream/stream_types.h"

// #include "regex"

#include <thread>
#include <chrono>
int main(){
    ins_camera::DeviceDiscovery discovery;
    auto list = discovery.GetAvailableDevices();
    for(int i = 0;i < list.size(); ++i) {
        std::cout << "device:" << list[i].serial_number << std::endl;
    }
}


