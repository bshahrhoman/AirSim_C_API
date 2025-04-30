// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <fstream>

using namespace msr::airlib;

Vector3r getLocalPosition(MultirotorRpcLibClient &client) {
    // simGetVehiclePose returns the current Pose (position+orientation)
    auto pose = client.simGetVehiclePose();
    return pose.position;
}

// Fetches the vehicle's attitude (roll, pitch, yaw) in degrees using the Pose API
std::tuple<float, float, float> getAttitudeDegrees(MultirotorRpcLibClient &client) {
    auto pose = client.simGetVehiclePose();
    Quaternionr ori = pose.orientation;
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(ori, pitch, roll, yaw);
    const float rad2deg = 180.0f / static_cast<float>(M_PIf);
    return std::make_tuple(roll * rad2deg, pitch * rad2deg, yaw * rad2deg);
}

// Captures a downward image from the specified camera name and returns raw uint8 data
std::vector<uint8_t> captureDownwardImage(MultirotorRpcLibClient &client,
                                          const std::string &camera_name,
                                          int &out_width, int &out_height) {
    std::vector<ImageCaptureBase::ImageRequest> requests;
    // false=pixels as float, true=compressed
    requests.emplace_back(camera_name, ImageCaptureBase::ImageType::Scene, false, true);
    auto responses = client.simGetImages(requests);
    if (responses.empty()) {
        throw std::runtime_error("No images received from camera " + camera_name);
    }
    out_width = responses[0].width;
    out_height = responses[0].height;
    return responses[0].image_data_uint8;
}

int main() {
    // 1) Connect with explicit IP (and default RPC port 41451)
    // Change to IP of SITL Instance
    MultirotorRpcLibClient client("172.24.192.1");
    try {
        client.confirmConnection();
        std::cout << "Connected to AirSim at 172.24.192.1" << std::endl;

        // 2) Get and print local position
        auto pos = getLocalPosition(client);
        std::cout << "Vehicle position (NED): x=" << pos.x()
                  << ", y=" << pos.y() << ", z=" << pos.z() << std::endl;

        // 3) Get and print attitude
        auto [roll_deg, pitch_deg, yaw_deg] = getAttitudeDegrees(client);
        std::cout << "Vehicle attitude (deg): roll=" << roll_deg
                  << ", pitch=" << pitch_deg
                  << ", yaw=" << yaw_deg << std::endl;

        // 4) Capture and save downward image for debugging
        int width, height;
        auto image_data = captureDownwardImage(client, "3", width, height);
        std::string filename = "downward_view.png";
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char *>(image_data.data()), image_data.size());
        file.close();
        std::cout << "Captured image " << width << "x" << height
                  << " and saved to " << filename << std::endl;
    }
    catch (rpc::rpc_error &e) {
        std::cerr << "RPC Error: " << e.get_error().as<std::string>() << std::endl;
        return -1;
    }
    catch (const std::exception &ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
