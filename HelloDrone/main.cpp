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

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <tuple>

using namespace msr::airlib;

Vector3r getLocalPosition(MultirotorRpcLibClient &client) {
    auto pose = client.simGetVehiclePose();
    return pose.position;
}

std::tuple<float, float, float> getAttitudeDegrees(MultirotorRpcLibClient &client) {
    auto pose = client.simGetVehiclePose();
    auto ori = pose.orientation;
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(ori, pitch, roll, yaw);
    constexpr float rad2deg = 180.0f / static_cast<float>(M_PIf);
    return { roll * rad2deg, pitch * rad2deg, yaw * rad2deg };
}

std::vector<uint8_t> captureDownwardImage(MultirotorRpcLibClient &client,
                                          const std::string &camera_name,
                                          int &out_width, int &out_height) {
    std::vector<ImageCaptureBase::ImageRequest> reqs;
    // false = float pixels, true = compressed (PNG)
    reqs.emplace_back(camera_name, ImageCaptureBase::ImageType::Scene, false, true);
    auto resp = client.simGetImages(reqs);
    if (resp.empty())
        throw std::runtime_error("No images from camera " + camera_name);
    out_width  = resp[0].width;
    out_height = resp[0].height;
    return resp[0].image_data_uint8;
}


int main() {
    // 1) Connect to AirSim SITS on WSL2
    MultirotorRpcLibClient client("172.24.192.1");
    try {
        client.confirmConnection();
        std::cout << "Connected to AirSim at 172.24.192.1\n";

        // 2) Print initial state for debug
        auto pos = getLocalPosition(client);
        std::cout << "Pos (NED): x=" << pos.x() << " y=" << pos.y() << " z=" << pos.z() << "\n";
        auto [r,p,y] = getAttitudeDegrees(client);
        std::cout << "Att (deg): roll=" << r << " pitch=" << p << " yaw=" << y << "\n";

        // 3) Create TCP listening socket
        int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            perror("socket");
            return -1;
        }
        sockaddr_in serv_addr{};
        serv_addr.sin_family      = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;      // listen on all interfaces
        serv_addr.sin_port        = htons(9000);     // choose any port
        if (bind(listen_fd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            perror("bind");
            return -1;
        }
        listen(listen_fd, 1);
        std::cout << "Waiting for client on port 9000...\n";

        int client_fd = accept(listen_fd, nullptr, nullptr);
        if (client_fd < 0) {
            perror("accept");
            return -1;
        }
        std::cout << "Client connected, starting stream.\n";

        // 4) Capture & send loop
        while (true) {
            int w, h;
            auto img_data = captureDownwardImage(client, "3", w, h);

            uint32_t len = htonl(static_cast<uint32_t>(img_data.size()));
            if (send(client_fd, &len, sizeof(len), 0) != sizeof(len)) {
                perror("send length");
                break;
            }

            size_t sent = 0;
            while (sent < img_data.size()) {
                ssize_t n = send(client_fd,
                                 img_data.data() + sent,
                                 img_data.size() - sent,
                                 0);
                if (n <= 0) {
                    perror("send data");
                    goto cleanup;
                }
                sent += n;
            }
            std::cout << "Sent frame" << w << "×" << h
                      << "), " << img_data.size() << " bytes\n";
            usleep(100000); // ~10 FPS
        }

    cleanup:
        close(client_fd);
        close(listen_fd);
        std::cout << "Streaming ended, sockets closed.\n";
    }
    catch (rpc::rpc_error &e) {
        std::cerr << "RPC Error: " << e.get_error().as<std::string>() << "\n";
        return -1;
    }
    catch (const std::exception &ex) {
        std::cerr << "Exception: " << ex.what() << "\n";
        return -1;
    }

    return 0;
}
