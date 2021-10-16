/**
* This file is a modification of the examples provided in ORB-SLAM3
*
* Copyright (C) 2021 Daniel Griffiths, BEER Labs
* Initial Copywrites:
*   Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*   Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "global.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <System.h>

// Include DepthAI header
//      Built using DepthAI-core: https://github.com/luxonis/depthai-core
#include "depthai/depthai.hpp"


using namespace std;

int stereo_depthAI_online(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./stereo_depthAI_online path_to_vocabulary path_to_settings" << endl;

        return 1;
    }

    // Parse filename for export
    bool bFileName = (((argc - 3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Custom Settings for DepthAI
    //      TODO: Add resolution control
    float fps; fsSettings["Camera.fps"] >> fps;

    /*
        Create DepthAI Pipeline
    */
    std::cout << "Starting DepthAI Pipeline" << std::endl;
    dai::Pipeline pipeline;
    // Sources
    std::cout << "\tSources" << std::endl;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    // Outputs
    std::cout << "\tOutputs" << std::endl;
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();
    // Name Output Streams
    std::cout << "\tStream Names" << std::endl;
    xoutRectifL->setStreamName("rectified_left");
    xoutRectifR->setStreamName("rectified_right");
    // Set Properties
    std::cout << "\tProperties" << std::endl;
    monoLeft->setFps(fps);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setFps(fps);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    stereo->initialConfig.setConfidenceThreshold(200);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(false);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(false);
    stereo->setRectification(true);
    stereo->setRectifyMirrorFrame(false);
    // Linking
    std::cout << "\tLinks" << std::endl;
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->rectifiedLeft.link(xoutRectifL->input);
    stereo->rectifiedRight.link(xoutRectifR->input);
    // Connect to device and start pipeline
    std::cout << "\tInit Device" << std::endl;
    dai::Device device(pipeline);
    // Create Output Queue/s
    std::cout << "\tQueues" << std::endl;
    auto qLeftRect = device.getOutputQueue("rectified_left", 4, false);
    auto qRightRect = device.getOutputQueue("rectified_right", 4, false);

    // Load intrinsic distortion parameters from camera memory
    auto calibration = device.readCalibration();
    std::vector<std::vector<float>> intrinsics_l_vec = calibration.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, (720, 1280));
    std::vector<std::vector<float>> intrinsics_r_vec = calibration.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, (720, 1280));     
    cv::Mat intrinsics_l = (cv::Mat1d(3, 3) << intrinsics_l_vec[0][0], 0, intrinsics_l_vec[0][2], 0, intrinsics_l_vec[1][1], intrinsics_l_vec[1][2], 0, 0, 1);
    cv::Mat intrinsics_r = (cv::Mat1d(3, 3) << intrinsics_r_vec[0][0], 0, intrinsics_r_vec[0][2], 0, intrinsics_r_vec[1][1], intrinsics_r_vec[1][2], 0, 0, 1);
    std::vector<float> distcoeff_l = calibration.getDistortionCoefficients(dai::CameraBoardSocket::LEFT);
    std::vector<float> distcoeff_r = calibration.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::cout << "Create SLAM Object" << std::endl;
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);
    SLAM.ChangeDataset();

    // Declare containers
    cv::Mat imLeft, imRight;
    cv::Mat imLeftRect, imRightRect;
    double tframe;
    std::string frame_idx;
    
    // Capture & SLAM loop
    std::cout << "Starting Processing Loop" << std::endl;
    while(true)
    {        
        // Acquire Rectified Images
        auto inLeft = qLeftRect->get<dai::ImgFrame>();
        imLeft = inLeft->getFrame();
        auto inRight = qRightRect->get<dai::ImgFrame>();
        imRight = inRight->getFrame();

        // Undistrort the bitch
        // Thank you depthAI
        cv::undistort(imLeft, imLeftRect, intrinsics_l, distcoeff_l);
        cv::undistort(imRight, imRightRect, intrinsics_r, distcoeff_r);

        // Acquire Image Timestamp -- Not used without IMU?
        tframe = 1e-6*(std::chrono::time_point_cast<std::chrono::microseconds>(inRight->getTimestamp())).time_since_epoch().count();

        // Frame Name
        frame_idx = std::to_string((unsigned long long)tframe);

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeftRect,imRightRect,tframe,vector<ORB_SLAM3::IMU::Point>(),frame_idx);
        
        // Escape Hatch
        //    TODO: Should be inside the Viewer.cpp
        cv::imshow("q to quit", imLeftRect);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }
    }
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "log/kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "log/f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("log/CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("log/KeyFrameTrajectory.txt");
    }

    return 0;
}
