# ORB_SLAM3_DepthAI
ORB SLAM3 running using live data from a stereo luxinous DepthAI chip.

Modified from the windows version (https://github.com/rexdsp/ORB_SLAM3_Windows) of ORB_SLAM3 https://github.com/UZ-SLAMLab/ORB_SLAM3 to integrate the luxinous API https://github.com/luxonis/depthai-core. Tested on Windows 10 with Visual Studio 2019. It is assumed your DepthAI camera setup has the correct calibration stored on-chip. 

Dependencies:
  - OpenCV 4 (tested with 4.5.3)
  - cmake >= 3.4
  - depthai-core & depthai-opencv dlls (https://github.com/luxonis/depthai-core)

Eigen, g20, & pangolin and required but pre-packaged thanks to the windows implementation this is derived from. 


TODO:
  - Transfer lens distortion removal from an image based to key point based operation
