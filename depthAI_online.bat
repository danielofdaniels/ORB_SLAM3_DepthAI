set SLAM=.\x64\Release\slam.exe

REM ------------------------------------
echo "Launching Live feed with Stereo sensor"
%SLAM% stereo_depthAI_online ./Vocabulary/ORBvoc.txt ./stereo-online.yaml stereo_depthAI_online > ./log/log001_depthAI_stereo_online.txt
