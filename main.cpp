
#include "global.h"

// Monocular

// Monocular-Inertial

// Stereo
int stereo_depthAI_online(int argc, char** argv);

// Stereo-Inertial


int main(int argc, char** argv)
{
    printf("ORB-SLAM3");
    for (int i = 1; i < argc; i++)
        printf(" %s", argv[i]);
    printf("\n");

    argc--;
    // Monocular

    // Monocular-Inertial

    // Stereo
    if (0 == strcmp(argv[1], "stereo_depthAI_online")) {
        stereo_depthAI_online(argc, &argv[1]);
    }
    // Stereo-Inertial


    else {
        printf("\nUsage: slam.exe testName testArguments");
        printf("\nAvailable tests as the following...");

        // Monocular
        printf("\n\nMonocular examples:");
        printf("\n  N/A");
  
        // Monocular-Inertial
        printf("\n\nMonocular-Inertial examples:");
        printf("\n  N/A");

        // Stereo
        printf("\n\nStereo:");
        printf("\n  stereo_depthAI_online path_to_vocabulary path_to_settings");
  
        // Stereo-Inertial
        printf("\n\nStereo-Inertial:");
        printf("\n  N/A");
  
        printf("\n");
    }

    return 0;
}

