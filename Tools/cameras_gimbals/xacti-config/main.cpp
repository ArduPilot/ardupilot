#include <stdio.h>
#include <iostream>
#include <cstring>
#include "CX-GBXXXCtrl.h"

const char* this_app_str = "xacti-config";

// display help
void display_help()
{
    printf("Usage: sudo %s option [value]\n", this_app_str);
    printf(" --dronecan\tenable (value=1) or disable (value=0) dronecan parsing\n");
    printf(" --format\tformat SD card\n");
    printf(" --help\t\tdisplay usage\n");
    printf(" --irpalette\t\tIR pallete (0:white hot, 1:black hot, 2:rainbow, 3:rainHC, 4:ironbow, 5:lava, 6:arctic, 7:glowbow, 8:graded fire, 9:hottest)\n");
    printf(" --msc\t\tchange to mass storage class mode (for downloading from SD card)\n");
}

int main(int argc, char **argv)
{
    // display help
    if ((argc <= 1) || ((argc >= 2) && (strcmp(argv[1], "--help") == 0))) {
        display_help();
        return 0;
    }

    // open camera
    CX_GBXXXCtrl camera_ctrl;
    if (!camera_ctrl.Open(NULL)) {
        printf("%s: failed to open camera\n", this_app_str);
        return 1;
    }

    // args_ok set to true when command line processed correctly
    bool args_ok = false;
    bool ret_ok = true;

    // enable DroneCAN
    if ((argc >= 3) && (strcmp(argv[1], "--dronecan") == 0)) {
        args_ok = true;
        uint8_t enable = (strcmp(argv[2], "1") == 0);
        ret_ok = camera_ctrl.SetCameraCtrl(0x07, 0x1e, &enable, sizeof(enable));
        const char* enable_or_disable_str = enable ? "enable" : "disable";
        if (ret_ok) {
            printf("%s: %s DroneCAN\n", this_app_str, enable_or_disable_str);
        } else {
            printf("%s: failed to %s DroneCAN\n", this_app_str, enable_or_disable_str);
        }
    }

    // format SD card
    if ((argc >= 2) && (strcmp(argv[1], "--format") == 0)) {
        args_ok = true;
        uint8_t format_sd = 0;
        ret_ok = camera_ctrl.SetCameraCtrl(0x07, 0x15, &format_sd, sizeof(format_sd));
        if (ret_ok) {
            printf("%s: formatted SD card\n", this_app_str);
        } else {
            printf("%s: failed format SD card\n", this_app_str);
        }
    }

    // IR palette
    if ((argc >= 3) && (strcmp(argv[1], "--irpalette") == 0)) {
        args_ok = true;
        int palette_int = 0;
        sscanf(argv[2], "%d", &palette_int);
        uint8_t palette_uint8 = (uint8_t)palette_int;
        ret_ok = camera_ctrl.SetCameraCtrl(0x07, 0x19, &palette_uint8, sizeof(palette_uint8));
        if (ret_ok) {
            printf("%s: IR palette set to %d\n", this_app_str, (int)palette_uint8);
        } else {
            printf("%s: failed to set IR palette to %d\n", this_app_str, (int)palette_uint8);
        }
    }

    // change to Mass Storage Mode to allow downloading of images and videos
    if ((argc >= 2) && (strcmp(argv[1], "--msc") == 0)) {
        args_ok = true;
        uint8_t msc_mode = 1;
        ret_ok = camera_ctrl.SetCameraCtrl(0x06, 0x07, &msc_mode, sizeof(msc_mode));
        if (ret_ok) {
            printf("%s: changed to mass storage mode\n", this_app_str);
        } else {
            printf("%s: failed to change to mass storage mode\n", this_app_str);
        }
    }

    // close camera
    camera_ctrl.Close();

    // display help if args could not be processed
    if (!args_ok) {
        display_help();
    }

    // return 0 if OK, 1 if not OK
    return ret_ok ? 0 : 1;
}
