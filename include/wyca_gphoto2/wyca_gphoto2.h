#ifndef WYCA_GPHOTO2_H
#define WYCA_GPHOTO2_H

//ROS includes
#include <ros/ros.h>

// ROS Services
#include <gphoto2_ros/GetConfig.h>
#include <gphoto2_ros/SetConfig.h>
#include <gphoto2_ros/Capture.h>
#include <gphoto2_ros/DownloadPictures.h>
#include <gphoto2_ros/GetPicturePathList.h>
#include <gphoto2_ros/DeletePictures.h>
#include <std_srvs/Trigger.h>

//ROS action
#include <actionlib/server/simple_action_server.h>
#include <gphoto2_ros/SetFocusAction.h>
#include <gphoto2_ros/TriggerAction.h>

//gphoto2 includes
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-context.h>
#include <gphoto2/gphoto2-setting.h>
#include <gphoto2/gphoto2-filesys.h>

//Others includes
#include <stdio.h>
#include <libusb-1.0/libusb.h>

#include <experimental/filesystem>

class WycaGphoto2 {
protected:
    ros::NodeHandle nh_;
    Camera* camera_;
    GPContext* context_;
    GPPortInfo port_info_;
    CameraAbilities abilities_;
    CameraList* camera_list_;
    GPPortInfoList* port_info_list_;
    CameraAbilitiesList* abilities_list_;

    std::string owner_;
    std::string shutter_speed_mode_;
    std::string aperture_mode_;
    std::string iso_mode_;

public:
    WycaGphoto2();
    ~WycaGphoto2();
    bool connectToCamera();
    bool setConfig(std::string param, std::string value);
    bool instantiateCameraObject();
    bool autoDetect();
    std::string getConfig(std::string param);
    static void contextError( GPContext* context, const char* error_string, void* data );
    int findWidget(std::string name, CameraWidget **child, CameraWidget **root);
};

#endif // WYCA_GPHOTO2_H
