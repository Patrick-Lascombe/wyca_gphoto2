#include <wyca_gphoto2/wyca_gphoto2.h>

WycaGphoto2::WycaGphoto2() {
    ros::NodeHandle nh_priv("~");

    //Camera filters
    nh_priv.getParam("owner", owner_);

    //Camera configs
    nh_priv.getParam("shutter_speed_mode", shutter_speed_mode_);
    nh_priv.getParam("aperture_mode", aperture_mode_);
    nh_priv.getParam("iso_mode", iso_mode_);
}

WycaGphoto2::~WycaGphoto2() {

}

bool WycaGphoto2::instantiateCameraObject() {
    if(context_ == NULL) {
        context_ = gp_context_new();
        gp_context_set_error_func( context_, WycaGphoto2::contextError, NULL );

    }

    if(gp_camera_new(&camera_) != GP_OK) {
        ROS_ERROR("Error executing : gp_camera_new()");
    }

}

bool WycaGphoto2::autoDetect() {
    // Creating camera list
    if(gp_list_new(&camera_list_) != GP_OK) {
        ROS_ERROR("Error executing : gp_list_new() in autoDetect");
        return false;
    }

    // List all ports connected
    if(gp_port_info_list_new(&port_info_list_) != GP_OK) {
        ROS_ERROR("Error executing : gp_port_info_list_new() in autoDetect");
        return false;
    }

    if(gp_port_info_list_load(port_info_list_) != GP_OK) {
        ROS_ERROR("Error executing : gp_port_info_list_load() in autoDetect");
        return false;
    }

    int port_count =  gp_port_info_list_count( port_info_list_ );
    if( port_count < GP_OK )
    {
        ROS_ERROR("Error executing : gp_port_info_list_count() in autoDetect");
        return false;
    }

    // Create a new abilities list
    if( gp_abilities_list_new( &abilities_list_ ) != GP_OK )
    {
        ROS_ERROR("Error executing : gp_abilities_list_new() in autoDetect");
         return false;
    }

    // Populate the abilities list
    if( gp_abilities_list_load( abilities_list_, context_ ) != GP_OK )
    {
        ROS_ERROR("Error executing : gp_abilities_list_load() in autoDetect");
        return false;
    }

}

bool WycaGphoto2::connectToCamera() {
    if(gp_list_new(&camera_list_) != GP_OK) {
        ROS_ERROR("Error executing : gp_list_new()");
    }

}

bool WycaGphoto2::setConfig(std::string param, std::string value) {
    CameraWidget *root;

//    gp_camera_get_config(camera_, )

    gp_camera_set_config(camera_, root, context_);

}

std::string WycaGphoto2::getConfig(std::string param) {
    CameraWidget **root, **child;
    gp_camera_get_config(camera_, root, context_);
    gp_widget_get_child_by_name(*root, param.c_str(), child);
}

int WycaGphoto2::findWidget(std::string name, CameraWidget **child,
                            CameraWidget **root) {
    int error_code;

    // Get camera configuration
    error_code = gp_camera_get_config( camera_, root, context_ );
    if (error_code != GP_OK)
    {
      ROS_ERROR("Error executing gp_camera_get_config");
      return error_code;
    }

    // Find child of configuration by name
    if( gp_widget_get_child_by_name( *root, name.c_str(), child ) == GP_OK )
    {
      return GP_OK;
    }

    // Find child of configuration  by label
    if( gp_widget_get_child_by_label( *root, name.c_str(), child ) == GP_OK )
    {
      return GP_OK;
    }

    // If full name is not found, search for last subname.
    // name delimeter is '/'
    size_t found_index = name.length();
    while( found_index == name.length() )
    {
      found_index = name.rfind( '/' );

      if( found_index == std::string::npos ) // No subname, we already failed this search above
      {
        gp_context_error( context_,"%s not found in configuration tree.", name.c_str() );
        gp_widget_free( *root );
        return GP_ERROR;
      }

      if( found_index == name.length() - 1 ) // end of string, cut it off
      {
        name = name.substr( 0, found_index );
      }
    }
    name = name.substr( found_index, name.length() - 1 );

    // Find child using
    if( gp_widget_get_child_by_name( *root, name.c_str(), child ) == GP_OK )
    {
      return GP_OK;
    }
    if( gp_widget_get_child_by_label( *root, name.c_str(), child ) == GP_OK )
    {
      return GP_OK;
    }

    // all matches have failed
    gp_context_error( context_, "%s not found in configuration tree.", name.c_str() );
    gp_widget_free( *root );
    return GP_ERROR;
}

void WycaGphoto2::contextError(GPContext *context, const char *error_string, void *data) {
    std::cerr << "\nphoto_reporter: Context error \n" << error_string << std::endl;

}

int main(int argc, char **argv)
{

}
