// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for ardupilot_msgs/GlobalPosition
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "ardupilot_msgs/msg/global_position.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_GlobalPosition_common : public MATLABROS2MsgInterface<ardupilot_msgs::msg::GlobalPosition> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_GlobalPosition_common(){}
    virtual void copy_from_struct(ardupilot_msgs::msg::GlobalPosition* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::GlobalPosition* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_GlobalPosition_common::copy_from_struct(ardupilot_msgs::msg::GlobalPosition* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'header' is wrong type; expected a struct.");
    }
    try {
        //coordinate_frame
        const matlab::data::TypedArray<uint8_t> coordinate_frame_arr = arr["coordinate_frame"];
        msg->coordinate_frame = coordinate_frame_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'coordinate_frame' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'coordinate_frame' is wrong type; expected a uint8.");
    }
    try {
        //type_mask
        const matlab::data::TypedArray<uint16_t> type_mask_arr = arr["type_mask"];
        msg->type_mask = type_mask_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'type_mask' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'type_mask' is wrong type; expected a uint16.");
    }
    try {
        //latitude
        const matlab::data::TypedArray<double> latitude_arr = arr["latitude"];
        msg->latitude = latitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'latitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'latitude' is wrong type; expected a double.");
    }
    try {
        //longitude
        const matlab::data::TypedArray<double> longitude_arr = arr["longitude"];
        msg->longitude = longitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'longitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'longitude' is wrong type; expected a double.");
    }
    try {
        //altitude
        const matlab::data::TypedArray<float> altitude_arr = arr["altitude"];
        msg->altitude = altitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'altitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'altitude' is wrong type; expected a single.");
    }
    try {
        //velocity
        const matlab::data::StructArray velocity_arr = arr["velocity"];
        auto msgClassPtr_velocity = getCommonObject<geometry_msgs::msg::Twist>("ros2_geometry_msgs_msg_Twist_common",loader);
        msgClassPtr_velocity->copy_from_struct(&msg->velocity,velocity_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'velocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'velocity' is wrong type; expected a struct.");
    }
    try {
        //acceleration_or_force
        const matlab::data::StructArray acceleration_or_force_arr = arr["acceleration_or_force"];
        auto msgClassPtr_acceleration_or_force = getCommonObject<geometry_msgs::msg::Twist>("ros2_geometry_msgs_msg_Twist_common",loader);
        msgClassPtr_acceleration_or_force->copy_from_struct(&msg->acceleration_or_force,acceleration_or_force_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'acceleration_or_force' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'acceleration_or_force' is wrong type; expected a struct.");
    }
    try {
        //yaw
        const matlab::data::TypedArray<float> yaw_arr = arr["yaw"];
        msg->yaw = yaw_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'yaw' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'yaw' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_GlobalPosition_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::GlobalPosition* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","coordinate_frame","FRAME_GLOBAL_INT","FRAME_GLOBAL_REL_ALT","FRAME_GLOBAL_TERRAIN_ALT","type_mask","IGNORE_LATITUDE","IGNORE_LONGITUDE","IGNORE_ALTITUDE","IGNORE_VX","IGNORE_VY","IGNORE_VZ","IGNORE_AFX","IGNORE_AFY","IGNORE_AFZ","FORCE","IGNORE_YAW","IGNORE_YAW_RATE","latitude","longitude","altitude","velocity","acceleration_or_force","yaw"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/GlobalPosition");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // coordinate_frame
    auto currentElement_coordinate_frame = (msg + ctr)->coordinate_frame;
    outArray[ctr]["coordinate_frame"] = factory.createScalar(currentElement_coordinate_frame);
    // FRAME_GLOBAL_INT
    auto currentElement_FRAME_GLOBAL_INT = (msg + ctr)->FRAME_GLOBAL_INT;
    outArray[ctr]["FRAME_GLOBAL_INT"] = factory.createScalar(currentElement_FRAME_GLOBAL_INT);
    // FRAME_GLOBAL_REL_ALT
    auto currentElement_FRAME_GLOBAL_REL_ALT = (msg + ctr)->FRAME_GLOBAL_REL_ALT;
    outArray[ctr]["FRAME_GLOBAL_REL_ALT"] = factory.createScalar(currentElement_FRAME_GLOBAL_REL_ALT);
    // FRAME_GLOBAL_TERRAIN_ALT
    auto currentElement_FRAME_GLOBAL_TERRAIN_ALT = (msg + ctr)->FRAME_GLOBAL_TERRAIN_ALT;
    outArray[ctr]["FRAME_GLOBAL_TERRAIN_ALT"] = factory.createScalar(currentElement_FRAME_GLOBAL_TERRAIN_ALT);
    // type_mask
    auto currentElement_type_mask = (msg + ctr)->type_mask;
    outArray[ctr]["type_mask"] = factory.createScalar(currentElement_type_mask);
    // IGNORE_LATITUDE
    auto currentElement_IGNORE_LATITUDE = (msg + ctr)->IGNORE_LATITUDE;
    outArray[ctr]["IGNORE_LATITUDE"] = factory.createScalar(currentElement_IGNORE_LATITUDE);
    // IGNORE_LONGITUDE
    auto currentElement_IGNORE_LONGITUDE = (msg + ctr)->IGNORE_LONGITUDE;
    outArray[ctr]["IGNORE_LONGITUDE"] = factory.createScalar(currentElement_IGNORE_LONGITUDE);
    // IGNORE_ALTITUDE
    auto currentElement_IGNORE_ALTITUDE = (msg + ctr)->IGNORE_ALTITUDE;
    outArray[ctr]["IGNORE_ALTITUDE"] = factory.createScalar(currentElement_IGNORE_ALTITUDE);
    // IGNORE_VX
    auto currentElement_IGNORE_VX = (msg + ctr)->IGNORE_VX;
    outArray[ctr]["IGNORE_VX"] = factory.createScalar(currentElement_IGNORE_VX);
    // IGNORE_VY
    auto currentElement_IGNORE_VY = (msg + ctr)->IGNORE_VY;
    outArray[ctr]["IGNORE_VY"] = factory.createScalar(currentElement_IGNORE_VY);
    // IGNORE_VZ
    auto currentElement_IGNORE_VZ = (msg + ctr)->IGNORE_VZ;
    outArray[ctr]["IGNORE_VZ"] = factory.createScalar(currentElement_IGNORE_VZ);
    // IGNORE_AFX
    auto currentElement_IGNORE_AFX = (msg + ctr)->IGNORE_AFX;
    outArray[ctr]["IGNORE_AFX"] = factory.createScalar(currentElement_IGNORE_AFX);
    // IGNORE_AFY
    auto currentElement_IGNORE_AFY = (msg + ctr)->IGNORE_AFY;
    outArray[ctr]["IGNORE_AFY"] = factory.createScalar(currentElement_IGNORE_AFY);
    // IGNORE_AFZ
    auto currentElement_IGNORE_AFZ = (msg + ctr)->IGNORE_AFZ;
    outArray[ctr]["IGNORE_AFZ"] = factory.createScalar(currentElement_IGNORE_AFZ);
    // FORCE
    auto currentElement_FORCE = (msg + ctr)->FORCE;
    outArray[ctr]["FORCE"] = factory.createScalar(currentElement_FORCE);
    // IGNORE_YAW
    auto currentElement_IGNORE_YAW = (msg + ctr)->IGNORE_YAW;
    outArray[ctr]["IGNORE_YAW"] = factory.createScalar(currentElement_IGNORE_YAW);
    // IGNORE_YAW_RATE
    auto currentElement_IGNORE_YAW_RATE = (msg + ctr)->IGNORE_YAW_RATE;
    outArray[ctr]["IGNORE_YAW_RATE"] = factory.createScalar(currentElement_IGNORE_YAW_RATE);
    // latitude
    auto currentElement_latitude = (msg + ctr)->latitude;
    outArray[ctr]["latitude"] = factory.createScalar(currentElement_latitude);
    // longitude
    auto currentElement_longitude = (msg + ctr)->longitude;
    outArray[ctr]["longitude"] = factory.createScalar(currentElement_longitude);
    // altitude
    auto currentElement_altitude = (msg + ctr)->altitude;
    outArray[ctr]["altitude"] = factory.createScalar(currentElement_altitude);
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    auto msgClassPtr_velocity = getCommonObject<geometry_msgs::msg::Twist>("ros2_geometry_msgs_msg_Twist_common",loader);
    outArray[ctr]["velocity"] = msgClassPtr_velocity->get_arr(factory, &currentElement_velocity, loader);
    // acceleration_or_force
    auto currentElement_acceleration_or_force = (msg + ctr)->acceleration_or_force;
    auto msgClassPtr_acceleration_or_force = getCommonObject<geometry_msgs::msg::Twist>("ros2_geometry_msgs_msg_Twist_common",loader);
    outArray[ctr]["acceleration_or_force"] = msgClassPtr_acceleration_or_force->get_arr(factory, &currentElement_acceleration_or_force, loader);
    // yaw
    auto currentElement_yaw = (msg + ctr)->yaw;
    outArray[ctr]["yaw"] = factory.createScalar(currentElement_yaw);
    }
    return std::move(outArray);
  } 
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_GlobalPosition_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_ardupilot_msgs_GlobalPosition_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_ardupilot_msgs_GlobalPosition_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::msg::GlobalPosition,ros2_ardupilot_msgs_msg_GlobalPosition_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_ardupilot_msgs_GlobalPosition_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::msg::GlobalPosition,ros2_ardupilot_msgs_msg_GlobalPosition_common>>();
  }
  std::shared_ptr<void> ros2_ardupilot_msgs_GlobalPosition_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<ardupilot_msgs::msg::GlobalPosition>();
    ros2_ardupilot_msgs_msg_GlobalPosition_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_ardupilot_msgs_GlobalPosition_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_ardupilot_msgs_msg_GlobalPosition_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (ardupilot_msgs::msg::GlobalPosition*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_GlobalPosition_common, MATLABROS2MsgInterface<ardupilot_msgs::msg::GlobalPosition>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_GlobalPosition_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER