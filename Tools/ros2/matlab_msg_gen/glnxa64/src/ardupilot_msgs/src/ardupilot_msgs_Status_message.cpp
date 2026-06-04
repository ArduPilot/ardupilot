// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for ardupilot_msgs/Status
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
#include "ardupilot_msgs/msg/status.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_Status_common : public MATLABROS2MsgInterface<ardupilot_msgs::msg::Status> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_Status_common(){}
    virtual void copy_from_struct(ardupilot_msgs::msg::Status* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Status* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_Status_common::copy_from_struct(ardupilot_msgs::msg::Status* msg, const matlab::data::Struct& arr,
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
        //vehicle_type
        const matlab::data::TypedArray<uint8_t> vehicle_type_arr = arr["vehicle_type"];
        msg->vehicle_type = vehicle_type_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'vehicle_type' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'vehicle_type' is wrong type; expected a uint8.");
    }
    try {
        //armed
        const matlab::data::TypedArray<bool> armed_arr = arr["armed"];
        msg->armed = armed_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'armed' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'armed' is wrong type; expected a logical.");
    }
    try {
        //mode
        const matlab::data::TypedArray<uint8_t> mode_arr = arr["mode"];
        msg->mode = mode_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'mode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'mode' is wrong type; expected a uint8.");
    }
    try {
        //flying
        const matlab::data::TypedArray<bool> flying_arr = arr["flying"];
        msg->flying = flying_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'flying' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'flying' is wrong type; expected a logical.");
    }
    try {
        //external_control
        const matlab::data::TypedArray<bool> external_control_arr = arr["external_control"];
        msg->external_control = external_control_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'external_control' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'external_control' is wrong type; expected a logical.");
    }
    try {
        //failsafe
        const matlab::data::TypedArray<uint8_t> failsafe_arr = arr["failsafe"];
        size_t nelem = failsafe_arr.getNumberOfElements();
        	msg->failsafe.resize(nelem);
        	std::copy(failsafe_arr.begin(), failsafe_arr.begin()+nelem, msg->failsafe.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'failsafe' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'failsafe' is wrong type; expected a uint8.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_Status_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Status* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","APM_ROVER","APM_ARDUCOPTER","APM_ARDUPLANE","APM_ANTENNATRACKER","APM_UNKNOWN","APM_REPLAY","APM_ARDUSUB","APM_IOFIRMWARE","APM_AP_PERIPH","APM_AP_DAL_STANDALONE","APM_AP_BOOTLOADER","APM_BLIMP","APM_HELI","vehicle_type","armed","mode","flying","external_control","FS_RADIO","FS_BATTERY","FS_GCS","FS_EKF","failsafe"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/Status");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // APM_ROVER
    auto currentElement_APM_ROVER = (msg + ctr)->APM_ROVER;
    outArray[ctr]["APM_ROVER"] = factory.createScalar(currentElement_APM_ROVER);
    // APM_ARDUCOPTER
    auto currentElement_APM_ARDUCOPTER = (msg + ctr)->APM_ARDUCOPTER;
    outArray[ctr]["APM_ARDUCOPTER"] = factory.createScalar(currentElement_APM_ARDUCOPTER);
    // APM_ARDUPLANE
    auto currentElement_APM_ARDUPLANE = (msg + ctr)->APM_ARDUPLANE;
    outArray[ctr]["APM_ARDUPLANE"] = factory.createScalar(currentElement_APM_ARDUPLANE);
    // APM_ANTENNATRACKER
    auto currentElement_APM_ANTENNATRACKER = (msg + ctr)->APM_ANTENNATRACKER;
    outArray[ctr]["APM_ANTENNATRACKER"] = factory.createScalar(currentElement_APM_ANTENNATRACKER);
    // APM_UNKNOWN
    auto currentElement_APM_UNKNOWN = (msg + ctr)->APM_UNKNOWN;
    outArray[ctr]["APM_UNKNOWN"] = factory.createScalar(currentElement_APM_UNKNOWN);
    // APM_REPLAY
    auto currentElement_APM_REPLAY = (msg + ctr)->APM_REPLAY;
    outArray[ctr]["APM_REPLAY"] = factory.createScalar(currentElement_APM_REPLAY);
    // APM_ARDUSUB
    auto currentElement_APM_ARDUSUB = (msg + ctr)->APM_ARDUSUB;
    outArray[ctr]["APM_ARDUSUB"] = factory.createScalar(currentElement_APM_ARDUSUB);
    // APM_IOFIRMWARE
    auto currentElement_APM_IOFIRMWARE = (msg + ctr)->APM_IOFIRMWARE;
    outArray[ctr]["APM_IOFIRMWARE"] = factory.createScalar(currentElement_APM_IOFIRMWARE);
    // APM_AP_PERIPH
    auto currentElement_APM_AP_PERIPH = (msg + ctr)->APM_AP_PERIPH;
    outArray[ctr]["APM_AP_PERIPH"] = factory.createScalar(currentElement_APM_AP_PERIPH);
    // APM_AP_DAL_STANDALONE
    auto currentElement_APM_AP_DAL_STANDALONE = (msg + ctr)->APM_AP_DAL_STANDALONE;
    outArray[ctr]["APM_AP_DAL_STANDALONE"] = factory.createScalar(currentElement_APM_AP_DAL_STANDALONE);
    // APM_AP_BOOTLOADER
    auto currentElement_APM_AP_BOOTLOADER = (msg + ctr)->APM_AP_BOOTLOADER;
    outArray[ctr]["APM_AP_BOOTLOADER"] = factory.createScalar(currentElement_APM_AP_BOOTLOADER);
    // APM_BLIMP
    auto currentElement_APM_BLIMP = (msg + ctr)->APM_BLIMP;
    outArray[ctr]["APM_BLIMP"] = factory.createScalar(currentElement_APM_BLIMP);
    // APM_HELI
    auto currentElement_APM_HELI = (msg + ctr)->APM_HELI;
    outArray[ctr]["APM_HELI"] = factory.createScalar(currentElement_APM_HELI);
    // vehicle_type
    auto currentElement_vehicle_type = (msg + ctr)->vehicle_type;
    outArray[ctr]["vehicle_type"] = factory.createScalar(currentElement_vehicle_type);
    // armed
    auto currentElement_armed = (msg + ctr)->armed;
    outArray[ctr]["armed"] = factory.createScalar(currentElement_armed);
    // mode
    auto currentElement_mode = (msg + ctr)->mode;
    outArray[ctr]["mode"] = factory.createScalar(currentElement_mode);
    // flying
    auto currentElement_flying = (msg + ctr)->flying;
    outArray[ctr]["flying"] = factory.createScalar(currentElement_flying);
    // external_control
    auto currentElement_external_control = (msg + ctr)->external_control;
    outArray[ctr]["external_control"] = factory.createScalar(currentElement_external_control);
    // FS_RADIO
    auto currentElement_FS_RADIO = (msg + ctr)->FS_RADIO;
    outArray[ctr]["FS_RADIO"] = factory.createScalar(currentElement_FS_RADIO);
    // FS_BATTERY
    auto currentElement_FS_BATTERY = (msg + ctr)->FS_BATTERY;
    outArray[ctr]["FS_BATTERY"] = factory.createScalar(currentElement_FS_BATTERY);
    // FS_GCS
    auto currentElement_FS_GCS = (msg + ctr)->FS_GCS;
    outArray[ctr]["FS_GCS"] = factory.createScalar(currentElement_FS_GCS);
    // FS_EKF
    auto currentElement_FS_EKF = (msg + ctr)->FS_EKF;
    outArray[ctr]["FS_EKF"] = factory.createScalar(currentElement_FS_EKF);
    // failsafe
    auto currentElement_failsafe = (msg + ctr)->failsafe;
    outArray[ctr]["failsafe"] = factory.createArray<ardupilot_msgs::msg::Status::_failsafe_type::const_iterator, uint8_t>({currentElement_failsafe.size(), 1}, currentElement_failsafe.begin(), currentElement_failsafe.end());
    }
    return std::move(outArray);
  } 
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_Status_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_ardupilot_msgs_Status_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_ardupilot_msgs_Status_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::msg::Status,ros2_ardupilot_msgs_msg_Status_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_ardupilot_msgs_Status_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::msg::Status,ros2_ardupilot_msgs_msg_Status_common>>();
  }
  std::shared_ptr<void> ros2_ardupilot_msgs_Status_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<ardupilot_msgs::msg::Status>();
    ros2_ardupilot_msgs_msg_Status_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_ardupilot_msgs_Status_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_ardupilot_msgs_msg_Status_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (ardupilot_msgs::msg::Status*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_Status_common, MATLABROS2MsgInterface<ardupilot_msgs::msg::Status>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_Status_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER