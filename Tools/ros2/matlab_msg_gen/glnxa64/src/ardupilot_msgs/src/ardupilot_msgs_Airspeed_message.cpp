// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for ardupilot_msgs/Airspeed
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
#include "ardupilot_msgs/msg/airspeed.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_Airspeed_common : public MATLABROS2MsgInterface<ardupilot_msgs::msg::Airspeed> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_Airspeed_common(){}
    virtual void copy_from_struct(ardupilot_msgs::msg::Airspeed* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Airspeed* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_Airspeed_common::copy_from_struct(ardupilot_msgs::msg::Airspeed* msg, const matlab::data::Struct& arr,
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
        //true_airspeed
        const matlab::data::StructArray true_airspeed_arr = arr["true_airspeed"];
        auto msgClassPtr_true_airspeed = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_true_airspeed->copy_from_struct(&msg->true_airspeed,true_airspeed_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'true_airspeed' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'true_airspeed' is wrong type; expected a struct.");
    }
    try {
        //eas_2_tas
        const matlab::data::TypedArray<float> eas_2_tas_arr = arr["eas_2_tas"];
        msg->eas_2_tas = eas_2_tas_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'eas_2_tas' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'eas_2_tas' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_Airspeed_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Airspeed* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","true_airspeed","eas_2_tas"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/Airspeed");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // true_airspeed
    auto currentElement_true_airspeed = (msg + ctr)->true_airspeed;
    auto msgClassPtr_true_airspeed = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["true_airspeed"] = msgClassPtr_true_airspeed->get_arr(factory, &currentElement_true_airspeed, loader);
    // eas_2_tas
    auto currentElement_eas_2_tas = (msg + ctr)->eas_2_tas;
    outArray[ctr]["eas_2_tas"] = factory.createScalar(currentElement_eas_2_tas);
    }
    return std::move(outArray);
  } 
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_Airspeed_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_ardupilot_msgs_Airspeed_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_ardupilot_msgs_Airspeed_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::msg::Airspeed,ros2_ardupilot_msgs_msg_Airspeed_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_ardupilot_msgs_Airspeed_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::msg::Airspeed,ros2_ardupilot_msgs_msg_Airspeed_common>>();
  }
  std::shared_ptr<void> ros2_ardupilot_msgs_Airspeed_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<ardupilot_msgs::msg::Airspeed>();
    ros2_ardupilot_msgs_msg_Airspeed_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_ardupilot_msgs_Airspeed_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_ardupilot_msgs_msg_Airspeed_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (ardupilot_msgs::msg::Airspeed*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_Airspeed_common, MATLABROS2MsgInterface<ardupilot_msgs::msg::Airspeed>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_Airspeed_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER