// Copyright 2021-2022 The MathWorks, Inc.
// Common copy functions for ardupilot_msgs/TakeoffRequest
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
#include "ardupilot_msgs/srv/takeoff.hpp"
#include "visibility_control.h"
#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
#include "ROS2ServiceTemplates.hpp"
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_TakeoffRequest_common : public MATLABROS2MsgInterface<ardupilot_msgs::srv::Takeoff::Request> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_TakeoffRequest_common(){}
    virtual void copy_from_struct(ardupilot_msgs::srv::Takeoff::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::srv::Takeoff::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_TakeoffRequest_common::copy_from_struct(ardupilot_msgs::srv::Takeoff::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //alt
        const matlab::data::TypedArray<float> alt_arr = arr["alt"];
        msg->alt = alt_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'alt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'alt' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_TakeoffRequest_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::srv::Takeoff::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","alt"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/TakeoffRequest");
    // alt
    auto currentElement_alt = (msg + ctr)->alt;
    outArray[ctr]["alt"] = factory.createScalar(currentElement_alt);
    }
    return std::move(outArray);
  }
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_TakeoffResponse_common : public MATLABROS2MsgInterface<ardupilot_msgs::srv::Takeoff::Response> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_TakeoffResponse_common(){}
    virtual void copy_from_struct(ardupilot_msgs::srv::Takeoff::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::srv::Takeoff::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_TakeoffResponse_common::copy_from_struct(ardupilot_msgs::srv::Takeoff::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //status
        const matlab::data::TypedArray<bool> status_arr = arr["status"];
        msg->status = status_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'status' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'status' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_TakeoffResponse_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::srv::Takeoff::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","status"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/TakeoffResponse");
    // status
    auto currentElement_status = (msg + ctr)->status;
    outArray[ctr]["status"] = factory.createScalar(currentElement_status);
    }
    return std::move(outArray);
  } 
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_Takeoff_service : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_ardupilot_msgs_Takeoff_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_ardupilot_msgs_Takeoff_service::generatePublisherInterface(ElementType type){
    if(type == eRequest){
        return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::srv::Takeoff::Request,ros2_ardupilot_msgs_msg_TakeoffRequest_common>>();
    }else if(type == eResponse){
        return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::srv::Takeoff::Response,ros2_ardupilot_msgs_msg_TakeoffResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          ros2_ardupilot_msgs_Takeoff_service::generateSubscriberInterface(ElementType type){
    if(type == eRequest){
        return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::srv::Takeoff::Request,ros2_ardupilot_msgs_msg_TakeoffRequest_common>>();
    }else if(type == eResponse){
        return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::srv::Takeoff::Response,ros2_ardupilot_msgs_msg_TakeoffResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
  }
  std::shared_ptr<void> ros2_ardupilot_msgs_Takeoff_service::generateCppMessage(ElementType type, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    if(type == eRequest){
        auto msg = std::make_shared<ardupilot_msgs::srv::Takeoff::Request>();
        ros2_ardupilot_msgs_msg_TakeoffRequest_common commonObj;
        commonObj.mCommonObjMap = commonObjMap;
        commonObj.copy_from_struct(msg.get(), arr[0], loader);
        return msg;
    }else if(type == eResponse){
        auto msg = std::make_shared<ardupilot_msgs::srv::Takeoff::Response>();
        ros2_ardupilot_msgs_msg_TakeoffResponse_common commonObj;
        commonObj.mCommonObjMap = commonObjMap;
        commonObj.copy_from_struct(msg.get(), arr[0], loader);
        return msg;
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          ros2_ardupilot_msgs_Takeoff_service::generateSvcServerInterface(){
    return std::make_shared<ROS2SvcServerImpl<ardupilot_msgs::srv::Takeoff,ardupilot_msgs::srv::Takeoff::Request,ardupilot_msgs::srv::Takeoff::Response,ros2_ardupilot_msgs_msg_TakeoffRequest_common,ros2_ardupilot_msgs_msg_TakeoffResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          ros2_ardupilot_msgs_Takeoff_service::generateSvcClientInterface(){
    return std::make_shared<ROS2SvcClientImpl<ardupilot_msgs::srv::Takeoff,ardupilot_msgs::srv::Takeoff::Request,ardupilot_msgs::srv::Takeoff::Response,ros2_ardupilot_msgs_msg_TakeoffRequest_common,ros2_ardupilot_msgs_msg_TakeoffResponse_common,rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedFuture>>();
  }
  matlab::data::StructArray ros2_ardupilot_msgs_Takeoff_service::generateMLMessage(ElementType type, 
                                                    void*  msgPtr ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    if(type == eRequest){
	    ros2_ardupilot_msgs_msg_TakeoffRequest_common commonObj;	
        commonObj.mCommonObjMap = commonObjMap;
	    MDFactory_T factory;
	    return commonObj.get_arr(factory, (ardupilot_msgs::srv::Takeoff::Request*)msgPtr, loader);
    }else if(type == eResponse){
        ros2_ardupilot_msgs_msg_TakeoffResponse_common commonObj;	
        commonObj.mCommonObjMap = commonObjMap;	
	    MDFactory_T factory;
	    return commonObj.get_arr(factory, (ardupilot_msgs::srv::Takeoff::Response*)msgPtr, loader);
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_TakeoffRequest_common, MATLABROS2MsgInterface<ardupilot_msgs::srv::Takeoff::Request>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_TakeoffResponse_common, MATLABROS2MsgInterface<ardupilot_msgs::srv::Takeoff::Response>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_Takeoff_service, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
