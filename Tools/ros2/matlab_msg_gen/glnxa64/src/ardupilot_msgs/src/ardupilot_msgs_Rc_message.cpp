// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for ardupilot_msgs/Rc
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
#include "ardupilot_msgs/msg/rc.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_msg_Rc_common : public MATLABROS2MsgInterface<ardupilot_msgs::msg::Rc> {
  public:
    virtual ~ros2_ardupilot_msgs_msg_Rc_common(){}
    virtual void copy_from_struct(ardupilot_msgs::msg::Rc* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Rc* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_ardupilot_msgs_msg_Rc_common::copy_from_struct(ardupilot_msgs::msg::Rc* msg, const matlab::data::Struct& arr,
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
        //is_connected
        const matlab::data::TypedArray<bool> is_connected_arr = arr["is_connected"];
        msg->is_connected = is_connected_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'is_connected' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'is_connected' is wrong type; expected a logical.");
    }
    try {
        //receiver_rssi
        const matlab::data::TypedArray<uint8_t> receiver_rssi_arr = arr["receiver_rssi"];
        msg->receiver_rssi = receiver_rssi_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'receiver_rssi' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'receiver_rssi' is wrong type; expected a uint8.");
    }
    try {
        //channels
        const matlab::data::TypedArray<int16_t> channels_arr = arr["channels"];
        size_t nelem = std::min<size_t>(32,channels_arr.getNumberOfElements());
        	msg->channels.resize(nelem);
        	std::copy(channels_arr.begin(), channels_arr.begin()+nelem, msg->channels.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'channels' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'channels' is wrong type; expected a int16.");
    }
    try {
        //active_overrides
        const matlab::data::TypedArray<bool> active_overrides_arr = arr["active_overrides"];
        size_t nelem = std::min<size_t>(32,active_overrides_arr.getNumberOfElements());
        	msg->active_overrides.resize(nelem);
        	std::copy(active_overrides_arr.begin(), active_overrides_arr.begin()+nelem, msg->active_overrides.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'active_overrides' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'active_overrides' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_ardupilot_msgs_msg_Rc_common::get_arr(MDFactory_T& factory, const ardupilot_msgs::msg::Rc* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","is_connected","receiver_rssi","channels","active_overrides"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardupilot_msgs/Rc");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // is_connected
    auto currentElement_is_connected = (msg + ctr)->is_connected;
    outArray[ctr]["is_connected"] = factory.createScalar(currentElement_is_connected);
    // receiver_rssi
    auto currentElement_receiver_rssi = (msg + ctr)->receiver_rssi;
    outArray[ctr]["receiver_rssi"] = factory.createScalar(currentElement_receiver_rssi);
    // channels
    auto currentElement_channels = (msg + ctr)->channels;
    outArray[ctr]["channels"] = factory.createArray<ardupilot_msgs::msg::Rc::_channels_type::const_iterator, int16_t>({currentElement_channels.size(), 1}, currentElement_channels.begin(), currentElement_channels.end());
    // active_overrides
    auto currentElement_active_overrides = (msg + ctr)->active_overrides;
    outArray[ctr]["active_overrides"] = factory.createArray<ardupilot_msgs::msg::Rc::_active_overrides_type::const_iterator, bool>({currentElement_active_overrides.size(), 1}, currentElement_active_overrides.begin(), currentElement_active_overrides.end());
    }
    return std::move(outArray);
  } 
class ARDUPILOT_MSGS_EXPORT ros2_ardupilot_msgs_Rc_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_ardupilot_msgs_Rc_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_ardupilot_msgs_Rc_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<ardupilot_msgs::msg::Rc,ros2_ardupilot_msgs_msg_Rc_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_ardupilot_msgs_Rc_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<ardupilot_msgs::msg::Rc,ros2_ardupilot_msgs_msg_Rc_common>>();
  }
  std::shared_ptr<void> ros2_ardupilot_msgs_Rc_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<ardupilot_msgs::msg::Rc>();
    ros2_ardupilot_msgs_msg_Rc_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_ardupilot_msgs_Rc_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_ardupilot_msgs_msg_Rc_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (ardupilot_msgs::msg::Rc*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_msg_Rc_common, MATLABROS2MsgInterface<ardupilot_msgs::msg::Rc>)
CLASS_LOADER_REGISTER_CLASS(ros2_ardupilot_msgs_Rc_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER