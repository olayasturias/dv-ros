// Generated by gencpp from file dynamic_reconfigure/Reconfigure.msg
// DO NOT EDIT!


#ifndef DYNAMIC_RECONFIGURE_MESSAGE_RECONFIGURE_H
#define DYNAMIC_RECONFIGURE_MESSAGE_RECONFIGURE_H

#include <ros/service_traits.h>

#include <dv_ros_messaging/messaging.hpp>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/ReconfigureResponse.h>


namespace dynamic_reconfigure
{

struct Reconfigure
	{

	typedef ReconfigureRequest_<boost::container::allocator<void>> Request;
	typedef ReconfigureResponse_<boost::container::allocator<void>> Response;
	Request request;
	Response response;

	typedef Request RequestType;
	typedef Response ResponseType;

	}; // struct Reconfigure
} // namespace dynamic_reconfigure


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamic_reconfigure::Reconfigure > {
	static const char* value()
	{
		return "bb125d226a21982a4a98760418dc2672";
	}

	static const char* value(const ::dynamic_reconfigure::Reconfigure&) { return value(); }
};

template<>
struct DataType< ::dynamic_reconfigure::Reconfigure > {
	static const char* value()
	{
		return "dynamic_reconfigure/Reconfigure";
	}

	static const char* value(const ::dynamic_reconfigure::Reconfigure&) { return value(); }
};


// service_traits::MD5Sum< ::dynamic_reconfigure::ReconfigureRequest> should match
// service_traits::MD5Sum< ::dynamic_reconfigure::Reconfigure >
template<>
struct MD5Sum<DV_ROS_MSGS(::dynamic_reconfigure::ReconfigureRequest)>
	{
	static const char* value()
	{
		return MD5Sum< ::dynamic_reconfigure::Reconfigure >::value();
	}
	static const char* value(const ::dynamic_reconfigure::ReconfigureRequest&)
	{
		return value();
	}
	};

// service_traits::DataType< ::dynamic_reconfigure::ReconfigureRequest> should match
// service_traits::DataType< ::dynamic_reconfigure::Reconfigure >
template<>
struct DataType< DV_ROS_MSGS(::dynamic_reconfigure::ReconfigureRequest)>
	{
	static const char* value()
	{
		return DataType< ::dynamic_reconfigure::Reconfigure >::value();
	}
	static const char* value(const ::dynamic_reconfigure::ReconfigureRequest&)
	{
		return value();
	}
	};

// service_traits::MD5Sum< ::dynamic_reconfigure::ReconfigureResponse> should match
// service_traits::MD5Sum< ::dynamic_reconfigure::Reconfigure >
template<>
struct MD5Sum< DV_ROS_MSGS(::dynamic_reconfigure::ReconfigureResponse)>
	{
	static const char* value()
	{
		return MD5Sum< ::dynamic_reconfigure::Reconfigure >::value();
	}
	static const char* value(const ::dynamic_reconfigure::ReconfigureResponse&)
	{
		return value();
	}
	};

// service_traits::DataType< ::dynamic_reconfigure::ReconfigureResponse> should match
// service_traits::DataType< ::dynamic_reconfigure::Reconfigure >
template<>
struct DataType< DV_ROS_MSGS(::dynamic_reconfigure::ReconfigureResponse)>
	{
	static const char* value()
	{
		return DataType< ::dynamic_reconfigure::Reconfigure >::value();
	}
	static const char* value(const ::dynamic_reconfigure::ReconfigureResponse&)
	{
		return value();
	}
	};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIC_RECONFIGURE_MESSAGE_RECONFIGURE_H
