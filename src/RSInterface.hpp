/**
 * @file RSInterface.hpp
 * @brief Tools for using realsense-camera through ROS.
 * @author Iwase
 * @date 2016.01.07.
 */
//----------------------------------------------------------
// Joint controller
//----------------------------------------------------------
#ifndef RS_INTERFACE_HPP
#define RS_INTERFACE_HPP
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <array>

#include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "robot_vision/Deproject.h"
/**
* @class ImagePublisher
* @brief Publish rs::frame to ros as sensor_msgs::ImagePtr.
*/
class ImagePublisher{
private:
	bool initialized_;
	bool frame_available_;
	int step_size_;
	double latest_ts_;
	std::mutex frame_mutex_;
    image_transport::Publisher pub_;

    rs::frame latest_frame_;
    rs::frame frame_buf_;
    cv::Mat image_;
    cv::Mat latest_image_;

    std::string topic_name_;
    std::string encoding_;

public:
	/**
	 * @brief A constructor 
	 */
	ImagePublisher(const int step_size, const std::string& topic_name, const std::string& encoding);
	/**
	 * @brief Initialize publisher
	 */
	void init();	
	/**
	 * @brief getter 
	 */
	rs::frame get_latest_frame();
	/**
	 * @brief getter 
	 */
	cv::Mat get_latest_image();
	/**
	 * @brief publish frame as imageptr
	 * @param frame Frame for publish
	 */
	void publish(rs::frame frame); 
	/**
	 * @brief publish frame_data as imageptr
	 * @param frame_data Frame for publish
	 */
	void publish(int height, int width, const void * frame_data); 
};

/**
* @class DeprojectService
* @brief Deproject pixel to point on Depth Camera.
*/
class DeprojectService{
public:
	static constexpr char def_service_name_ [] = "deproject";

private:
	bool initialized_;
	bool exit_;
	float depth_scale_meters_;
    rs::intrinsics z_intrinsic_;
    rs::extrinsics z_extrinsic_;
    std::string service_name_;

    ImagePublisher* image_publisher_ptr_;

	std::mutex mtx_;
	std::thread service_thread_;

public:
	/**
	 * @brief A constructor 
	 */
	DeprojectService(const float depth_scale_meters, const rs::intrinsics& z_intrinsic, const rs::extrinsics& depth_color_extrinsics, ImagePublisher* image_publisher_ptr);
	~DeprojectService();
	/**
	 * @brief Initialize and start service
	 */
	void start_service();

private:
	/**
	 * @brief wait loop
	 */
	void wait_for_request();
	/**
	 * @brief publish frame as imageptr
	 * @param frame Frame for publish
	 */
	bool service_callback(robot_vision::Deproject::Request  &req, robot_vision::Deproject::Response &res); 
};

#endif