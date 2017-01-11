/**
 * @file ImagePublisher.cpp
 * @brief 
 *	Implimentation of ImagePublisher
 * @author Iwase
 */
#include "RSInterface.hpp"

ImagePublisher::ImagePublisher(const int step_size, const std::string& topic_name, const std::string& encoding) : 
	initialized_(false),
	frame_available_(false),
	step_size_(step_size),
	latest_ts_(-1),
	topic_name_(topic_name),
	encoding_(encoding)
{};
void ImagePublisher::init()
{
	initialized_ = true;
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    pub_ = it.advertise(topic_name_.c_str(), 1);
};
rs::frame ImagePublisher::get_latest_frame()
{
	assert(initialized_);
	std::unique_lock<std::mutex> lock(frame_mutex_);
	frame_buf_ = std::move(latest_frame_);
	return std::move(frame_buf_);
};
cv::Mat ImagePublisher::get_latest_image()
{
	assert(initialized_);
	std::unique_lock<std::mutex> lock(frame_mutex_);
	return std::move(latest_image_);
};
void ImagePublisher::publish(rs::frame frame)
{
	assert(initialized_);
    // mutex to ensure only one frame per stream is processed at a time
	std::unique_lock<std::mutex> lock(frame_mutex_);
	double frame_ts = frame.get_timestamp();
	if (latest_ts_ != frame_ts)  // Publish frames only if its not duplicate
	{
		image_ = cv::Mat(frame.get_height(), frame.get_width(), cv_bridge::getCvType(encoding_));
		image_.data = (unsigned char *)(frame.get_data());
	
		// Publish stream only if there is at least one subscriber.
		if (pub_.getNumSubscribers() > 0)
		{
	    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding_.c_str(), image_).toImageMsg();
	    	// Publish timestamp to synchronize frames.
	    	msg->width = frame.get_width();
	    	msg->height = frame.get_height();
	    	msg->is_bigendian = false;
	    	msg->step = frame.get_width() * step_size_;
	    	pub_.publish(msg);
	    }
	}
	latest_ts_ = frame_ts;
	latest_frame_ = std::move(frame);
	frame_available_ = true;
};
void ImagePublisher::publish(int height, int width, const void* frame_data)
{
	assert(initialized_);
    // mutex to ensure only one frame per stream is processed at a time
	std::unique_lock<std::mutex> lock(frame_mutex_);
	image_ = cv::Mat(height, width, cv_bridge::getCvType(encoding_));
	image_.data = (unsigned char *)(frame_data);
	image_.copyTo(latest_image_);

	// Publish stream only if there is at least one subscriber.
	if (pub_.getNumSubscribers() > 0)
	{
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding_.c_str(), latest_image_).toImageMsg();
		// Publish timestamp to synchronize frames.
		msg->width = width;
		msg->height = height;
		msg->is_bigendian = false;
		msg->step = width * step_size_;
		pub_.publish(msg);
	}

};

constexpr char DeprojectService::def_service_name_[];

DeprojectService::DeprojectService(const float depth_scale_meters, const rs::intrinsics& z_intrinsic, ImagePublisher* image_publisher_ptr):
	initialized_(false),
	exit_(false),
	depth_scale_meters_(depth_scale_meters),
	z_intrinsic_(z_intrinsic),
	service_name_(def_service_name_),
	image_publisher_ptr_(image_publisher_ptr)
{};
DeprojectService::~DeprojectService(){
	// joint thread
	if(service_thread_.joinable()) service_thread_.join();

	std::cout << "DeprojectService stop" << std::endl;
}
void DeprojectService::start_service()
{
	initialized_ = true;
	// Start publish loop
	try {
		service_thread_ = std::thread( [this]{ wait_for_request(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}
};
void DeprojectService::wait_for_request()
{
    ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService(service_name_, &DeprojectService::service_callback, this);
	ros::spin();
};
bool DeprojectService::service_callback(robot_vision::Deproject::Request &req, robot_vision::Deproject::Response &res)
{
	assert(initialized_);
	assert(req.u >= 0 && req.v >= 0);

	cv::Mat image = image_publisher_ptr_->get_latest_image();

	assert(req.u < image.cols && req.v < image.rows);

	float depth_point[3] = {0, 0, 0};
	float color_point[3] = {0, 0, 0};
	float depth_pixel[2] = {static_cast<float>(req.u), static_cast<float>(req.v)};

	const uint16_t *image_depth16 = reinterpret_cast<const uint16_t *>(image.data); 
	float scaled_depth = static_cast<float>( image_depth16[ req.v * image.cols + req.u ] ) * depth_scale_meters_;
	// depth pixel -> 3d point in depth camera frame.
	rs_deproject_pixel_to_point(depth_point, &z_intrinsic_, depth_pixel, scaled_depth);
	
	res.x = color_point[0];
	res.y = color_point[1];
	res.z = color_point[2];

	cv::circle(image, cv::Point(req.u, req.v), 30, 0.0, 3, 8);
	
	cv::namedWindow("detected pos", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("detected pos", image);
	cv::moveWindow("detected pos", 100, 100);
	cv::waitKey(5000);
	cv::destroyWindow("detected pos");
	
	return true;
}; 