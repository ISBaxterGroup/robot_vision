/**
 * @file rs_controller
 * @brief Node for using realsense-camera through ROS.
 * @author Iwase
 * @date 2016.01.07.
 */

#include "RSInterface.hpp"

int main(int argc, char **argv) try
{
    ros::init(argc, argv, "rs_controller");
    ros::NodeHandle n;

    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if (ctx.get_device_count() == 0) return EXIT_FAILURE;

    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    const auto streams = 3;

    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    try { dev->enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}

  	float depth_scale_meters = dev->get_depth_scale();
    rs::intrinsics z_intrinsic = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics z_extrinsic = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	
	ImagePublisher ip_depth(sizeof(uint16_t), "rs_camera/image_depth", "mono16");
	ImagePublisher ip_color(sizeof(unsigned char) * 3, "rs_camera/image_color", "rgb8");
	ImagePublisher ip_color_aligned_to_depth(sizeof(unsigned char) * 3, "rs_camera/image_color_aligned_to_depth", "rgb8");
    ImagePublisher ip_depth_aligned_to_color(sizeof(unsigned char) * 3, "rs_camera/image_depth_aligned_to_color", "mono16");

    DeprojectService deproject_service(depth_scale_meters, z_intrinsic, z_extrinsic, &ip_depth);

	// start all service
    dev->start();
	deproject_service.start_service();
	ip_depth.init();
	ip_color.init();
	ip_color_aligned_to_depth.init();
    ip_depth_aligned_to_color.init();

    // publish functions
	std::function<void(int, int, const void*)> depth_publish = std::bind(static_cast<void (ImagePublisher::*)(int, int, const void*)>
		(&ImagePublisher::publish), &ip_depth, 
		std::placeholders::_1, 
		std::placeholders::_2, 
		std::placeholders::_3);
	std::function<void(int, int, const void*)> color_publish = std::bind(static_cast<void (ImagePublisher::*)(int, int, const void*)>
		(&ImagePublisher::publish), &ip_color, 
		std::placeholders::_1, 
		std::placeholders::_2, 
		std::placeholders::_3);
	std::function<void(int, int, const void*)> color_aligned_to_depth_publish = std::bind(static_cast<void (ImagePublisher::*)(int, int, const void*)>
		(&ImagePublisher::publish), 
		&ip_color_aligned_to_depth, 
		std::placeholders::_1, 
		std::placeholders::_2, 
		std::placeholders::_3);
    std::function<void(int, int, const void*)> depth_aligned_to_color_publish = std::bind(static_cast<void (ImagePublisher::*)(int, int, const void*)>
        (&ImagePublisher::publish), 
        &ip_depth_aligned_to_color, 
        std::placeholders::_1, 
        std::placeholders::_2, 
        std::placeholders::_3);
    
    ros::Rate loop_rate(10);
    while(n.ok())
    {
    	dev->wait_for_frames();
    	
    	depth_publish(dev->get_stream_height(rs::stream::depth), 
    								   dev->get_stream_width(rs::stream::depth), 
    								   dev->get_frame_data(rs::stream::depth));
		
    	color_publish(dev->get_stream_height(rs::stream::color), 
    								   dev->get_stream_width(rs::stream::color), 
    								   dev->get_frame_data(rs::stream::color));
    	
    	color_aligned_to_depth_publish(dev->get_stream_height(rs::stream::color_aligned_to_depth), 
    								   dev->get_stream_width(rs::stream::color_aligned_to_depth), 
    								   dev->get_frame_data(rs::stream::color_aligned_to_depth));
		
        depth_aligned_to_color_publish(dev->get_stream_height(rs::stream::depth_aligned_to_color), 
                                       dev->get_stream_width(rs::stream::depth_aligned_to_color), 
                                       dev->get_frame_data(rs::stream::depth_aligned_to_color));
    	loop_rate.sleep();
    }
    dev->stop();

    return EXIT_SUCCESS;
}
catch (const rs::error & e)
{
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}

