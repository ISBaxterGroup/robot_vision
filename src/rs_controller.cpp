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

    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    try { dev->enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}

    // set accuracy option
    dev->set_option(rs::option::f200_motion_range, 90); // [0 - 100]
    dev->set_option(rs::option::f200_accuracy, 3); // [0 - 3]
    
    // Instanciate Image publisher
    ImagePublisher ip_depth(sizeof(uint16_t), "rs_camera/image_depth", "mono16");
    ImagePublisher ip_color(sizeof(unsigned char) * 3, "rs_camera/image_color", "rgb8");
    ImagePublisher ip_color_aligned_to_depth(sizeof(unsigned char) * 3, "rs_camera/image_color_aligned_to_depth", "rgb8");
    ImagePublisher ip_depth_aligned_to_color(sizeof(unsigned char) * 3, "rs_camera/image_depth_aligned_to_color", "mono16");

    // Instanciate Deproject Service
    DeprojectService deproject_service;

    // start all service
    dev->start();

    // Initialize Image publisher
    ip_depth.init();
    ip_color.init();
    ip_color_aligned_to_depth.init();
    ip_depth_aligned_to_color.init();

    // set publish functions
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

    // initialize deproject service
    dev->wait_for_frames();
    cv::Mat image(dev->get_stream_height(rs::stream::color), 
                  dev->get_stream_width(rs::stream::color), 
                  cv_bridge::getCvType("mono16"), 
                  (unsigned char *)(dev->get_frame_data(rs::stream::depth_aligned_to_color)));
    deproject_service.set_data(dev->get_depth_scale(), dev->get_stream_intrinsics(rs::stream::color), image);
    std::cout << "start deproject_service." << std::endl;
    deproject_service.start_service();
    
    std::cout << "start publish image." << std::endl;
    // service loop
    ros::Rate loop_rate(10);
    while(n.ok())
    {
        dev->wait_for_frames();
        
        // Publish Images
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

        // Update data for deproject service 
        cv::Mat image(dev->get_stream_height(rs::stream::color), 
                      dev->get_stream_width(rs::stream::color), 
                      cv_bridge::getCvType("mono16"), 
                      (unsigned char *)(dev->get_frame_data(rs::stream::depth_aligned_to_color)));
<<<<<<< HEAD
        deproject_service.set_data(dev->get_depth_scale(), dev->get_stream_intrinsics(rs::stream::color), image.clone());
=======
        deproject_service.set_data(dev->get_depth_scale(), dev->get_stream_intrinsics(rs::stream::color), image);
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
        
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

