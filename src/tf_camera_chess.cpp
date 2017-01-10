//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <fstream>
#include <utility>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "ROSFramework.hpp"

constexpr unsigned int horizontal_corner_num(8);
constexpr unsigned int vertical_corner_num(6);
constexpr double length_between_corner(35); // [mm]

inline std::ostream& operator<<(std::ostream& os, const tf::Vector3& v)  
{  
    os << v[0] << " " << v[1] << " " << v[2];  
    return os;  
} 

void drawAxis(cv::Mat &_image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
    cv::InputArray _rvec, cv::InputArray _tvec, double length) {
    // project axis points
    std::vector< cv::Point3f > axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    std::vector< cv::Point2f > imagePoints;
    cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    // draw axis lines
    cv::line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
    cv::line(_image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}

void measure(tf::Vector3& p, tf::Quaternion& q, const cv::Mat& src_img){    

    // Find chessbord corners
    cv::namedWindow("src", CV_WINDOW_AUTOSIZE);

    std::vector<cv::Point2f> corners;
    cv::Size pattern_size(horizontal_corner_num, vertical_corner_num);
    bool is_find = cv::findChessboardCorners(src_img, pattern_size, corners);

    cv::Mat gray( src_img.rows, src_img.cols, CV_8UC1 );
    cv::cvtColor( src_img, gray, CV_BGR2GRAY);
    //cv::Size window_size(20, 20);
    //cv::cornerSubPix(gray, corners, window_size, cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
    cv::drawChessboardCorners( src_img, cv::Size(horizontal_corner_num, vertical_corner_num), ( cv::Mat )corners, is_find);

    std::cout << corners.size() << std::endl;
    cv::imshow("src", src_img);
    std::cout << "corners that found. ... press any key." << std::endl;
    cv::waitKey();

    cv::Mat intrinsic;
    cv::Mat distortion;

    cv::FileStorage fs("src/robot_vision/yaml/ost.yaml", cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cerr << "Failed to open YAML file." << std::endl;
        exit(EXIT_FAILURE);
    }

    fs["camera_matrix"] >> intrinsic;
    fs["distortion_coefficients"] >> distortion;

    //cv::Mat object_points(horizontal_corner_num * vertical_corner_num, 1, CV_32FC3); // Corner position on chessboard coordinate system
    std::vector<cv::Point3f> object_points(horizontal_corner_num * vertical_corner_num); // Corner position on chessboard coordinate system
    //cv::Mat object_points2(horizontal_corner_num * vertical_corner_num, 1, CV_32FC2); // Corner position on u-v coordinate system
    std::vector<cv::Point2f> object_points2(horizontal_corner_num * vertical_corner_num); // Corner position on u-v coordinate system
    
    cv::Mat rot_rodrigues;
    cv::Mat rot_camera_chess;
    cv::Mat trans_camera_chess;
    
    // Set object points
    for(int i = 0; i < horizontal_corner_num * vertical_corner_num; ++ i){
        object_points[i].x = (i % horizontal_corner_num) * length_between_corner;
        object_points[i].y = (i / horizontal_corner_num) * length_between_corner;
        object_points[i].z = 0.0;
        object_points2[i].x = corners[i].x;
        object_points2[i].y = corners[i].y;
    }

    // Find Extrinsic param
    cv::solvePnP(object_points, object_points2, intrinsic, distortion, rot_rodrigues, trans_camera_chess);
    cv::Mat mat_img( src_img );

    // Drow coordinate system using Extrinsic parameter 
    drawAxis(mat_img, intrinsic, distortion, rot_rodrigues, trans_camera_chess, length_between_corner);

    std::cout << "Display chess board coordinate system using Extrinsic parameter. ... press any key." << std::endl;
    cv::imshow("src", mat_img);
    cv::waitKey();

    // Convert to rotation matrix
    cv::Rodrigues(rot_rodrigues, rot_camera_chess);

    cv::Mat rot_chess_camera;
    cv::Mat trans_chess_camera;

    rot_chess_camera = rot_camera_chess.t();
    trans_chess_camera = - rot_chess_camera * trans_camera_chess;

    double * data;
    // Convert to tf style
    p[0] = trans_chess_camera.at<double>(0, 0) / 1000.0;
    p[1] = trans_chess_camera.at<double>(1, 0) / 1000.0;
    p[2] = trans_chess_camera.at<double>(2, 0) / 1000.0;

    
    tf::Matrix3x3 tf_mat{ rot_chess_camera.at<double>(0), rot_chess_camera.at<double>(1), rot_chess_camera.at<double>(2),
                          rot_chess_camera.at<double>(3), rot_chess_camera.at<double>(4), rot_chess_camera.at<double>(5),
                          rot_chess_camera.at<double>(6), rot_chess_camera.at<double>(7), rot_chess_camera.at<double>(8) };

    tf_mat.getRotation(q);
}

void broadcast_transform(const tf::Vector3& p, const tf::Quaternion& q){
    static tf::TransformBroadcaster br;
    tf::Transform transform(q, p);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "chess", "camera"));
}

//main entry point of this program
int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_camera_chess");
    ros::NodeHandle n;

    // get image
    std::cout << "c to capture image." << std::endl;

    // start subscriber
    ROSSubscriberInterface<sensor_msgs::ImageConstPtr> image_subscriber("rs_camera/image_color");
    image_subscriber.init();

    cv::namedWindow("captured image", cv::WINDOW_AUTOSIZE);
    while(cv::waitKey(0) != 'c');

    // Get latest image in topic
    sensor_msgs::ImageConstPtr sm_image;
    if(image_subscriber.new_data()){
        image_subscriber.get(sm_image);
    }
    else{
        std::cout << "There are no image in ros topic." << std::endl;
        exit(EXIT_FAILURE);
    }

    // convert sensor_msgs to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(sm_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(EXIT_FAILURE);
    }
    cv::Mat imgage = cv_ptr->image;
    cv::imshow("captured image", imgage);
    cv::waitKey(0);

    // Calculate transform camera to chess bord
    std::cout << "Measure start." << std::endl;
    tf::Vector3 trans_chess_camera;
    tf::Quaternion rot_chess_camera;
    measure(trans_chess_camera, rot_chess_camera, imgage);

    // Broadcast transform on tf
    std::cout << "TF Broadcast start." << std::endl;
    ros::Rate rate(10.0);
    while (n.ok()){
        broadcast_transform(trans_chess_camera, rot_chess_camera);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
