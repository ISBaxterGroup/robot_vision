<<<<<<< HEAD
//----------------------------------------------------------
// include
//----------------------------------------------------------
=======
/**
 * @file tf_chess_base
 * @brief A ROS-Node for broad cast transform chess to bese.
 * @author Iwase
 * @date 2016.01.07.
 */
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
#include <iostream>
#include <fstream>
#include <utility>
#include <list>
#include <vector>

<<<<<<< HEAD
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

=======
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

inline std::ostream& operator<<(std::ostream& os, const tf::Vector3& v)  
{  
    os << v[0] << " " << v[1] << " " << v[2];  
    return os;  
} 
<<<<<<< HEAD

=======
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
tf::Vector3 get_curr_hand_pos(){
    static tf::TransformListener listener;
    tf::StampedTransform tf_pose_l;

    try{
<<<<<<< HEAD
        //ros::Time now = ros::Time::now();
        ros::Time now = ros::Time(0);
                    
        listener.waitForTransform("base", "left_endgripper2", now, ros::Duration(1.0));
        listener.lookupTransform("base", "left_endgripper2", now, tf_pose_l);      //baseから見た位置姿勢をTFから取得する　ros::Time(0)は最新のタイムスタンプ
=======
        ros::Time now = ros::Time(0);
                    
        listener.waitForTransform("base", "left_endgripper2", now, ros::Duration(1.0));
        listener.lookupTransform("base", "left_endgripper2", now, tf_pose_l);
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    return tf::Vector3(tf_pose_l.getOrigin().x(), tf_pose_l.getOrigin().y(), tf_pose_l.getOrigin().z());
}
<<<<<<< HEAD

=======
void broadcast_transform(const tf::Vector3& p, const tf::Quaternion& q){
    static tf::TransformBroadcaster br;
    tf::Transform transform(q, p);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "chess"));
}
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
void calc_transform_base_chess(const std::list<tf::Vector3> pos_list, tf::Vector3& p, tf::Quaternion& q){
    auto it = pos_list.begin();
    tf::Vector3 origin;
    tf::Vector3 x_dir;
<<<<<<< HEAD
=======
    // Calculate average
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
    for(int i = 0; i < pos_list.size() / 2; ++ i){
        origin += *it / (pos_list.size() / 2);
        ++ it;
        x_dir += *it / (pos_list.size() / 2);
        ++ it;
    }
    tf::Vector3 chess_x_dir = x_dir - origin;
    chess_x_dir = chess_x_dir.normalize();
<<<<<<< HEAD
    std::cout << "chess_x_dir : " << chess_x_dir << std::endl;
=======
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8

    tf::Quaternion q_ideal = tf::Quaternion(tf::Vector3(0, 1, 0), M_PI);
    tf::Transform transform(q_ideal);
    x_dir = transform( tf::Vector3(1, 0, 0) );

    double angle = x_dir.angle(chess_x_dir);
<<<<<<< HEAD
    //tf::Vector3 axis = tf::Vector3(0, 0, -1).normalize();
=======
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
    tf::Vector3 axis = x_dir.cross(chess_x_dir);
    axis = tf::Vector3(0, 0, -axis[2]);
    axis = axis.normalize();

    q += q_ideal * tf::Quaternion(axis, angle);
    p += origin;
<<<<<<< HEAD
         
    
};

void mesurment(tf::Vector3& p, tf::Quaternion& q){
    std::list<tf::Vector3> pos_list;
    std::cout << "Move left hand to origin of chess-coordinates" << std::endl;
    std::cout << "c to capture" << std::endl;

    //cv::Mat image[2];
    //image[0] = cv::imread("src/lis_core/image/process1.jpg", cv::IMREAD_COLOR);
    //image[1] = cv::imread("src/lis_core/image/process2.jpg", cv::IMREAD_COLOR);

    
    const int loop_num(6);
    assert(loop_num % 2 == 0);
    while(pos_list.size() < loop_num){
    	cv::namedWindow("process", cv::WINDOW_AUTOSIZE);
        //cv::imshow("process", image[pos_list.size() % 2]);
=======
};
void mesure(tf::Vector3& p, tf::Quaternion& q){
    std::list<tf::Vector3> pos_list;
    std::cout << "c to capture" << std::endl;
 
    const int loop_num(6);
    while(pos_list.size() < loop_num){
        if(pos_list.size() % 2 == 0 )   std::cout << "Move left hand to origin of chess-coordinates" << std::endl;
        else std::cout << "Move left hand to x-axis-direction of chess-coordinates" << std::endl;
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
        char key = cv::waitKey();
        if(key == 'c'){
            pos_list.push_back(get_curr_hand_pos());
            std::cout << "capture : " << pos_list.back() << std::endl;
            std::cout << "captured number of position : " << pos_list.size() << " / "<< loop_num <<  std::endl; 
        }
<<<<<<< HEAD
        if(pos_list.size() == loop_num){
            std::cout << "calc_transform_base_chess" << std::endl;
            calc_transform_base_chess(pos_list, p, q);     
            std::cout << "calc_transform_base_chess_end" << std::endl;
        }
        if(pos_list.size() % 2 == 0)
    		std::cout << "Move left hand to origin of chess-coordinates" << std::endl;
        else 
    		std::cout << "Move left hand to x-axis direction of chess-coordinates" << std::endl;
    	cv::destroyWindow("process");
    }
}

void broadcast_transform(const tf::Vector3& p, const tf::Quaternion& q){
    static tf::TransformBroadcaster br;
    tf::Transform transform(q, p);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "chess"));
}

=======
    }

    calc_transform_base_chess(pos_list, p, q);     
}
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
//main entry point of this program
int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_chess_base");
    ros::NodeHandle n;

    tf::Vector3 trans;
    tf::Quaternion rot;
    // Calculate transform bese to chess bord
    std::cout << "Mesure start." << std::endl;
<<<<<<< HEAD
    mesurment(trans, rot);
    // Broadcast transform on tf
    std::cout << "TF Broadcast start." << std::endl;
    
=======
    mesure(trans, rot);
    // Broadcast transform on tf
    std::cout << "TF Broadcast start." << std::endl;
    // Broad cast loop
>>>>>>> 5d7c81f007a3d614496af9c638fe732139d2cca8
    ros::Rate rate(10.0);
    while (n.ok()){
      broadcast_transform(trans, rot);
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}