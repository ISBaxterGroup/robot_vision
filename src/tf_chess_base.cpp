/**
 * @file tf_chess_base
 * @brief A ROS-Node for broad cast transform chess to bese.
 * @author Iwase
 * @date 2016.01.07.
 */
#include <iostream>
#include <fstream>
#include <utility>
#include <list>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

inline std::ostream& operator<<(std::ostream& os, const tf::Vector3& v)  
{  
    os << v[0] << " " << v[1] << " " << v[2];  
    return os;  
} 
tf::Vector3 get_curr_hand_pos(){
    static tf::TransformListener listener;
    tf::StampedTransform tf_pose_l;

    try{
        ros::Time now = ros::Time(0);
                    
        listener.waitForTransform("base", "left_endgripper2", now, ros::Duration(1.0));
        listener.lookupTransform("base", "left_endgripper2", now, tf_pose_l);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    return tf::Vector3(tf_pose_l.getOrigin().x(), tf_pose_l.getOrigin().y(), tf_pose_l.getOrigin().z());
}
void broadcast_transform(const tf::Vector3& p, const tf::Quaternion& q){
    static tf::TransformBroadcaster br;
    tf::Transform transform(q, p);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "chess"));
}
void calc_transform_base_chess(const std::list<tf::Vector3> pos_list, tf::Vector3& p, tf::Quaternion& q){
    auto it = pos_list.begin();
    tf::Vector3 origin;
    tf::Vector3 x_dir;
    // Calculate average
    for(int i = 0; i < pos_list.size() / 2; ++ i){
        origin += *it / (pos_list.size() / 2);
        ++ it;
        x_dir += *it / (pos_list.size() / 2);
        ++ it;
    }
    tf::Vector3 chess_x_dir = x_dir - origin;
    chess_x_dir = chess_x_dir.normalize();

    tf::Quaternion q_ideal = tf::Quaternion(tf::Vector3(0, 1, 0), M_PI);
    tf::Transform transform(q_ideal);
    x_dir = transform( tf::Vector3(1, 0, 0) );

    double angle = x_dir.angle(chess_x_dir);
    tf::Vector3 axis = x_dir.cross(chess_x_dir);
    axis = tf::Vector3(0, 0, -axis[2]);
    axis = axis.normalize();

    q += q_ideal * tf::Quaternion(axis, angle);
    p += origin;
};
void mesure(tf::Vector3& p, tf::Quaternion& q){
    std::list<tf::Vector3> pos_list;
    std::cout << "c to capture" << std::endl;
 
    const int loop_num(6);
    while(pos_list.size() < loop_num){
        if(pos_list.size() % 2 == 0 )   std::cout << "Move left hand to origin of chess-coordinates" << std::endl;
        else std::cout << "Move left hand to x-axis-direction of chess-coordinates" << std::endl;
        char key = cv::waitKey();
        if(key == 'c'){
            pos_list.push_back(get_curr_hand_pos());
            std::cout << "capture : " << pos_list.back() << std::endl;
            std::cout << "captured number of position : " << pos_list.size() << " / "<< loop_num <<  std::endl; 
        }
    }

    calc_transform_base_chess(pos_list, p, q);     
}
//main entry point of this program
int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_chess_base");
    ros::NodeHandle n;

    tf::Vector3 trans;
    tf::Quaternion rot;
    // Calculate transform bese to chess bord
    std::cout << "Mesure start." << std::endl;
    mesure(trans, rot);
    // Broadcast transform on tf
    std::cout << "TF Broadcast start." << std::endl;
    // Broad cast loop
    ros::Rate rate(10.0);
    while (n.ok()){
      broadcast_transform(trans, rot);
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}