// #include <ros/ros.h>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>
// #include <geometry_msgs/Twist.h>

// #include <stdio.h>

// #include <boost/bind.hpp>
// #include <Eigen/Eigen>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <gazebo/common/Plugin.hh>

// #include "gazebo/transport/transport.hh"
// #include "gazebo/msgs/msgs.hh"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char **argv)
{
    if (!ros::isInitialized())
    {
        ros::init(argc, argv, "ros_wind_pub");
    }
    ros::NodeHandle nh;
    // this->rosNode = new ros::NodeHandle(this->namespace_);
    // this->my_wind_pub_ = this->rosNode->advertise();
    ros::Publisher my_wind_pub_ = nh.advertise<geometry_msgs::Vector3>("/my_custom_wind", 10);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::Vector3 my_wind_v;
        my_wind_v.x = 0.0;
        my_wind_v.y = 0.0;
        my_wind_v.z = 0.0;

        my_wind_pub_.publish(my_wind_v);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // my_wind_pub_->Publish(*my_wind_v)  //이거는 가제보에서 쓰는 방식
    /*
        geometry_msgs::Twist my_wind_v;
        my_wind_v.linear.x = 0.5;
        my_wind_v.linear.y = 0;
        my_wind_v.linear.z = 0;

    */

    // ignition::math::Vector3d my_wind_v
    /*
    gazebo::msgs::Vector3d* my_wind_v = new gazebo::msgs::Vector3d();
    my_wind_v->set_x(5);
    my_wind_v->set_y(5);
    my_wind_v->set_z(5);
    */

    /*
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/my_change_wind");

    while (ros::ok())
    {
        gazebo::msgs::Vector3d* my_wind_v = new gazebo::msgs::Vector3d();
        // 여기에 my_wind_v를 설정하는 코드 추가
        pub->Publish(*my_wind_v);

        ros::spinOnce();
    }
    */

    return 0;
}
