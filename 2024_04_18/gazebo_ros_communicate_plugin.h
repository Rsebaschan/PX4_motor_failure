#ifndef GAZEBO_ROS_COMMUNICATE_PLUGIN_HH
#define GAZEBO_ROS_COMMUNICATE_PLUGIN_HH

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <std_msgs/Float64.h>
#include "Float.pb.h"
#include <ros/ros.h>
#include <iostream>

namespace gazebo
{

  class MotorVelocityPlugin : public ModelPlugin
  {
  public:
    MotorVelocityPlugin();
    virtual ~MotorVelocityPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  protected:
    void motorVelocitySubCallback(const boost::shared_ptr<const std_msgs::msgs::Float> &msg);
    void OnUpdate();

  private:
    transport::NodePtr node_handle;                                    // Gazebo 노드 핸들
    transport::SubscriberPtr motor_velocity_sub;                       // Gazebo 구독자
    ros::NodeHandle *rosNode;                                          // ROS 노드 핸들
    ros::Publisher my_motor_velocity_pub;                              // ROS 퍼블리셔
    std_msgs::Float64 my_motor_velocity;                               // ROS 메시지
    event::ConnectionPtr updateConnection;                             // Gazebo 업데이트 연결
    static const std::string kDefaultCommunicateMotorVelocityPubTopic; // 기본 모터 속도 발행 토픽
    std::string communicate_motor_Speed_Pub_Topic;                     // 모터 속도 발행 토픽
    physics::ModelPtr model_;                                          // 모델 포인터
    std::string namespace_;                                            // 네임스페이스
    int communicate_motor_Number{0};
    int a = 0;
  };

}

#endif // GAZEBO_ROS_COMMUNICATE_PLUGIN_HH
