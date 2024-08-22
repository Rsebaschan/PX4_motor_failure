#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

// 전역 변수로 ROS 메시지 선언
std_msgs::Float64 my_motor_velocity;

// 콜백 함수: 수신된 메시지를 처리
void motorVelocitySubCallback(const boost::shared_ptr<const gazebo::msgs::Any> &msg)
{
    if (msg->has_double_value()) // 메시지에 double 값이 있는지 확인
    {
        my_motor_velocity.data = msg->double_value();
        ROS_INFO("Received motor velocity: %f", my_motor_velocity.data);
    }
    else
    {
        ROS_WARN("Received a message that does not contain a double value.");
    }
}

int main(int argc, char **argv)
{
    // Gazebo transport 초기화
    gazebo::client::setup(argc, argv);

    // Gazebo 노드 생성 및 초기화
    gazebo::transport::NodePtr node_handle(new gazebo::transport::Node());
    node_handle->Init();

    // Gazebo 토픽 구독 및 콜백 함수 등록
    gazebo::transport::SubscriberPtr motor_velocity_sub = node_handle->Subscribe("/gazebo/default/octocopter3/motor_speed/0", motorVelocitySubCallback);

    // ROS 초기화
    ros::init(argc, argv, "motor_velocity_pub", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Publisher motor_velocity_pub = nh.advertise<std_msgs::Float64>("/my/octocopter3/motor_speed", 10);

    ros::Rate loop_rate(10);

    // 무한 루프: Gazebo 메시지를 지속적으로 수신하고 ROS로 퍼블리시
    while (ros::ok())
    {
        motor_velocity_pub.publish(my_motor_velocity); // ROS 메시지 퍼블리시
        ros::spinOnce(); // ROS 콜백 처리
        loop_rate.sleep(); // 루프 주기 대기
    }

    // Gazebo 종료
    gazebo::client::shutdown();
    return 0;
}
