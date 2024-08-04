#include "gazebo_ros_communicate_plugin.h"

namespace gazebo
{
    const std::string MotorVelocityPlugin::kDefaultCommunicateMotorVelocityPubTopic = "/motor_speed";

    MotorVelocityPlugin::MotorVelocityPlugin() : ModelPlugin(), my_motor_velocity()
    {
    }

    MotorVelocityPlugin::~MotorVelocityPlugin()
    {
        delete rosNode; // 동적으로 할당된 ROS 노드 핸들 해제
    }

    void MotorVelocityPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;
        namespace_.clear();

        if (_sdf->HasElement("robotNamespace"))
        {
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        else
        {
            gzerr << "[gazebo_ros_communicate_plugin.cpp] Please specify a robotNamespace.\n";
        }

        if (_sdf->HasElement("communicatemotorSpeedPubTopic"))
        {
            communicate_motor_Speed_Pub_Topic = _sdf->GetElement("communicatemotorSpeedPubTopic")->Get<std::string>();
        }
        else
        {
            communicate_motor_Speed_Pub_Topic = kDefaultCommunicateMotorVelocityPubTopic;
            gzerr << "[gazebo_ros_communicate_plugin.cpp] Please specify a communicatemotorSpeedPubTopic.\n";
        }

        if (_sdf->HasElement("communicatemotorNumber"))
        {
            communicate_motor_Number = _sdf->GetElement("communicatemotorNumber")->Get<int>();
        }
        else
        {
            gzerr << "[gazebo_ros_communicate_plugin.cpp] Please specify a communicatemotorNumber.\n";
        }

        node_handle = transport::NodePtr(new transport::Node());
        node_handle->Init(namespace_);

        std::string topic_name = communicate_motor_Speed_Pub_Topic + std::to_string(communicate_motor_Number);
        motor_velocity_sub = node_handle->Subscribe<std_msgs::msgs::Float>(topic_name, &MotorVelocityPlugin::motorVelocitySubCallback, this);
        std::cout << "[gazebo_ros_communicate_plugin.cpp] Gazebo topic subscribed: " << topic_name << std::endl;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "motor_velocity_pub", ros::init_options::NoSigintHandler);
        }

        rosNode = new ros::NodeHandle(namespace_);
        motor_velocity_pub = rosNode->advertise<std_msgs::Float64>("/my/octocopter3/motor_speed", 10);
        std::cout << "[gazebo_ros_communicate_plugin.cpp] ROS node initialized and topic advertised: /my/octocopter3/motor_speed" << std::endl;

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MotorVelocityPlugin::OnUpdate, this));
    }

    void MotorVelocityPlugin::motorVelocitySubCallback(const boost::shared_ptr<const std_msgs::msgs::Float> &msg)
    {
        // std::cout << "[gazebo_ros_communicate_plugin.cpp] Motor velocity message received" << std::endl;
        if (msg->has_data())
        {
            my_motor_velocity.data = msg->data();
            std::cout << "[gazebo_ros_communicate_plugin.cpp] Received motor velocity: " << my_motor_velocity.data << std::endl;
        }
        else
        {
            std::cout << "[gazebo_ros_communicate_plugin.cpp] Received a message that does not contain a double value." << std::endl;
        }
    }

    void MotorVelocityPlugin::OnUpdate()
    {
        motor_velocity_pub.publish(my_motor_velocity);
        // std::cout << "Published motor velocity: " << my_motor_velocity.data << std::endl;
    }

    GZ_REGISTER_MODEL_PLUGIN(MotorVelocityPlugin)
}
