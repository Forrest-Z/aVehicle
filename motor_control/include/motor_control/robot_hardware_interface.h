#include <angles/angles.h>
#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <motor_control/ads_motor_controller.h>

class RobotHardwareInterface : public hardware_interface::RobotHW
{
  public:
    RobotHardwareInterface(ros::NodeHandle& nh);
    ~RobotHardwareInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void write(ros::Duration elapsed_time);
    void getPositions();

    AdsMotorController ads_motor_controller;

  protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;

    std::string joint_name_[2] = {"left_wheel_joint", "right_wheel_joint"};
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];

    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  private:
    motor_control::VelocitySetpoints velocity_setpoints_;

    double wheel_radius_;
    
    bool first_cb_came;
    double joint_position_first[2];
    
    int radianToLinear(double radian_val);
    double linearToRadian(double linear_val);
};