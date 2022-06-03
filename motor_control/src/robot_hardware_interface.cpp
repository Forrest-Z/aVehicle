#include <motor_control/robot_hardware_interface.h>

RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh) {
  nh_ = nh;

  init();
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  loop_hz_= 20; //20 hz
  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHardwareInterface::update, this);

  velocity_setpoints_ = motor_control::VelocitySetpoints();

  // Get the radius of the wheels from rosparam
  ros::param::get("/mobile_robot/mobile_base_controller/wheel_radius", wheel_radius_);
  first_cb_came = false;
}

RobotHardwareInterface::~RobotHardwareInterface() {
}

void RobotHardwareInterface::init() {
	
	for(int i=0; i<2; i++)
	{
    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
    // hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
  // Register all joints interfaces    
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&velocityJointSaturationInterface);
}

void RobotHardwareInterface::update(const ros::TimerEvent& e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  
  // ROS_INFO("callback: joint_position_[0]: %f, joint_position_[1]: %f",
  //   joint_position_[0], joint_position_[1]
  // );
  getPositions(); // get data from plc
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}


void RobotHardwareInterface::getPositions() {

    motor_control::AngularPositions _positions = ads_motor_controller.getData();
    if (!first_cb_came) {
      joint_position_first[0] = _positions.left_motor; // left motor
      joint_position_first[1] = _positions.right_motor; // right motor
      first_cb_came = true;
    }

    joint_position_[0] = -(_positions.left_motor - joint_position_first[0]);
    joint_position_[1] = _positions.right_motor - joint_position_first[1];

    ROS_INFO_STREAM( "-----\nleft_motor: "<< joint_position_[0] <<
        " , right_motor:" << joint_position_[1] << "\n-----");
}

void RobotHardwareInterface::write(ros::Duration elapsed_time) {
  velocityJointSaturationInterface.enforceLimits(elapsed_time);

  int left_velocity, right_velocity;

  ROS_INFO("joint_velocity_command: [%f, %f]", joint_velocity_command_[0], joint_velocity_command_[1]);

  left_velocity = radianToLinear(joint_velocity_command_[0]);
  right_velocity = radianToLinear(joint_velocity_command_[1]);

  velocity_setpoints_.left_motor = left_velocity;
  velocity_setpoints_.right_motor = right_velocity;

  ads_motor_controller.setData(velocity_setpoints_);
}

int RobotHardwareInterface::radianToLinear(double radian_val) {
  return (int)(radian_val * wheel_radius_ * 1000);
}

double RobotHardwareInterface::linearToRadian(double linear_val) {
  return linear_val / (wheel_radius_ * 1000);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    RobotHardwareInterface robot(nh);
    spinner.spin();
    return 0;
}