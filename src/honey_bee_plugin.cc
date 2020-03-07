#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>

#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace gazebo
{

    class HONEY_BEE : public ModelPlugin
    {
        physics::LinkPtr BASE_LINK;
        physics::LinkPtr L_GEAR_LINK;
        physics::LinkPtr R_GEAR_LINK;
        physics::LinkPtr RL_MOTOR_LINK;
        physics::LinkPtr RR_MOTOR_LINK;
        physics::LinkPtr FL_MOTOR_LINK;
        physics::LinkPtr FR_MOTOR_LINK;

        physics::JointPtr L_GEAR_JOINT;
        physics::JointPtr R_GEAR_JOINT;
        physics::JointPtr RL_MOTOR_JOINT;
        physics::JointPtr RR_MOTOR_JOINT;
        physics::JointPtr FL_MOTOR_JOINT;
        physics::JointPtr FR_MOTOR_JOINT;

        physics::ModelPtr model;


        sensors::SensorPtr IMU_Sensor;
        sensors::ImuSensorPtr IMU;

        sensors::SensorPtr ALTI_Sensor;
        sensors::AltimeterSensorPtr ALTIMETER;

        // Servo Control
        double tar_q[2] = {0, 0};
        double q_err[2];
        double kp_q[2] = {10, 10};
        double kd_q[2] = {2, 2};
        double actual_q[2] = {0, 0};
        double actual_q_dot[2] = {0, 0};
        double pre_actual_q[2] = {0, 0};
        double tar_torque[2] = {0, 0};
        double goal_angle = 0.0;
        double goal_q[2] = {0, 0};
        double init_q[2] = {0, 0};
        bool instruction_flag = false;
        bool servo_flag=false;
        
        // Thrust Control
        double Fg = (1.0 + 0.25 * 2 + 0.6 * 4)*9.8;
        double h_err;
        double kp_h = 20;
        double kd_h =2;
        double actual_height;
        double actual_vel_z;
        double tar_force_height;
        double init_height = 0.0;
        //double tar_height=1.0;
        double tar_height = 0.0;
        double goal_height;

        //Orientation Control
        double tar_Roll = 0.0;
        double tar_Pitch = 0.0;
        double tar_Yaw = 0.0;
        double kp_roll = 20;
        double kd_roll = 2;
        double kp_pitch = 20;
        double kd_pitch = 2;
        double kp_yaw = 10;
        double kd_yaw = 2;
        double Roll_force;
        double Pitch_force;

        // IMU
        double wx = 0.0, wy = 0.0, wz = 0.0;
        double ax = 0.0, ay = 0.0, az = 0.0;
        double actual_Roll = 0.0, actual_Pitch = 0.0, actual_Yaw = 0.0;

        //setting for getting <dt>(=derivative time)
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;
        unsigned int cnt = 0;
        double cycle = 3.0;

        ros::NodeHandle n;
        ros::Subscriber S_target;
        ros::Subscriber S_joy;
        ros::Publisher P_data;
        std_msgs::Float64MultiArray m_data;

        math::Vector3 m_RL_force;
        math::Vector3 m_RR_force;
        math::Vector3 m_FL_force;
        math::Vector3 m_FR_force;
        math::Pose pose;

        //Joy stick param
        double vel_x = 0.0;
        double vel_y = 0.0;
        double vel_z = 0.0;
        double ang_z = 0.0;
        double des_z = 0.0;
        double des_forward = 0.0;
        double des_backward = 0.0;
        double des_y = 0.0;
        MatrixXd Rot_mat = MatrixXd::Zero(3, 3);
        
        VectorXd e_RL_force_z=VectorXd::Zero(3);
        VectorXd e_RR_force_z=VectorXd::Zero(3);
        VectorXd e_FL_force_z=VectorXd::Zero(3);
        VectorXd e_FR_force_z=VectorXd::Zero(3);
        
        VectorXd e_RL_force=VectorXd::Zero(3);
        VectorXd e_RR_force=VectorXd::Zero(3);
        VectorXd e_FL_force=VectorXd::Zero(3);
        VectorXd e_FR_force=VectorXd::Zero(3);
        
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void Callback1(const std_msgs::Float64 &msg);
        void Callback2(const sensor_msgs::Joy& msg);
        MatrixXd Rotation_Matrix(double Roll, double Pitch);
    };
    GZ_REGISTER_MODEL_PLUGIN(HONEY_BEE);
}

void gazebo::HONEY_BEE::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    // model = link + joint +sensor
    this->model = _model;

    //LINK DEFINITION
    this->BASE_LINK = this->model->GetLink("base_link");
    this->L_GEAR_LINK = this->model->GetLink("left_gear_link");
    this->R_GEAR_LINK = this->model->GetLink("right_gear_link");
    this->RL_MOTOR_LINK = this->model->GetLink("RL_motor_link");
    this->RR_MOTOR_LINK = this->model->GetLink("RR_motor_link");
    this->FL_MOTOR_LINK = this->model->GetLink("FL_motor_link");
    this->FR_MOTOR_LINK = this->model->GetLink("FR_motor_link");

    //JOINT DEFINITION
    this->L_GEAR_JOINT = this->model->GetJoint("left_joint");
    this->R_GEAR_JOINT = this->model->GetJoint("right_joint");
    this->RL_MOTOR_JOINT = this->model->GetJoint("RL_joint");
    this->RR_MOTOR_JOINT = this->model->GetJoint("RR_joint");
    this->FL_MOTOR_JOINT = this->model->GetJoint("FL_joint");
    this->FR_MOTOR_JOINT = this->model->GetJoint("FR_joint");

    this->IMU_Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(IMU_Sensor);
    this->IMU->SetActive(true);

    this->ALTI_Sensor = sensors::get_sensor("ALTIMETER");
    this->ALTIMETER = std::dynamic_pointer_cast<sensors::AltimeterSensor>(ALTI_Sensor);
    this->ALTIMETER->SetActive(true);

    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&HONEY_BEE::UpdateAlgorithm, this));

    S_target = n.subscribe("tar_angle", 1, &gazebo::HONEY_BEE::Callback1, this);
    S_joy = n.subscribe("joy", 1, &gazebo::HONEY_BEE::Callback2, this);
    P_data = n.advertise<std_msgs::Float64MultiArray>("Print", 1);
    m_data.data.resize(10);
}

void gazebo::HONEY_BEE::UpdateAlgorithm()
{
    pose = this->model->GetWorldPose();
    //* Calculate time
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    // IMU Data
    wx = this->IMU->AngularVelocity(false)[0];
    wy = this->IMU->AngularVelocity(false)[1];
    wz = this->IMU->AngularVelocity(false)[2];
    ax = this->IMU->LinearAcceleration(false)[0];
    ay = this->IMU->LinearAcceleration(false)[1];
    az = this->IMU->LinearAcceleration(false)[2];

    //+) Kalman Filter
    actual_Roll = pose.rot.GetRoll(); //PongBotQ.IMURoll + PongBotQ.IMURoll_dot*dt;
    actual_Pitch = pose.rot.GetPitch(); //PongBotQ.IMUPitch + PongBotQ.IMUPitch_dot*dt;
    actual_Yaw = pose.rot.GetYaw(); //PongBotQ.IMUYaw + PongBotQ.IMUYaw_dot*dt;



    // Altimeter Data
    actual_height = this->ALTIMETER->Altitude();
    actual_vel_z = this->ALTIMETER->VerticalVelocity();

    //* Get Angle
    actual_q[0] = this->L_GEAR_JOINT->GetAngle(0).Radian();
    actual_q[1] = this->R_GEAR_JOINT->GetAngle(0).Radian();
    for (int i = 0; i < 2; ++i) {
        actual_q_dot[i] = (actual_q[i] - pre_actual_q[i]) / dt;
        pre_actual_q[i] = actual_q[i];
    }

    if (instruction_flag == true) {
        if(servo_flag==true){
                if (cnt == 0) {
                    goal_q[0] = goal_angle*D2R;
                    goal_q[1] = -goal_angle*D2R;
                    cnt++;
                }
                else if (cnt <= (unsigned int) (cycle / dt)) {
                    for (int i = 0; i < 2; ++i) {
                        tar_q[i] = init_q[i]+(goal_q[i] - init_q[i]) / 2.0 * (1 - cos(PI / cycle * (double) (cnt) * dt));
                    }
                    cnt++;
                }
                else {
                    for (int i = 0; i < 2; ++i) {
                        init_q[i] = tar_q[i];
                    }
                    cnt = 0;
                    servo_flag=false;
                }
         }
        
        des_z = des_z + vel_z * dt * 0.5;
        tar_height = des_z;
        
        if(vel_x>0){
         des_backward=0.0;
         des_forward = vel_x*3;    
        }
        else if(vel_x<0){
         des_forward=0.0;
         des_backward = -vel_x*3;    
        }
        else{
         des_forward=0.0;   
         des_backward=0.0;
        }
        ///des_y = vel_y;
    }

    else {
        des_z = tar_height;
    }

    // Error
    for (int i = 0; i < 2; ++i) {
        q_err[i] = tar_q[i] - actual_q[i];
    }
    h_err = (tar_height - actual_height);

    // Get Target Torque & Force
    for (int i = 0; i < 2; ++i) {
        tar_torque[i] = kp_q[i] * q_err[i] + kd_q[i]*(0.0 - actual_q_dot[i]);
    }
    tar_force_height = kp_h * h_err + kd_h * (0.0 - actual_vel_z);
    Roll_force = kp_roll * (0.0 - actual_Roll) + kd_roll * (0.0 - wx);
    Pitch_force = kp_pitch * (0.0 - actual_Pitch) + kd_pitch * (0.0 - wy);

    // Set Force & Torque
    this->L_GEAR_JOINT->SetForce(1, tar_torque[0]);
    this->R_GEAR_JOINT->SetForce(1, tar_torque[1]);

//    e_RL_force_z<<0,0,(tar_force_height + Pitch_force + Roll_force) + des_forward;
//    e_RR_force_z<<0,0,(tar_force_height + Pitch_force - Roll_force) + des_forward;
//    e_FL_force_z<<0,0,(tar_force_height - Pitch_force + Roll_force) + des_backward;
//    e_FR_force_z<<0,0,(tar_force_height - Pitch_force - Roll_force) + des_backward;
    
    e_RL_force_z<<0,0,( Pitch_force + Roll_force) + des_forward;
    e_RR_force_z<<0,0,( Pitch_force - Roll_force) + des_forward;
    e_FL_force_z<<0,0,(-Pitch_force + Roll_force) + des_backward;
    e_FR_force_z<<0,0,(-Pitch_force - Roll_force) + des_backward;
    
    //    RL_force.x = 0.0;
//    RL_force.y = 0.0;
    //RL_force.z = (tar_force_height + Pitch_force + Roll_force) + des_x;
//    RR_force.x = 0.0;
//    RR_force.y = 0.0;
//    RR_force.z = (tar_force_height + Pitch_force - Roll_force) + des_x;
//    FL_force.x = 0.0;
//    FL_force.y = 0.0;
//    FL_force.z = (tar_force_height - Pitch_force + Roll_force);
//    FR_force.x = 0.0;
//    FR_force.y = 0.0;
//    FR_force.z = (tar_force_height - Pitch_force - Roll_force);

    Rot_mat = Rotation_Matrix(actual_Roll, actual_Pitch);
    e_RL_force=Rot_mat*e_RL_force_z;
    e_RR_force=Rot_mat*e_RR_force_z;
    e_FL_force=Rot_mat*e_FL_force_z;
    e_FR_force=Rot_mat*e_FR_force_z;
    
    m_RL_force.x=e_RL_force(0);
    m_RL_force.y=e_RL_force(1);
    m_RL_force.z=e_RL_force(2)+tar_force_height;
    
    m_RR_force.x=e_RR_force(0);
    m_RR_force.y=e_RR_force(1);
    m_RR_force.z=e_RR_force(2)+tar_force_height;
    
    m_FL_force.x=e_FL_force(0);
    m_FL_force.y=e_FL_force(1);
    m_FL_force.z=e_FL_force(2)+tar_force_height;
    
    m_FR_force.x=e_FR_force(0);
    m_FR_force.y=e_FR_force(1);
    m_FR_force.z=e_FR_force(2)+tar_force_height;
   
    this->RL_MOTOR_LINK->SetForce(m_RL_force);
    this->RR_MOTOR_LINK->SetForce(m_RR_force);
    this->FL_MOTOR_LINK->SetForce(m_FL_force);
    this->FR_MOTOR_LINK->SetForce(m_FR_force);

  
    m_data.data[0] = e_RL_force[0];
    m_data.data[1] = e_RL_force[1];
    m_data.data[2] = e_RL_force[2];
    m_data.data[3] = e_FL_force[0];
    m_data.data[4] = e_FL_force[1];
    m_data.data[5] = e_FL_force[2];
//    m_data.data[2] = e_FL_force[2];
//    m_data.data[3] = e_FR_force[0];
//    m_data.data[0] = e_RL_force[0];
//    m_data.data[1] = e_RR_force[1];
//    m_data.data[2] = e_FL_force[2];
//    m_data.data[3] = e_FR_force[0];
//    m_data.data[0] = e_RL_force[0];
//    m_data.data[1] = e_RR_force[1];
//    m_data.data[2] = e_FL_force[2];
//    m_data.data[3] = e_FR_force[0];
    
//    m_data.data[4] = des_x;
//    m_data.data[5] = des_y;
//    m_data.data[6] = actual_height;
//    m_data.data[7] = actual_Pitch;
    P_data.publish(m_data);


    this->last_update_time = current_time;
}

void gazebo::HONEY_BEE::Callback1(const std_msgs::Float64 &msg)
{
    goal_angle = msg.data;
    instruction_flag = true;
}

void gazebo::HONEY_BEE::Callback2(const sensor_msgs::Joy& msg)
{
    vel_x = (float) msg.axes[1]; //pitch
    vel_y = (float) msg.axes[0];
    //    ang_z = (float) msg.axes[2];
    vel_z = (float) msg.axes[3];
    
    if ((int) msg.buttons[6] == 1) {
        instruction_flag = true;
    }

    if ((int) msg.buttons[7] == 1) {
        instruction_flag = false;
    }
    
    if((int) msg.buttons[8] == 1) {
        goal_angle=90;
        servo_flag=true;
    }
    if((int) msg.buttons[9] == 1){
        goal_angle=0;
        servo_flag=true;
    }
}

MatrixXd gazebo::HONEY_BEE::Rotation_Matrix(double Roll, double Pitch)
{
    MatrixXd R_roll(3, 3);
    MatrixXd R_pitch(3, 3);
    MatrixXd Rot(3, 3);

    R_roll << 1, 0, 0 \
, 0, cos(Roll), -sin(Roll) \
, 0, sin(Roll), cos(Roll);


    R_pitch << cos(Pitch), 0, sin(Pitch)\
             , 0, 1, 0\
, -sin(Pitch), 0, cos(Pitch);
    Rot = R_roll*R_pitch;

    return Rot;
}