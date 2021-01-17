#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyS2"

#define PROTOCOL_VERSION                1.0 
#define PERIOD_PROTOCOL                 15
#define PERIOD_MOVEMENT                 15

DynamixelWorkbench dxl_wb;
//all id

int32_t acceleration = 0;

float goal_velocity;

double ms, ms0;
bool disable_flag = false;

uint8_t d_gain = 0; //128
uint8_t i_gain = 128; //16
uint8_t p_gain = 64; //16

uint8_t dxl_id = 1; // for angle 
std::vector<double> present_speed = {0};

bool ping_motor(uint8_t id, uint16_t model_number){
    const char *log;
    bool result = false;
    result = dxl_wb.ping(id,&model_number,&log);
    if (result == false){
        ROS_ERROR_STREAM(log);
        ROS_ERROR_STREAM("Failed to ping id: "<<(int)id);
    }
    return result;
}
//initialization of motors
bool dxl_init(){
    bool result = false;
    uint16_t model_number = 0;
    const char *log;
    result = dxl_wb.init(DEVICENAME, BAUDRATE, &log);
    if (result == false){
        ROS_ERROR_STREAM(log);
        ROS_ERROR_STREAM("Failed to init");
        return false;
    }
    else{
        ROS_INFO_STREAM("Succeed to init: "<<BAUDRATE); 
    }


    if (ping_motor(dxl_id,model_number) == false){
        printf("dxl_id: %d \n", dxl_id);
        return(false);
	}
	else
		ROS_INFO_STREAM("Succeeded to ping: "<<(int)dxl_id);

    result = dxl_wb.wheelMode(dxl_id, acceleration, &log); //first value - speed, second value - acceleration
    if (result == false){
        ROS_ERROR_STREAM(log);
        ROS_ERROR_STREAM("Failed to change wheel mode");
        return(false);
    } 
    else{
          ROS_INFO_STREAM("Succeed to change wheel mode");
    }
   
  
    dxl_wb.writeRegister(dxl_id, uint16_t(26), uint16_t(1), &d_gain, &log);
    dxl_wb.writeRegister(dxl_id, uint16_t(27), uint16_t(1), &i_gain, &log);
    dxl_wb.writeRegister(dxl_id, uint16_t(28), uint16_t(1), &p_gain, &log);
  
  
    ROS_INFO_STREAM("Succeeded to init dxl");
    return(true);
}

//send positions
bool write_vel(){
    bool result = false;
    const char*log;
    int32_t value = 0;
    value = dxl_wb.convertVelocity2Value(dxl_id, goal_velocity);
    // ROS_INFO_STREAM(value);
    result = dxl_wb.goalVelocity(dxl_id, abs(value), &log);

    //if(result == false){
    //    ROS_INFO_STREAM("Failed to send goal position");
      //  ROS_ERROR_STREAM(log);
    //}
    //else
      //  ROS_INFO_STREAM(log);
    return(result);
    
}
//enabling torque for motor with current id
bool enableTorque(uint8_t id){
    const char*log;
    bool result = false;
    result = dxl_wb.torqueOn(id, &log);
    if(result == false){
        ROS_ERROR_STREAM("Failed to enable Torque on id: "<<(int)id);
        ROS_ERROR_STREAM(log);
    }
    else 
        ROS_INFO_STREAM("Succeded to enable torque on id: "<<(int)id);  
    return result;
}   

//disabling torque for motor with current id
bool disableTorque(uint8_t id){
    const char*log;
    bool result = false;
    result = dxl_wb.torqueOff(id, &log);
    if(result == false){
        ROS_ERROR_STREAM("Failed to disable Torque on id: "<<(int)id);
        ROS_ERROR_STREAM(log);
    }
    else 
        ROS_INFO_STREAM("Succeded to disable torque on id: "<<(int)id);
    return(result);
}

//read position of motor with current id
bool read_vel(uint8_t id){
    const char*log;
    bool result = false;
    int32_t get_data = 0;
    result = dxl_wb.itemRead(id,"Present_Speed", &get_data, &log);
    if(result == false)
    {

        //ROS_ERROR_STREAM(log);
        return(false);
    }
    present_speed[0] = -1*dxl_wb.convertValue2Velocity(dxl_id, get_data);
    return(true);
}
//get time in milliseconds
double millis()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    return (te.tv_sec + (te.tv_usec / 1000000.0)) * 1000.0;
}

void messageJointscmd(const std_msgs::Float64::ConstPtr& msg)  //controll position
{
    goal_velocity = (double)msg->data;
}

int main(int argc, char **argv)
{   
    std::cout << "test node" << std::endl;
    bool result = false;
    bool err_flag = false;
    int iter = 0;
    
    ms0 = millis();
    ros::init(argc,argv, "Dxl_Conveer");
    ros::NodeHandle nh;
    bool param;
    sensor_msgs::JointState joints_msg;
    ros::Subscriber jointcmd = nh.subscribe<std_msgs::Float64> ("cmd_vel",10, messageJointscmd);
    ros::Publisher Joint_State = nh.advertise<sensor_msgs::JointState>("vel_joint_states", 64);

    ros::Rate loop_rate(PERIOD_PROTOCOL);
    nh.getParam("/conveer/disable_torque",param);
    ROS_INFO_STREAM("take param:"<<param);
        if(param == true)
               disable_flag = true;
         else
               disable_flag = false;
    disable_flag = false;
    while(result == false)
        result = dxl_init();

    int error_count = 0;
    bool disable_prev_flag = false;
    while(ros::ok()){
	        
	        result = read_vel(dxl_id); 
	        ms = millis();
	        double delta = ms-ms0;
	        if(delta >= PERIOD_MOVEMENT && err_flag == false && disable_flag == false){
	            err_flag = !write_vel();
	            err_flag = false;
                error_count += err_flag;
	        }
	        if (error_count>32)
	            return 0;
	        if((disable_flag == true) and (disable_prev_flag == false)){
                disableTorque(dxl_id);
	            disable_prev_flag = true;
	        }else if((disable_flag == false) and (disable_prev_flag == true)){
                enableTorque(dxl_id);
	            disable_prev_flag = false;
	        }
	        ms0 = millis();
	        joints_msg.velocity = present_speed;
	        joints_msg.header.stamp = ros::Time::now();
	        Joint_State.publish(joints_msg);
	        // present_position.clear();
	        // present_speed.clear();
	        ros::spinOnce();
	        loop_rate.sleep();
    	}
    
    disableTorque(dxl_id);
    return result;
}
