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
#include <std_msgs/Bool.h>
#include <time.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define PROTOCOL_VERSION                1.0 
#define PERIOD_PROTOCOL                 15
#define PERIOD_MOVEMENT                 15

DynamixelWorkbench dxl_wb;
//all id
uint8_t DXL_COUNT = 0;
uint8_t *dxl_id;
int32_t *goal_position; //zero poses
int32_t velocity = 30;
int32_t *goal_velocity;
int32_t *goal_acceleration;
std::vector<double> present_position;
std::vector<double> present_speed;
std::vector<double> present_temp;
std::vector<double> present_load;

double ms, ms0;
bool disable_flag = false;

uint8_t d_gain = 0; //128
uint8_t i_gain = 128; //16
uint8_t p_gain = 64; //16

uint8_t dxl_id_ping[6] = {1,4,2,3,5,6}; // for angle 
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
bool dxl_init(std::string manipulator_type){
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
        //ROS_INFO_STREAM("Succeed to init: "<<BAUDRATE); 
        //std::cout<<"Succeed to init: "<<BAUDRATE<<std::endl;
        
    }

   	if(manipulator_type == "angle"){
   		
   		for (int cnt = 0; cnt < 6; cnt++){
            if (ping_motor(dxl_id_ping[cnt],model_number) == false){
                printf("dxl_id: %d \n", dxl_id_ping[cnt]);
                return(false);
        	}
        	else{
                //ROS_INFO_STREAM("Succeeded to ping: "<<(int)dxl_id[cnt]);
            }
   		}
   		// delete dxl_id_ping;
   	}
   	else{
	    for(int cnt = 0; cnt < DXL_COUNT; cnt++){
            if (ping_motor(dxl_id[cnt],model_number) == false){
                printf("dxl_id: %d \n", dxl_id[cnt]);
                return(false);
            }
            else
            {
                //std::cout<<cnt+1;
            }
	    }
	    //ROS_INFO_STREAM("Succeeded to ping");
	}	

    for (int cnt = 0; cnt< DXL_COUNT; cnt++){
        result = dxl_wb.jointMode(dxl_id[cnt], velocity, 100, &log); //first value - speed, second value - acceleration
        if (result == false){
            ROS_ERROR_STREAM(log);
            ROS_ERROR_STREAM("Failed to change joint mode");
            return(false);
        } 
        else{
            //std::cout<<cnt+1;
        }
    }
    //ROS_INFO_STREAM("Succeed to change joint mode");
    //std::cout<<"Succeed to change joint mode"<<std::endl;
    
   if(manipulator_type == "delta"){
        for(int iter = 0; iter < DXL_COUNT; iter++){
	        dxl_wb.writeRegister(dxl_id[iter], uint16_t(26), uint16_t(1), &d_gain, &log);
	        dxl_wb.writeRegister(dxl_id[iter], uint16_t(27), uint16_t(1), &i_gain, &log);
	        dxl_wb.writeRegister(dxl_id[iter], uint16_t(28), uint16_t(1), &p_gain, &log);
        	//ROS_INFO_STREAM(log);
        }
    }    
    else{
    	dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position",&log);
	    //ROS_INFO_STREAM(log);
	    dxl_wb.addSyncWriteHandler(dxl_id[0], "Moving_Speed",&log);
	    //ROS_INFO_STREAM(log);
	    dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Acceleration",&log);
    	//ROS_INFO_STREAM(log);    
        for(int iter = 0; iter < DXL_COUNT-2; iter++){
            dxl_wb.writeRegister(dxl_id[iter], uint16_t(26), uint16_t(1), &d_gain, &log);
            dxl_wb.writeRegister(dxl_id[iter], uint16_t(27), uint16_t(1), &i_gain, &log);
            dxl_wb.writeRegister(dxl_id[iter], uint16_t(28), uint16_t(1), &p_gain, &log);
        }
    }
    //ROS_INFO_STREAM("Succeeded to init dxl");
    //std::cout<<"Succeeded to init dxl"<<std::endl;
    return(true);
}

//send positions
bool sync_write_pos(std::string manipulator_type){
    bool result = false;
    const char*log;
	if(manipulator_type != "delta"){    
	    result = dxl_wb.syncWrite((uint8_t)0, (uint8_t*)dxl_id, (uint8_t)6, 
	                            (int32_t*)goal_position, 1, &log);

	    if(result == false){
	        //ROS_INFO_STREAM("Failed to send goal position");
	        ROS_ERROR_STREAM(log);
	    }
	    result = dxl_wb.syncWrite((uint8_t)1, (uint8_t*)dxl_id, (uint8_t)6, 
	                            (int32_t*)goal_velocity, 1, &log);
	    if(result == false){
	        //ROS_INFO_STREAM("Failed to send goal velocity");
	        ROS_ERROR_STREAM(log);
	    }
	    result = dxl_wb.syncWrite((uint8_t)2, (uint8_t*)dxl_id, (uint8_t)4, 
	                            (int32_t*)goal_acceleration, 1, NULL);
	    if(result == false){
	        ROS_INFO_STREAM("Failed to send goal acceleration");
	    }
	}
    else{
	    for(int cnt=0; cnt<DXL_COUNT; cnt++){
	    	// result = dxl_wb.goalPosition(dxl_id[cnt], goal_position[cnt], &log);
	    	result = dxl_wb.writeRegister(dxl_id[cnt], "Goal_Position", goal_position[cnt], &log);
	    	result = dxl_wb.writeRegister(dxl_id[cnt], "Moving_Speed", goal_velocity[cnt], &log);
	    	result = dxl_wb.writeRegister(dxl_id[cnt], "Goal_Acceleration", goal_acceleration[cnt], &log);
	    	
	    }
	}
    return(result);
}
//enabling torque for motor with current id
bool enableTorque(uint8_t id){
    const char*log;
    bool result = false;
    result = dxl_wb.torqueOn(id, &log);
    if(result == false){
        //ROS_ERROR_STREAM("Failed to enable Torque on id: "<<(int)id);
        ROS_ERROR_STREAM(log);
    }
    else{
        for(int i;i<DXL_COUNT;i++){
            //goal_position[i] = (int32_t)present_position[i];
        }
    } 
    return (result);
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
    return(result);
}

//read position of motor with current id
bool read_pos(uint8_t id){
      const char*log;
      bool result = false;
      int32_t get_data = 0;
      result = dxl_wb.itemRead(id, "Present_Position", &get_data, &log);
      
      if (result == false)
      {
          //ROS_ERROR_STREAM(log);
      	  //ROS_ERROR_STREAM("Failed to get present position on id = " << (int)id);
          return(false);
      }
      present_position[id-1] = get_data;
      return(true);
}
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
    present_speed[id-1] = get_data;
    return(true);
}

bool read_temp(uint8_t id){
    const char*log;
    bool result = false;
    int32_t get_data = 0;
    result = dxl_wb.itemRead(id,"Present_Temperature", &get_data, &log);
    if(result == false)
    {

        //ROS_ERROR_STREAM(log);
        return(false);
    }
    present_temp[id-1] = get_data;
    return(true);
}

bool read_load(uint8_t id){
    const char*log;
    bool result = false;
    int32_t get_data = 0;
    result = dxl_wb.itemRead(id,"Present_Load", &get_data, &log);
    if(result == false)
    {

        //ROS_ERROR_STREAM(log);
        return(false);
    }
    present_load[id-1] = get_data;
    return(true);
}
//get time in milliseconds
double millis()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    return (te.tv_sec + (te.tv_usec / 1000000.0)) * 1000.0;
}

void messageJointscmd(const sensor_msgs::JointState::ConstPtr& toggle_msg)  //controll position
{	
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_position[i] = (int)toggle_msg->position[i];
    }
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_velocity[i] = (int)toggle_msg->velocity[i];
    }
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_acceleration[i] = (int)toggle_msg->effort[i];
    }
}

void messageTorqueMotor(const std_msgs::Bool::ConstPtr& msg){
    disable_flag = msg->data;
}

bool set_manipulator_state(std::string manipulator_type){
    if(manipulator_type == "palletizer"){
        DXL_COUNT = 5;
        dxl_id = new uint8_t[DXL_COUNT];
        goal_position = new int32_t[DXL_COUNT];
        goal_velocity = new int32_t[DXL_COUNT];
        goal_acceleration = new int32_t[DXL_COUNT-2];
        uint8_t val_id[] = {1,2,3,4,5};
        int32_t val_pos[] = {2048,1930,2230,512,512};
        int32_t val_vel[] = {40,40,40,40,40};
        int32_t val_acc[] = {2,2,2};
        for(int i = 0; i<DXL_COUNT;i++){
            dxl_id[i] = (uint8_t)val_id[i];
            goal_position[i] = val_pos[i];
            goal_velocity[i] = val_vel[i];
            goal_acceleration[i] = val_acc[i];
            present_load.push_back(0);
            present_temp.push_back(0);
            present_speed.push_back(0);
            present_position.push_back(0);
        }
	return true;
    }
    if(manipulator_type == "angle"){
        DXL_COUNT = 6;
        dxl_id = new uint8_t[DXL_COUNT];
        goal_position = new int32_t[DXL_COUNT];
        goal_velocity = new int32_t[DXL_COUNT];
        goal_acceleration = new int32_t[DXL_COUNT-2];
        static uint8_t val_id[] = {1,2,3,4,5,6};
        static int32_t val_pos[] = {2048, 2048, 2048, 2048, 512, 680};
        static int32_t val_vel[] = {40,40,40,40,40,40};
        static int32_t val_acc[] = {2,2,2,2};
        for(int i = 0; i<DXL_COUNT;i++){
            dxl_id[i] = val_id[i];
            goal_position[i] = val_pos[i];
            goal_velocity[i] = val_vel[i];
            goal_acceleration[i] = val_acc[i];
            present_load.push_back(0);
            present_temp.push_back(0);
            present_speed.push_back(0);
            present_position.push_back(0);
        }
        return true;
    }
    if(manipulator_type == "delta"){
        DXL_COUNT = 3;
        dxl_id = new uint8_t[DXL_COUNT];
        goal_position = new int32_t[DXL_COUNT];
        goal_velocity = new int32_t[DXL_COUNT];
        goal_acceleration = new int32_t[DXL_COUNT];
        static uint8_t val_id[] = {1,2,3};
        static int32_t val_pos[] = {2048, 2048, 2048};
        static int32_t val_vel[] = {40,40,40};
        static int32_t val_acc[] = {2,2,2};
        for(int i = 0; i<DXL_COUNT;i++){
            dxl_id[i] = val_id[i];
            printf("id: %d \n", dxl_id[i]);
            goal_position[i] = val_pos[i];
            goal_velocity[i] = val_vel[i];
            goal_acceleration[i] = val_acc[i];
            present_load.push_back(0);
            present_temp.push_back(0);
            present_speed.push_back(0);
            present_position.push_back(0);
        }
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{   
    bool result = false;
    bool err_flag = false;
    int iter = 0;
    
    ms0 = millis();
    ros::init(argc,argv, "Dxl_Arm");
    ros::NodeHandle nh;
    bool param;
    sensor_msgs::JointState joints_msg;
    sensor_msgs::JointState temp_load;
    ros::Subscriber jointcmd = nh.subscribe<sensor_msgs::JointState> ("cmd_joints",10, messageJointscmd );
    ros::Subscriber torque_motor = nh.subscribe<std_msgs::Bool> ("disable_torque",10, messageTorqueMotor);
    ros::Publisher Joint_State = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 64);
    ros::Publisher Temp_And_Load = nh.advertise<sensor_msgs::JointState>("temp_load", 64);

    ros::Rate loop_rate(PERIOD_PROTOCOL);
    std::string manipulator_type;
    // manipulator_type = "delta";
    nh.getParam("/arm/manipulator_type",manipulator_type);
    //std::cout<<"take param"<<manipulator_type<<std::endl;
    //ROS_INFO_STREAM("take param:"<<manipulator_type);
    result = set_manipulator_state(manipulator_type);
    if(result == false){
    	return 1;
    }
    result = false;
    while(result == false)
        result = dxl_init(manipulator_type);

    int error_count = 0;
    bool disable_prev_flag = false;
    struct timeval t_s,t_c;
    gettimeofday(&t_s, NULL);
    //std::cout<<"start";
    while(ros::ok()){
        for(int i = 0; i < DXL_COUNT; i++){
            result = read_pos(dxl_id[i]);
            result = read_vel(dxl_id[i]);
        }
        ms = millis();
        double delta = ms-ms0;
        if(delta >= PERIOD_MOVEMENT && err_flag == false && disable_flag == false){
            err_flag = !sync_write_pos(manipulator_type);
            err_flag = false;
            error_count += err_flag;
        }
        if (error_count>32)
            return 0;
        if((disable_flag == true) and (disable_prev_flag == false)){
            for(int i = 0; i < DXL_COUNT; i++)
                disableTorque(dxl_id[i]);
            disable_prev_flag = true;
        }else if((disable_flag == false) and (disable_prev_flag == true)){
            for(int i = 0; i < DXL_COUNT; i++)
                enableTorque(dxl_id[i]);
            disable_prev_flag = false;
        }
        ms0 = millis();
        joints_msg.velocity = present_speed;
        joints_msg.position = present_position;
        joints_msg.header.stamp = ros::Time::now();
        Joint_State.publish(joints_msg);
        gettimeofday(&t_c, NULL);
        if(t_c.tv_sec - t_s.tv_sec>1.0){
            for(int i = 0; i < DXL_COUNT; i++){
                result = read_temp(dxl_id[i]);
                result = read_load(dxl_id[i]);
            }
            temp_load.velocity = present_load;
            temp_load.position = present_temp;
            temp_load.header.stamp = ros::Time::now();
            Temp_And_Load.publish(temp_load);
            t_s = t_c;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    for(int i = 0; i < DXL_COUNT; i++)
        disableTorque(dxl_id[i]);
    return result;
}
