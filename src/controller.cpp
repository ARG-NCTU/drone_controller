#include <iostream>
#include <string> 
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <behavior_tree/behavior_tree.h>

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}


class Control{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_control_output;
        ros::Subscriber sub_desired_pose;
        ros::Subscriber sub_current_pose;

        std::string m_control_target = "";
        std::string m_control_type = "";

        bool m_control_enable = false;

        float m_desired_pose[7] = {0, 0, 0, 0, 0, 0, 0};
        float m_current_pose[7] = {0, 0, 0, 0, 0, 0, 0};

        float m_error_diff = 0;

        float m_P_param = 0.5;
        float m_I_param = 0;
        float m_D_param = 0;

        float m_x_offset = 0;
        float m_y_offset = 0;
        float m_z_offset = 0;

    public:
        bt::Condition condition;
        bt::Action action;
        Control();
        void getParam();
        void desiredPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w);
        void pubOutput(float value);
        void controller();
};

Control :: Control() : condition(appendString(ros::this_node::getName(), (string)"_control_required")), 
                        action(appendString(ros::this_node::getName(), (string)"_control")){
    pub_control_output = n.advertise<std_msgs::Float32>(appendString(ros::this_node::getName(), (string)"/output"), 1);
    sub_desired_pose = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1,  &Control::desiredPositionCallback, this);
    sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Control::currentPositionCallback, this);
}

void Control :: getParam(){
    string node_ns = ros::this_node::getName();

    n.getParam("/" + node_ns + "/control_type", m_control_type);
    n.getParam("/" + node_ns + "/control_target", m_control_target);

    n.getParam("/" + node_ns + "/P_param", m_P_param);
    n.getParam("/" + node_ns + "/I_param", m_I_param);
    n.getParam("/" + node_ns + "/D_param", m_D_param);

    n.getParam("/" + node_ns + "/x_offset", m_x_offset);
    n.getParam("/" + node_ns + "/y_offset", m_y_offset);
    n.getParam("/" + node_ns + "/z_offset", m_z_offset);
    // n.getParam("/" + node_ns + "/roll_offset", m_D_param);
    // n.getParam("/" + node_ns + "/pitch_offset", m_D_param);
    // n.getParam("/" + node_ns + "/yaw_offset", m_D_param);
    return;
}

void Control :: desiredPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(!m_control_enable){ m_control_enable = true; }
    m_desired_pose[0] = msg->pose.position.x + m_x_offset;
    m_desired_pose[1] = msg->pose.position.y + m_y_offset;
    m_desired_pose[2] = msg->pose.position.z + m_z_offset;

    float angular_x, angular_y, angular_z;
    quaternion2euler(
        &angular_x, 
        &angular_y, 
        &angular_z, 
        msg->pose.orientation.x, 
        msg->pose.orientation.y, 
        msg->pose.orientation.z, 
        msg->pose.orientation.w
    );
    m_desired_pose[3] = angular_x;
    m_desired_pose[4] = angular_y;
    m_desired_pose[5] = angular_z;

    return;
}

void Control :: currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_current_pose[0] = msg->pose.position.x;
    m_current_pose[1] = msg->pose.position.y;
    m_current_pose[2] = msg->pose.position.z;

    float angular_x, angular_y, angular_z;
    quaternion2euler(
        &angular_x, 
        &angular_y, 
        &angular_z, 
        msg->pose.orientation.x, 
        msg->pose.orientation.y, 
        msg->pose.orientation.z, 
        msg->pose.orientation.w
    );
    m_current_pose[3] = angular_x;
    m_current_pose[4] = angular_y;
    m_current_pose[5] = angular_z;

    return;
}

void Control :: quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w){
    double tmp_r, tmp_p, tmp_y;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(tmp_r, tmp_p, tmp_y);
    *roll = (float)tmp_r;
    *pitch = (float)tmp_p;
    *yaw = (float)tmp_y;
    return;
}

void Control :: pubOutput(float value){
    std_msgs::Float32 pub_msg;
    pub_msg.data = value;
    pub_control_output.publish(pub_msg);
    return;
}

void Control :: controller(){
    ros::Rate rate(50);
    if(m_control_enable){
        float error = 0, output = 0;
        if(m_control_type == "linear"){
            if(m_control_target == "x"){
                error = m_desired_pose[0] - m_current_pose[0];
            }else if(m_control_target == "y"){
                error = m_desired_pose[1] - m_current_pose[1];
            }else if(m_control_target == "z"){
                error = m_desired_pose[2] - m_current_pose[2];
            }else{
                ROS_INFO("ERROR CONTROL TARGET");
            }
        }
        else if(m_control_type == "angular"){
            if(m_control_target == "x"){
                error = m_desired_pose[3] - m_current_pose[3];
            }else if(m_control_target == "y"){
                error = m_desired_pose[4] - m_current_pose[4];
            }else if(m_control_target == "z"){

                error = m_desired_pose[5] - m_current_pose[5];
            }else{
                ROS_INFO("ERROR CONTROL TARGET");
            }
        }
        else{
            ROS_INFO("ERROR CONTROL TARGET");
        }
        

        output = error * m_P_param + m_error_diff * m_D_param;

        m_error_diff = error - m_error_diff;

        pubOutput(output);
    }
    rate.sleep();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    Control pid;
    pid.getParam();
    while(ros::ok()){
        pid.controller();
        ros::spinOnce();
    }
    return 0;
}
