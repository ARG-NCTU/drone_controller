#include <iostream>
#include <string> 
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
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
        ros::Publisher pub_twist;

        ros::Subscriber sub_linear_x;
        ros::Subscriber sub_linear_y;
        ros::Subscriber sub_linear_z;
        ros::Subscriber sub_anglular_r;
        ros::Subscriber sub_anglular_p;
        ros::Subscriber sub_anglular_y;

        float m_twist[6] = {0, 0, 0, 0, 0, 0};
        int m_rate = 20;

    public:
        bt::Condition condition;
        Control();
        void linearXCallback(const std_msgs::Float32::ConstPtr& msg);
        void linearYCallback(const std_msgs::Float32::ConstPtr& msg);
        void linearZCallback(const std_msgs::Float32::ConstPtr& msg);
        void angularXCallback(const std_msgs::Float32::ConstPtr& msg);
        void angularYCallback(const std_msgs::Float32::ConstPtr& msg);
        void angularZCallback(const std_msgs::Float32::ConstPtr& msg);
        void getParam();
        void pubTwist();
};

Control :: Control() : condition("controller_running"){
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);

    sub_linear_x = n.subscribe<std_msgs::Float32>("linear_x_output", 1,  &Control::linearXCallback, this);
    sub_linear_y = n.subscribe<std_msgs::Float32>("linear_y_output", 1,  &Control::linearYCallback, this);
    sub_linear_z = n.subscribe<std_msgs::Float32>("linear_z_output", 1,  &Control::linearZCallback, this);

    sub_anglular_r = n.subscribe<std_msgs::Float32>("angular_x_output", 1,  &Control::angularXCallback, this);
    sub_anglular_p = n.subscribe<std_msgs::Float32>("angular_y_output", 1,  &Control::angularYCallback, this);
    sub_anglular_y = n.subscribe<std_msgs::Float32>("angular_z_output", 1,  &Control::angularZCallback, this);
}


void Control :: linearXCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[0] = msg->data; return; }
void Control :: linearYCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[1] = msg->data; return; }
void Control :: linearZCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[2] = msg->data; return; }
void Control :: angularXCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[3] = msg->data; return; }
void Control :: angularYCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[4] = msg->data; return; }
void Control :: angularZCallback(const std_msgs::Float32::ConstPtr& msg){ m_twist[5] = msg->data; return; }

void Control :: getParam(){
    string node_ns = ros::this_node::getName();
    n.getParam("/" + node_ns + "/rate", m_rate);
    return;
}

void Control :: pubTwist(){

    ros::Rate rate(m_rate);

    geometry_msgs::Twist pub_msg_twist;
    
    pub_msg_twist.linear.x = m_twist[0];
    pub_msg_twist.linear.y = m_twist[1];
    pub_msg_twist.linear.z = m_twist[2];
    pub_msg_twist.angular.x = m_twist[3];
    pub_msg_twist.angular.y = m_twist[4]; 
    pub_msg_twist.angular.z = m_twist[5]; 
    pub_twist.publish(pub_msg_twist);
    // if(m_twist[0] != 0 || m_twist[1] != 0 || m_twist[2] != 0 || m_twist[3] != 0 || m_twist[4] != 0 || m_twist[5] != 0){
    //     condition.set(true);
    //     condition.publish();
    // }else{
    //     condition.set(false);
    //     condition.publish();
    // }
    
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "fusion_controller");
    Control pid;
    while(ros::ok()){
        pid.pubTwist();
        ros::spinOnce();
    }
    return 0;
}
