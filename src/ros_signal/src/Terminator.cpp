#include "ros_signal/Terminator.hpp"

Terminator::Terminator() {
    ros::Subscriber sub = node.subscribe("pid", 1000, &Terminator::pidListener, this);
    ROS_INFO("Waiting for Handler to broadcast pid");
    
    // Spin untill pid is broadcast
    ros::Rate rate(2);
    while(ros::ok() && this->programPid == 0) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Gottem: %d", this->programPid);
    
    int count = 0;
    for(;;) {
        int signal = this->signals[rand() % 5];
        ROS_INFO("Trying signal %d", signal);
        
        // Send signal
        kill(this->programPid, signal);
        
        rate.sleep();
        
        // Check if pid is still active
        if (kill(this->programPid, 0) == -1) {
            ROS_INFO("Program is terminated, exiting..");
            break;
        }
    }
    
}

void Terminator::pidListener(const std_msgs::Int32::ConstPtr& msg) {
    
    this->programPid = msg->data;
}
