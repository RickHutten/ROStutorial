#include "ros_signal/Handler.hpp"

static int chosen_signal;

Handler::Handler() {
    // Seed rng
    srand(time(NULL));

    ROS_INFO("Program PID: %d", getpid());    

    chosen_signal = this->get_random_signal();
    ROS_INFO("Chosen signal to exit: %d", chosen_signal); 
    
    // Set signal catchers
    int i;
    for (i = 0; i <=5; i++) {
        signal(signals[i], &Handler::signal_handler); 
    }
    
    ROS_INFO("Awaiting signals...");

    // Publish pid
    ros::Publisher publisher = node.advertise<std_msgs::Int32>("pid", 1000);
    ros::Rate rate(10);
    while(ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = getpid();
        publisher.publish(msg);
        
        rate.sleep();
    }
}

int Handler::get_random_signal() {
    return signals[rand() % 5];
}

void Handler::signal_handler(int signal_num) { 
    ROS_INFO("Interruption signal %d is sent", signal_num);

    if (signal_num == chosen_signal) {
        ROS_INFO("Exiting program");
        exit(0);
    }
    ROS_INFO("Received signal is not the chosen signal");
} 
