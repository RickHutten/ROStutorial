#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream> 
#include <csignal>
#include <cstdlib>
#include <time.h>
#include <unistd.h>
#include <random>

class Terminator {
public:
    Terminator();
private:
	int signals[5] = {SIGINT, SIGTERM, SIGABRT, SIGSEGV, SIGFPE};
	ros::NodeHandle node;
	int programPid = 0;
	void pidListener(const std_msgs::Int32::ConstPtr&);
};

