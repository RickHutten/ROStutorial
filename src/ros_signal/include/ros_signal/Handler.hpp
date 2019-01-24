#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream> 
#include <csignal>
#include <cstdlib>
#include <time.h>
#include <unistd.h>

class Handler {
public:
    Handler();
private:
    int signals[5] = {SIGINT, SIGTERM, SIGABRT, SIGSEGV, SIGFPE};
    int get_random_signal();
    static void signal_handler(int);
    ros::NodeHandle node;
};
