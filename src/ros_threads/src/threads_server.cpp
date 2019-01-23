#include <ros/ros.h>
#include <string>

#include "ros_threads/UnixTimeNow.h"

class ThreadsServer {
public:
    ThreadsServer();
private:
    bool callback(ros_threads::UnixTimeNow::Request&, ros_threads::UnixTimeNow::Response&);
    ros::NodeHandle node;
};

bool ThreadsServer::callback(ros_threads::UnixTimeNow::Request& req,
                             ros_threads::UnixTimeNow::Response& res) {
    long int delay = req.Delay_s;
    uint64_t time = ros::Time::now().toNSec();

    if (0 < delay && delay <= 3) {
        ros::Duration(delay).sleep();
    }
    res.Time = std::to_string(time);
    
    return true;
}


ThreadsServer::ThreadsServer() {
    ROS_INFO("Waiting for requests");

    ros::ServiceServer service = node.advertiseService("unix_time_now", &ThreadsServer::callback, this);
    ros::spin();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "threads_server");

    ThreadsServer threadsServer;
    
    return 0;  
}
