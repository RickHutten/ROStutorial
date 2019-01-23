#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include "ros_threads/UnixTimeNow.h"

class ThreadsClient {
public:
    ThreadsClient();
private:
    void printLoop();
    void requestLoop();
    void callService();

    ros::NodeHandle node;
    boost::random::mt19937 rng;
};

ThreadsClient::ThreadsClient() {

    // Start both print and request loop
    boost::thread t(&ThreadsClient::printLoop, this);
    this->requestLoop(); 
}

void ThreadsClient::printLoop() {
    ros::Rate rate(2); // Rate in Hz
    while(ros::ok()) {
        ROS_INFO("Current Time: %ld", ros::Time::now().toNSec());
        rate.sleep();
    }
}

void ThreadsClient::requestLoop() {
    ros::Rate rate(1);
    while(ros::ok()) {
        callService();
        rate.sleep();
    }
}

void ThreadsClient::callService() {
    ros::ServiceClient client = node.serviceClient<ros_threads::UnixTimeNow>("unix_time_now");
    ros_threads::UnixTimeNow service;

    // Generate delay in seconds
    boost::random::uniform_int_distribution<> dist(0, 5);
    int delay = dist(rng);
    service.request.Delay_s = delay;

    // Call the service
    if (client.call(service)) {
        int64_t receivedTime_ns;
        std::istringstream(service.response.Time) >> receivedTime_ns;
        
        int64_t difference_ns = ros::Time::now().toNSec() - receivedTime_ns;
        int64_t difference_ms = difference_ns / 1000000;

        ROS_INFO("Time Difference: %ld", difference_ms);
    } else {
        ROS_ERROR("Failed to call service");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "threads_client");
    
    ThreadsClient threadsClient;

    return 0;  
}
