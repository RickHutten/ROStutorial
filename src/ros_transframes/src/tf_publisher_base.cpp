#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include </usr/include/boost/thread.hpp>

//#include <mutex>

#include "geometry_msgs/TransformStamped.h"

struct terminos; // Define struct to save and set terminal settings

class PublishBase {
public:
    PublishBase();
    void inputLoop();
    void broadcastLoop();

private:
    std::string getAnswer();
    void parseAnswer(std::string);

    ros::NodeHandle node;
    ros::Publisher pub;
    tf::TransformBroadcaster broadcaster;
    float parsedAnswer[6];
    //std::mutex mutex;
};

PublishBase::PublishBase() {
    pub = node.advertise<geometry_msgs::TransformStamped>("world/base_tf_enu", 1000);
}


void PublishBase::inputLoop() {
    while(ros::ok()) {
        std::string answer = this->getAnswer();
        std::cout << "parsing" << std::endl;
        this->parseAnswer(answer);
    }
}

void PublishBase::broadcastLoop() {

    ros::Rate rate(1);
    while(ros::ok()) {
        // Create message
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "test";
        msg.child_frame_id = "test2";
        msg.transform.translation.x = 0;
        msg.transform.translation.y = 1;
        msg.transform.translation.z = 0;

        this->broadcaster.sendTransform(msg);
        rate.sleep();
    }
}


std::string PublishBase::getAnswer() {
    std::string answer;
    std::cout << "x,y,z,roll,pitch,yaw:" << std::endl;
    std::cin >> answer;
    return answer;
}

void PublishBase::parseAnswer(std::string answer) {
    int pos = 0;
    int count = 0;
    std::string token;
    std::string delimiter = ",";
    while ((pos = answer.find(delimiter)) != std::string::npos) {
        token = answer.substr(0, pos);
        std::cout << token << std::endl;
        answer.erase(0, pos + delimiter.length());
        std::istringstream(token) >> this->parsedAnswer[count];
        count++;
    }

    std::cout << "Parsed answer " << this->parsedAnswer[0] << " " << this->parsedAnswer[1] << " " << this->parsedAnswer[2] << " " << this->parsedAnswer[3] << " " << this->parsedAnswer[4] << " " << this->parsedAnswer[5] << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_base");

    PublishBase publishBase;
    
    // Start both input and broadcast loop
    boost::thread inputThread(&PublishBase::inputLoop, &publishBase);
    publishBase.broadcastLoop();    
}
