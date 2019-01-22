#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "geometry_msgs/TransformStamped.h"


class PublishBase {
public:
    PublishBase();
    void inputLoop();
    void broadcastLoop();

private:
    std::string getAnswer();
    void parseAnswer(std::string);

    ros::NodeHandle node;
    float parsedAnswer[6] = {0};
    tf::TransformBroadcaster broadcaster;
};

PublishBase::PublishBase() {
    // Start both input and broadcast loop
    boost::thread inputThread(&PublishBase::inputLoop, this);
    this->broadcastLoop(); 
}

void PublishBase::inputLoop() {
    while(ros::ok()) {
        std::string answer = this->getAnswer();
        this->parseAnswer(answer);
    }
}

void PublishBase::broadcastLoop() {
    ros::Rate rate(1);
    while(ros::ok()) {
        tf::Transform transform;

        // Set position
        transform.setOrigin(
            tf::Vector3(
                this->parsedAnswer[0], 
                this->parsedAnswer[1], 
                this->parsedAnswer[2]
            ) 
        );

        // Set rotation
        tf::Quaternion quaternion;
        quaternion.setRPY(
            this->parsedAnswer[3], 
            this->parsedAnswer[4], 
            this->parsedAnswer[5] 
        );
        transform.setRotation(quaternion);

        this->broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "world", "world/base_tf_enu")
        );

        rate.sleep();
    }
}

std::string PublishBase::getAnswer() {
    std::string answer;
    std::cout << "x,y,z,roll,pitch,yaw:" << std::endl;
    std::getline(std::cin, answer);
    return answer;
}

void PublishBase::parseAnswer(std::string answer) {
    std::vector<std::string> split;
    boost::split(split, answer, boost::is_any_of(","));
    
    // Check if the answer contains exactly 6 arguments
    if (split.size() != 6) {
        std::cout << "Not the correct amount of inputs" << std::endl;
        return;
    }

    int i;
    for (i = 0; i < 6; i ++) {
        // Try to parse answer, else input zero
        try{
            std::istringstream(split[i]) >> this->parsedAnswer[i];
        } catch(...) {
            this->parsedAnswer[i] = 0;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_base");

    PublishBase publishBase;
    
    return 0;  
}
