#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include <string>

struct terminos; // Define struct to save and set terminal settings

class PublishBase {
public:
    PublishBase();
    void loop();

private:
    void getAnswer();
    void parseAnswer();

    ros::NodeHandle node;
    ros::Publisher pub;
    tf::TransformBroadcaster broadcaster;
    std::string answer;
    std::string parsedAnswer[6];
};

PublishBase::PublishBase() {
    pub = node.advertise<geometry_msgs::TransformStamped>("world/base_tf_enu", 1000);
    this->getAnswer();
}

void PublishBase::loop() {

    ros::Rate rate(1);

    while(ros::ok())
    {
        // Create message
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "test";
        msg.child_frame_id = "test2";
        msg.transform.translation.x = 0;
        msg.transform.translation.y = 1;
        msg.transform.translation.z = 0;

        // Publish transform over tf
        //broadcaster.sendTransform(msg);

        // Publish over ROS
        pub.publish(msg);

        ROS_INFO("SENDING TRANSFORM");

        rate.sleep();
    }
};

void PublishBase::getAnswer() {
    std::cout << "x,y,z,roll,pitch,yaw:" << std::endl;
    std::cin >> answer;
    ROS_INFO("Your answer: %s", answer);
}

void PublishBase::parseAnswer() {
    std::string delimiter = ",";

    int pos = 0;
    int count = 0;
    std::string token;
    while ((pos = this->answer.find(delimiter)) != std::string::npos) {
        token = answer.substr(0, pos);
        std::cout << token << std::endl;
        answer.erase(0, pos + delimiter.length());
        parsedAnswer[count] = token;
        count++;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_base");

    PublishBase publishBase;
    publishBase.loop();

}
