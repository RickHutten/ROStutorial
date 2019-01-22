#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/TransformStamped.h"


class ReceiverTranslator {
public:
    ReceiverTranslator();
    
private:
    void listenerLoop();

    ros::NodeHandle node;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
};

ReceiverTranslator::ReceiverTranslator() {
    this->listenerLoop();
}

void ReceiverTranslator::listenerLoop() {
    ros::Rate rate(1);
    while (node.ok()){
        tf::StampedTransform transformENU;
        try{
            listener.waitForTransform("world", "world/base_tf_enu",
                ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("world", "world/base_tf_enu",
                ros::Time(0), transformENU);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        tf::Quaternion quaternionENU = transformENU.getRotation();
        
        // Convert ENU -> NED
        tf::Quaternion quaternionNED;
        quaternionNED.setValue(
            quaternionENU.y(),
            quaternionENU.x(),
            -quaternionENU.z(),
            quaternionENU.w()
        );
        tf::StampedTransform transformNED = transformENU;
        transformNED.setRotation(quaternionNED);
        transformNED.child_frame_id_ = "world/tf_ned";

        // Convert ENU -> NWU
        tf::Quaternion quaternionNWU;
        quaternionNWU.setValue(
            quaternionENU.y(),
            -quaternionENU.x(),
            quaternionENU.z(),
            quaternionENU.w()
        );
        tf::StampedTransform transformNWU = transformENU;
        transformNWU.setRotation(quaternionNWU);
        transformNWU.child_frame_id_ = "world/tf_nwu";
        
        // Send transforms
        this->broadcaster.sendTransform(transformNED);
        this->broadcaster.sendTransform(transformNWU);

        std::cout << "Transforms send" << std::endl;
        rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_receiver_translator");

    ReceiverTranslator translator;

    return 0;
};
