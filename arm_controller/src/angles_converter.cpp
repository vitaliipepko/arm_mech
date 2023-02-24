#include "ros/ros.h"
#include "arm_controller/AnglesConverter.h"
#include "math.h"


bool convert_radians_to_degrees(arm_controller::AnglesConverter::Request  &req,
         arm_controller::AnglesConverter::Response &res)
{
    res.shoulder_Y = static_cast<int>(((req.shoulder_Y+(M_PI/2))*180)/M_PI);
    res.shoulder_X = static_cast<int>(((req.shoulder_X+(M_PI/2))*180)/M_PI);
    res.elbow_Z = static_cast<int>(((req.elbow_Z+(M_PI/2))*180)/M_PI);
    res.elbow_X = 180-static_cast<int>(((req.elbow_X+(M_PI/2))*180)/M_PI);
    res.wrist_Z = static_cast<int>(((req.wrist_Z+(M_PI/2))*180)/M_PI);
    res.wrist_X = static_cast<int>(((req.wrist_X+(M_PI/2))*180)/M_PI);
    res.thumb = 180-static_cast<int>(((req.thumb+(M_PI/2))*180)/M_PI);
    res.finger_thumb = 180-static_cast<int>(((req.finger_thumb+(M_PI/2))*180)/M_PI);
    res.phalanx_thumb_1 = 180-static_cast<int>(((req.phalanx_thumb_1+(M_PI/2))*180)/M_PI);
    res.phalanx_thumb_2 = 180-static_cast<int>(((req.phalanx_thumb_2+(M_PI/2))*180)/M_PI);
    res.lit_finger = 180-static_cast<int>(((req.lit_finger+(M_PI/2))*180)/M_PI);
    res.phalanx_lit_finger_1 = 180-static_cast<int>(((req.phalanx_lit_finger_1+(M_PI/2))*180)/M_PI);
    res.phalanx_lit_finger_2 = 180-static_cast<int>(((req.phalanx_lit_finger_2+(M_PI/2))*180)/M_PI);
    res.phalanx_lit_finger_3 = 180-static_cast<int>(((req.phalanx_lit_finger_3+(M_PI/2))*180)/M_PI);
    res.ring_finger = 180-static_cast<int>(((req.ring_finger+(M_PI/2))*180)/M_PI);
    res.phalanx_ring_finger_1 = 180-static_cast<int>(((req.phalanx_ring_finger_1+(M_PI/2))*180)/M_PI);
    res.phalanx_ring_finger_2 = 180-static_cast<int>(((req.phalanx_ring_finger_2+(M_PI/2))*180)/M_PI);
    res.phalanx_ring_finger_3 = 180-static_cast<int>(((req.phalanx_ring_finger_3+(M_PI/2))*180)/M_PI);
    res.mid_finger = 180-static_cast<int>(((req.mid_finger+(M_PI/2))*180)/M_PI);
    res.phalanx_mid_finger_1 = 180-static_cast<int>(((req.phalanx_mid_finger_1+(M_PI/2))*180)/M_PI);
    res.phalanx_mid_finger_2 = 180-static_cast<int>(((req.phalanx_mid_finger_2+(M_PI/2))*180)/M_PI);
    res.phalanx_mid_finger_3 = 180-static_cast<int>(((req.phalanx_mid_finger_3+(M_PI/2))*180)/M_PI);
    res.forefinger = 180-static_cast<int>(((req.forefinger+(M_PI/2))*180)/M_PI);
    res.phalanx_forefinger_1 = 180-static_cast<int>(((req.phalanx_forefinger_1+(M_PI/2))*180)/M_PI);
    res.phalanx_forefinger_2 = 180-static_cast<int>(((req.phalanx_forefinger_2+(M_PI/2))*180)/M_PI);
    res.phalanx_forefinger_3 = 180-static_cast<int>(((req.phalanx_forefinger_3+(M_PI/2))*180)/M_PI);
    return true;
}

bool convert_degrees_to_radians(arm_controller::AnglesConverter::Request  &req,
         arm_controller::AnglesConverter::Response &res)
{
    res.shoulder_Y = (((req.shoulder_Y+(M_PI/2))*180)/M_PI);
    res.shoulder_X = (((req.shoulder_X+(M_PI/2))*180)/M_PI);
    res.elbow_Z = (((req.elbow_Z+(M_PI/2))*180)/M_PI);
    res.elbow_X = (((180-req.elbow_X)*M_PI)-((M_PI/2)*180))/180;
    res.wrist_Z = (((req.wrist_Z+(M_PI/2))*180)/M_PI);
    res.wrist_X = (((req.wrist_X+(M_PI/2))*180)/M_PI);
    res.thumb = (((180-req.elbow_X)*M_PI)-((M_PI/2)*180))/180;
    res.finger_thumb = (((180-req.finger_thumb)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_thumb_1 = (((180-req.phalanx_thumb_1)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_thumb_2 = (((180-req.phalanx_thumb_2)*M_PI)-((M_PI/2)*180))/180;
    res.lit_finger = (((180-req.lit_finger)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_lit_finger_1 = (((180-req.phalanx_lit_finger_1)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_lit_finger_2 = (((180-req.phalanx_lit_finger_2)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_lit_finger_3 = (((180-req.phalanx_lit_finger_3)*M_PI)-((M_PI/2)*180))/180;
    res.ring_finger = (((180-req.ring_finger)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_ring_finger_1 = (((180-req.phalanx_ring_finger_1)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_ring_finger_2 = (((180-req.phalanx_ring_finger_2)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_ring_finger_3 = (((180-req.phalanx_ring_finger_3)*M_PI)-((M_PI/2)*180))/180;
    res.mid_finger = (((180-req.mid_finger)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_mid_finger_1 = (((180-req.phalanx_mid_finger_1)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_mid_finger_2 = (((180-req.phalanx_mid_finger_2)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_mid_finger_3 = (((180-req.phalanx_mid_finger_3)*M_PI)-((M_PI/2)*180))/180;
    res.forefinger = (((180-req.forefinger)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_forefinger_1 = (((180-req.phalanx_forefinger_1)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_forefinger_2 = (((180-req.phalanx_forefinger_2)*M_PI)-((M_PI/2)*180))/180;
    res.phalanx_forefinger_3 = (((180-req.phalanx_forefinger_3)*M_PI)-((M_PI/2)*180))/180;
    return true;
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "angles_converter");
    ros::NodeHandle n;

    ros::ServiceServer radians_to_degrees = n.advertiseService("radians_to_degrees", convert_radians_to_degrees);
    ros::ServiceServer degrees_to_radians = n.advertiseService("degrees_to_radians", convert_degrees_to_radians);
    
    ROS_INFO("Angles Converter Service Started");

    ros::spin();

    return 0;
}
