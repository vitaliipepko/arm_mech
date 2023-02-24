#include "arm_controller/arm_interface.h"
#include "arm_controller/AnglesConverter.h"
#include <std_msgs/UInt16MultiArray.h>

ArmInterface::ArmInterface(ros::NodeHandle& nh) : nh_(nh),
        pnh_("~"),
        pos_(26, 0),
        vel_(26, 0),
        eff_(26, 0),
        cmd_(26, 0),
        names_{"joint_shoulder_Y", 
                "joint_shoulder_X", 
                "joint_elbow_Z", 
                "joint_elbow_X", 
                "joint_wrist_Z", 
                "joint_wrist_X", 
                "joint_thumb", 
                "joint_finger_thumb", 
                "joint_phalanx_thumb_1", 
                "joint_phalanx_thumb_2",
                "joint_lit_finger",
                "joint_phalanx_lit_finger_1", 
                "joint_phalanx_lit_finger_2",
                "joint_phalanx_lit_finger_3",
                "joint_ring_finger",
                "joint_phalanx_ring_finger_1",
                "joint_phalanx_ring_finger_2",
                "joint_phalanx_ring_finger_3",
                "joint_mid_finger",
                "joint_phalanx_mid_finger_1",
                "joint_phalanx_mid_finger_2",
                "joint_phalanx_mid_finger_3",
                "joint_forefinger",
                "joint_phalanx_forefinger_1",
                "joint_phalanx_forefinger_2",
                "joint_phalanx_forefinger_3"}
{
    pnh_.param("joints_names", names_,names_);

    hardware_pub_ = pnh_.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);
    hardware_srv_ = pnh_.serviceClient<arm_controller::AnglesConverter>("/radians_to_degrees");

    ROS_INFO("Starting Arm Hardware Interface");

    hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    joint_state_interface_.registerHandle(state_handle1);

    hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    joint_state_interface_.registerHandle(state_handle2);

    hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    joint_state_interface_.registerHandle(state_handle3);

    hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    joint_state_interface_.registerHandle(state_handle4);

    hardware_interface::JointStateHandle state_handle5(names_.at(4), &pos_.at(4), &vel_.at(4), &eff_.at(4));
    joint_state_interface_.registerHandle(state_handle5);

    hardware_interface::JointStateHandle state_handle6(names_.at(5), &pos_.at(5), &vel_.at(5), &eff_.at(5));
    joint_state_interface_.registerHandle(state_handle6);

    hardware_interface::JointStateHandle state_handle7(names_.at(6), &pos_.at(6), &vel_.at(6), &eff_.at(6));
    joint_state_interface_.registerHandle(state_handle7);

    hardware_interface::JointStateHandle state_handle8(names_.at(7), &pos_.at(7), &vel_.at(7), &eff_.at(7));
    joint_state_interface_.registerHandle(state_handle8);

    hardware_interface::JointStateHandle state_handle9(names_.at(8), &pos_.at(8), &vel_.at(8), &eff_.at(8));
    joint_state_interface_.registerHandle(state_handle9);

    hardware_interface::JointStateHandle state_handle10(names_.at(9), &pos_.at(9), &vel_.at(9), &eff_.at(9));
    joint_state_interface_.registerHandle(state_handle10);

    hardware_interface::JointStateHandle state_handle11(names_.at(10), &pos_.at(10), &vel_.at(10), &eff_.at(10));
    joint_state_interface_.registerHandle(state_handle11);

    hardware_interface::JointStateHandle state_handle12(names_.at(11), &pos_.at(11), &vel_.at(11), &eff_.at(11));
    joint_state_interface_.registerHandle(state_handle12);

    hardware_interface::JointStateHandle state_handle13(names_.at(12), &pos_.at(12), &vel_.at(12), &eff_.at(12));
    joint_state_interface_.registerHandle(state_handle13);

    hardware_interface::JointStateHandle state_handle14(names_.at(13), &pos_.at(13), &vel_.at(13), &eff_.at(13));
    joint_state_interface_.registerHandle(state_handle14);

    hardware_interface::JointStateHandle state_handle15(names_.at(14), &pos_.at(14), &vel_.at(14), &eff_.at(14));
    joint_state_interface_.registerHandle(state_handle15);

    hardware_interface::JointStateHandle state_handle16(names_.at(15), &pos_.at(15), &vel_.at(15), &eff_.at(15));
    joint_state_interface_.registerHandle(state_handle16);

    hardware_interface::JointStateHandle state_handle17(names_.at(16), &pos_.at(16), &vel_.at(16), &eff_.at(16));
    joint_state_interface_.registerHandle(state_handle17);

    hardware_interface::JointStateHandle state_handle18(names_.at(17), &pos_.at(17), &vel_.at(17), &eff_.at(17));
    joint_state_interface_.registerHandle(state_handle18);

    hardware_interface::JointStateHandle state_handle19(names_.at(18), &pos_.at(18), &vel_.at(18), &eff_.at(18));
    joint_state_interface_.registerHandle(state_handle19);

    hardware_interface::JointStateHandle state_handle20(names_.at(19), &pos_.at(19), &vel_.at(19), &eff_.at(19));
    joint_state_interface_.registerHandle(state_handle20);

    hardware_interface::JointStateHandle state_handle21(names_.at(20), &pos_.at(20), &vel_.at(20), &eff_.at(20));
    joint_state_interface_.registerHandle(state_handle21);

    hardware_interface::JointStateHandle state_handle22(names_.at(21), &pos_.at(21), &vel_.at(21), &eff_.at(21));
    joint_state_interface_.registerHandle(state_handle22);

    hardware_interface::JointStateHandle state_handle23(names_.at(22), &pos_.at(22), &vel_.at(22), &eff_.at(22));
    joint_state_interface_.registerHandle(state_handle23);

    hardware_interface::JointStateHandle state_handle24(names_.at(23), &pos_.at(23), &vel_.at(23), &eff_.at(23));
    joint_state_interface_.registerHandle(state_handle24);

    hardware_interface::JointStateHandle state_handle25(names_.at(24), &pos_.at(24), &vel_.at(24), &eff_.at(24));
    joint_state_interface_.registerHandle(state_handle25);

    hardware_interface::JointStateHandle state_handle26(names_.at(25), &pos_.at(25), &vel_.at(25), &eff_.at(25));
    joint_state_interface_.registerHandle(state_handle26);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    joint_position_interface_.registerHandle(position_handle1);

    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    joint_position_interface_.registerHandle(position_handle2);

    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    joint_position_interface_.registerHandle(position_handle3);

    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    joint_position_interface_.registerHandle(position_handle4);

    hardware_interface::JointHandle position_handle5(joint_state_interface_.getHandle(names_.at(4)), &cmd_.at(4));
    joint_position_interface_.registerHandle(position_handle5);

    hardware_interface::JointHandle position_handle6(joint_state_interface_.getHandle(names_.at(5)), &cmd_.at(5));
    joint_position_interface_.registerHandle(position_handle6);

    hardware_interface::JointHandle position_handle7(joint_state_interface_.getHandle(names_.at(6)), &cmd_.at(6));
    joint_position_interface_.registerHandle(position_handle7);

    hardware_interface::JointHandle position_handle8(joint_state_interface_.getHandle(names_.at(7)), &cmd_.at(7));
    joint_position_interface_.registerHandle(position_handle8);

    hardware_interface::JointHandle position_handle9(joint_state_interface_.getHandle(names_.at(8)), &cmd_.at(8));
    joint_position_interface_.registerHandle(position_handle9);

    hardware_interface::JointHandle position_handle10(joint_state_interface_.getHandle(names_.at(9)), &cmd_.at(9));
    joint_position_interface_.registerHandle(position_handle10);

    hardware_interface::JointHandle position_handle11(joint_state_interface_.getHandle(names_.at(10)), &cmd_.at(10));
    joint_position_interface_.registerHandle(position_handle11);

    hardware_interface::JointHandle position_handle12(joint_state_interface_.getHandle(names_.at(11)), &cmd_.at(11));
    joint_position_interface_.registerHandle(position_handle12);

    hardware_interface::JointHandle position_handle13(joint_state_interface_.getHandle(names_.at(12)), &cmd_.at(12));
    joint_position_interface_.registerHandle(position_handle13);

    hardware_interface::JointHandle position_handle14(joint_state_interface_.getHandle(names_.at(13)), &cmd_.at(13));
    joint_position_interface_.registerHandle(position_handle14);

    hardware_interface::JointHandle position_handle15(joint_state_interface_.getHandle(names_.at(14)), &cmd_.at(14));
    joint_position_interface_.registerHandle(position_handle15);

    hardware_interface::JointHandle position_handle16(joint_state_interface_.getHandle(names_.at(15)), &cmd_.at(15));
    joint_position_interface_.registerHandle(position_handle16);

    hardware_interface::JointHandle position_handle17(joint_state_interface_.getHandle(names_.at(16)), &cmd_.at(16));
    joint_position_interface_.registerHandle(position_handle17);

    hardware_interface::JointHandle position_handle18(joint_state_interface_.getHandle(names_.at(17)), &cmd_.at(17));
    joint_position_interface_.registerHandle(position_handle18);

    hardware_interface::JointHandle position_handle19(joint_state_interface_.getHandle(names_.at(18)), &cmd_.at(18));
    joint_position_interface_.registerHandle(position_handle19);

    hardware_interface::JointHandle position_handle20(joint_state_interface_.getHandle(names_.at(19)), &cmd_.at(19));
    joint_position_interface_.registerHandle(position_handle20);

    hardware_interface::JointHandle position_handle21(joint_state_interface_.getHandle(names_.at(20)), &cmd_.at(20));
    joint_position_interface_.registerHandle(position_handle21);

    hardware_interface::JointHandle position_handle22(joint_state_interface_.getHandle(names_.at(21)), &cmd_.at(21));
    joint_position_interface_.registerHandle(position_handle22);

    hardware_interface::JointHandle position_handle23(joint_state_interface_.getHandle(names_.at(22)), &cmd_.at(22));
    joint_position_interface_.registerHandle(position_handle23);

    hardware_interface::JointHandle position_handle24(joint_state_interface_.getHandle(names_.at(23)), &cmd_.at(23));
    joint_position_interface_.registerHandle(position_handle24);

    hardware_interface::JointHandle position_handle25(joint_state_interface_.getHandle(names_.at(24)), &cmd_.at(24));
    joint_position_interface_.registerHandle(position_handle25);

    hardware_interface::JointHandle position_handle26(joint_state_interface_.getHandle(names_.at(25)), &cmd_.at(25));
    joint_position_interface_.registerHandle(position_handle26);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &ArmInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}


void ArmInterface::update(const ros::TimerEvent& e)
{
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ArmInterface::read()
{
    pos_.at(0) = cmd_.at(0);
    pos_.at(1) = cmd_.at(1);
    pos_.at(2) = cmd_.at(2);
    pos_.at(3) = cmd_.at(3);
    pos_.at(4) = cmd_.at(4);
    pos_.at(5) = cmd_.at(5);
    pos_.at(6) = cmd_.at(6);
    pos_.at(7) = cmd_.at(7);
    pos_.at(8) = cmd_.at(8);
    pos_.at(9) = cmd_.at(9);
    pos_.at(10) = cmd_.at(10);
    pos_.at(11) = cmd_.at(11);
    pos_.at(12) = cmd_.at(12);
    pos_.at(13) = cmd_.at(13);
    pos_.at(14) = cmd_.at(14);
    pos_.at(15) = cmd_.at(15);
    pos_.at(16) = cmd_.at(16);
    pos_.at(17) = cmd_.at(17);
    pos_.at(18) = cmd_.at(18);
    pos_.at(19) = cmd_.at(19);
    pos_.at(20) = cmd_.at(20);
    pos_.at(21) = cmd_.at(21);
    pos_.at(22) = cmd_.at(22);
    pos_.at(23) = cmd_.at(23);
    pos_.at(24) = cmd_.at(24);
    pos_.at(25) = cmd_.at(25);
}

void ArmInterface::write(ros::Duration elapsed_time)
{   
    arm_controller::AnglesConverter srv;
    srv.request.shoulder_Y = cmd_.at(0);
    srv.request.shoulder_X = cmd_.at(1);
    srv.request.elbow_Z = cmd_.at(2);
    srv.request.elbow_X = cmd_.at(3);
    srv.request.wrist_Z = cmd_.at(4);
    srv.request.wrist_X = cmd_.at(5);
    srv.request.thumb = cmd_.at(6);
    srv.request.finger_thumb = cmd_.at(7);
    srv.request.phalanx_thumb_1 = cmd_.at(8);
    srv.request.phalanx_thumb_2 = cmd_.at(9);
    srv.request.lit_finger = cmd_.at(10);
    srv.request.phalanx_lit_finger_1 = cmd_.at(11);
    srv.request.phalanx_lit_finger_2 = cmd_.at(12);
    srv.request.phalanx_lit_finger_3 = cmd_.at(13);
    srv.request.ring_finger = cmd_.at(14);
    srv.request.phalanx_ring_finger_1 = cmd_.at(15);
    srv.request.phalanx_ring_finger_2 = cmd_.at(16);
    srv.request.phalanx_ring_finger_3 = cmd_.at(17);
    srv.request.mid_finger = cmd_.at(18);
    srv.request.phalanx_mid_finger_1 = cmd_.at(19);
    srv.request.phalanx_mid_finger_2 = cmd_.at(20);
    srv.request.phalanx_mid_finger_3 = cmd_.at(21);
    srv.request.forefinger = cmd_.at(22);
    srv.request.phalanx_forefinger_1 = cmd_.at(23);
    srv.request.phalanx_forefinger_2 = cmd_.at(24);
    srv.request.phalanx_forefinger_3 = cmd_.at(25);

    if (hardware_srv_.call(srv))
    {
        std::vector<unsigned int> angles_deg;
        angles_deg.push_back(srv.response.shoulder_Y);
        angles_deg.push_back(srv.response.shoulder_X);
        angles_deg.push_back(srv.response.elbow_Z);
        angles_deg.push_back(srv.response.elbow_X);
        angles_deg.push_back(srv.response.wrist_Z);
        angles_deg.push_back(srv.response.wrist_X);
        angles_deg.push_back(srv.response.thumb);
        angles_deg.push_back(srv.response.finger_thumb);
        angles_deg.push_back(srv.response.phalanx_thumb_1);
        angles_deg.push_back(srv.response.phalanx_thumb_2);
        angles_deg.push_back(srv.response.lit_finger);
        angles_deg.push_back(srv.response.phalanx_lit_finger_1);
        angles_deg.push_back(srv.response.phalanx_lit_finger_2);
        angles_deg.push_back(srv.response.phalanx_lit_finger_3);
        angles_deg.push_back(srv.response.ring_finger);
        angles_deg.push_back(srv.response.phalanx_ring_finger_1);
        angles_deg.push_back(srv.response.phalanx_ring_finger_2);
        angles_deg.push_back(srv.response.phalanx_ring_finger_3);
        angles_deg.push_back(srv.response.mid_finger);
        angles_deg.push_back(srv.response.phalanx_mid_finger_1);
        angles_deg.push_back(srv.response.phalanx_mid_finger_2);
        angles_deg.push_back(srv.response.phalanx_mid_finger_3);
        angles_deg.push_back(srv.response.forefinger);
        angles_deg.push_back(srv.response.phalanx_forefinger_1);
        angles_deg.push_back(srv.response.phalanx_forefinger_2);
        angles_deg.push_back(srv.response.phalanx_forefinger_3);


        std_msgs::UInt16MultiArray msg;

        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = angles_deg.size();
        msg.layout.dim[0].stride = 1;

        msg.data.clear();
        msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

        hardware_pub_.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to call service radians_to_degrees");
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArmInterface robot(nh);

    spinner.spin();

    return 0;
}
