/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <utility>

class simple_controller {
public:
    int id;
    mavros_msgs::State current_state;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    geometry_msgs::PoseStamped pose;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    simple_controller() {}

    simple_controller(int id) : id(id) {}

    void state_cb(const mavros_msgs::State::ConstPtr &msg) {
        current_state = *msg;
    }

    void pub() {
        local_pos_pub.publish(pose);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ROS_INFO("multi_offb_node start");
    std::vector<simple_controller> sc(3);

    std::vector<std::vector<int>> pos = {
            {0, 0, 2},
            {1, 1, 3},
            {2, 2, 4}
    };
    for (int id = 0; id < 3; id++) {
        sc[id].id = id;
        sc[id].state_sub = nh.subscribe<mavros_msgs::State>("uav" + std::to_string(id) + "/mavros/state", 10,
                                                            &simple_controller::state_cb, &sc[id]);
        sc[id].local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
                "uav" + std::to_string(id) + "/mavros/setpoint_position/local", 10);
        sc[id].arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
                "uav" + std::to_string(id) + "/mavros/cmd/arming");
        sc[id].set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
                "uav" + std::to_string(id) + "/mavros/set_mode");

        sc[id].pose.pose.position.x = pos[id][0];
        sc[id].pose.pose.position.y = pos[id][1];
        sc[id].pose.pose.position.z = pos[id][2];
    }
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok()) {
        ROS_INFO("wait for FCU connection");
        bool ok = true;
        for (int id = 0; id < 3; id++) {
            ok &= sc[id].current_state.connected;
        }
        if (ok) break;

        ros::spinOnce();
        rate.sleep();
    }
    //send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i) {
        ROS_INFO("send a few setpoints : %d", i);
        for (int id = 0; id < 3; id++) {
            sc[id].pub();
        }
        ros::spinOnce();
        rate.sleep();
    }

    for (int id = 0; id < 3; id++) {
        sc[id].offb_set_mode.request.custom_mode = "OFFBOARD";
        sc[id].arm_cmd.request.value = true;
        sc[id].set_mode_client.call(sc[id].offb_set_mode);
        ROS_INFO("%d : Offboard enabled", id);
    }

    std::vector<ros::Time> last_requests(3, ros::Time::now());
    while (ros::ok()) {
        for (int id = 0; id < 3; id++) {
            if (sc[id].current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_requests[id] > ros::Duration(5.0))) {
                if (sc[id].set_mode_client.call(sc[id].offb_set_mode) && sc[id].offb_set_mode.response.mode_sent) {
                    ROS_INFO("%d : Offboard enabled", id);
                }
                last_requests[id] = ros::Time::now();
            } else {
                if (!sc[id].current_state.armed &&
                    (ros::Time::now() - last_requests[id] > ros::Duration(5.0))) {
                    if (sc[id].arming_client.call(sc[id].arm_cmd) &&
                        sc[id].arm_cmd.response.success) {
                        ROS_INFO("%d : Vehicle armed", id);
                    }
                    last_requests[id] = ros::Time::now();
                }
            }
            sc[id].pub();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}