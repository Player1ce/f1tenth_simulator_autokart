#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"


class KeepLeftController {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Subscribers
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;


    // rough estimation of state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double ttc_threshold;
    bool in_collision=false;

    double collision_angle = 0;

//    // for collision logging
//    std::ofstream collision_file;
//    double beginning_seconds;
//    int collision_count=0;




public:
    KeepLeftController() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("keep_left_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &KeepLeftController::odom_callback, this);

        // start a sub for the scan
        scan_sub = n.subscribe(scan_topic, 1, &KeepLeftController::laser_callback, this);

        // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};

        // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = racecar_simulator::Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = racecar_simulator::Precompute::get_car_distances(scan_beams, wheelbase, width,
                                                      scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

    }

    void laser_callback(const sensor_msgs::LaserScan & msg) {
        collision_checker(msg);
    }


    void collision_checker(const sensor_msgs::LaserScan & msg) {
        // This function calculates TTC to see if there's a collision

        int min_ttc = -1;
        int min_range = -1;
        for (size_t i = 0; i < msg.ranges.size(); i++) {
            double angle = msg.angle_min + i * msg.angle_increment;
            double range = msg.ranges[i];

            // calculate projected velocity
            double proj_velocity = state.velocity * cosines[i];
            double ttc = (msg.ranges[i] - car_distances[i]) / proj_velocity;

//            if (min_ttc == -1) {
//                min_ttc = ttc;
//                collision_angle = angle;
//            }
//            else if (ttc < min_ttc) {
//                min_ttc = ttc;
//                collision_angle = angle;
//            }

            if (min_range == -1) {
                min_range = range;
                collision_angle = angle;
            }
            else if (range < min_range) {
                min_range = range;
                collision_angle = angle;
            }

            // if it's small, there's a collision
            if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                ROS_INFO("Collision detected in keep_left");
//                    // Send a blank mux and write to file
//                    collision_helper();
//
//                    in_collision = true;
//
//                    collision_count++;
//                    collision_file << "Collision #" << collision_count << " detected:\n";
//                    collision_file << "TTC: " << ttc << " seconds\n";
//                    collision_file << "Angle to obstacle: " << angle << " radians\n";
//                    collision_file << "Time since start of sim: " << (ros::Time::now().toSec() - beginning_seconds) << " seconds\n";
//                    collision_file << "\n";
                return;
            }
        }
        if (collision_angle == 0) {
            ROS_INFO("perfectly straight");
        }
        else if (collision_angle < 0) {
            ROS_INFO("closest collision is right");
        }
        else {
            ROS_INFO("closest collision is left");
        }

        // if it's gone through all beams without detecting a collision, reset in_collision
        in_collision = false;
    }


    void odom_callback(const nav_msgs::Odometry & msg) {
        // Keep track of state to be used elsewhere
        state.velocity = msg.twist.twist.linear.x;
        state.angular_velocity = msg.twist.twist.angular.z;
        state.x = msg.pose.pose.position.x;
        state.y = msg.pose.pose.position.y;
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;


        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double) rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        double random_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = random_angle;

        int sign = collision_angle;
        sign = sign / std::abs(sign);

        drive_msg.steering_angle = range / 3.0;
        drive_msg.steering_angle *= sign * -1;

        ROS_INFO_STREAM("collision angle: " << collision_angle << " steering angle" << drive_msg.steering_angle << "sign: " << sign);

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);


    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "keep_left_controller");
    KeepLeftController rw;
    ros::spin();
    return 0;
}