#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>
#include <termios.h>
#include <std_msgs/Header.h>
#include "trajectory.h"
#include "orientationhandler.h"
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <daryl_msgs/SetDriveVelocity.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


/// ======================================================================================================
/// The Steer_robot class implents a trajectory traking controller based on input output linearization of
/// the unicycle kinematic model, for theoretical details see in section 3.3 of "Control of Wheeled Mobile
/// Robots: An Experimental Overview" by Alessandro De Luca, Giuseppe Oriolo, Marilena Vendittelli
/// ======================================================================================================
class Steer_robot
{



public:
    Steer_robot(const ros::NodeHandle& node);
    void steerLoop();
    void callbackReadPath(const nav_msgs::Path::ConstPtr& path_msg);
    void callbackSetRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    double set_angle_to_range(double alpha, double min);
    double compute_sigma(double vsig, double wsig);
    
    int initialize();



    /// the gains of the feedback controller
    double k_one_;
    double k_two_;
    /// parameter to how far I need to track
    double b_;
    /// Time stamp
    double ts_;

private:


    int  driving_;





    ros::NodeHandle nhs_;

    ros::Subscriber path_listener_;
    ros::Subscriber odom_listener_;

    ros::Publisher vel_publisher_;
    ros::Publisher end_track_publisher;

    
    geometry_msgs::PoseStamped robot_pose_;

    std::vector<geometry_msgs::Quaternion> orientations_;
    std::vector<geometry_msgs::Point> current_path_;

    /// Vectors to store feedforward commands
    std::vector<double> thetad;
    std::vector<double> dx_d;
    std::vector<double> dy_d;
    std::vector<double> ddx_d;
    std::vector<double> ddy_d;
    std::vector<double> vd;
    std::vector<double> wd;
    



    
};


