#ifndef SRL_SMP_H
#define SRL_SMP_H


#endif // SRL_SMP_H




#include <cstdlib>
#include <fstream>
#include <iostream>

#include "trajectory.h"
#include "orientationhandler.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>




/// ==================================================================================
/// srl_smp
/// This node solves a motion planning problem given the current robot_state and the 
/// goal. It is based on the Sampling-based Motion Planning Library from MIT, where you 
/// can use three RRT variants.
/// ==================================================================================


//! SMP HEADER

#include <smp/components/samplers/uniform.hpp>
//#include <smp/components/samplers/uniformc.hpp>
// #include <smp/components/samplers/gauss.hpp>




/// For a gaussian use regionc and gauss
//#include <smp/components/samplers/gauss.hpp>





//#include <smp/components/samplers/opra.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
//#include <smp/components/extenders/dubins.hpp>
//#include <smp/components/extenders/single_integrator.hpp>
#include <smp/components/extenders/pos.hpp>
//#include <smp/components/extenders/splines.hpp>
// #include <smp/components/extenders/discrMP.hpp>



//#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/collision_checkers/circle.hpp>

#include <smp/components/multipurpose/minimum_time_reachability.hpp>
// #include <smp/components/multipurpose/smoothness_cost.hpp>


#include <smp/planners/rrtstar.hpp>
#include <smp/planner_utils/trajectory.hpp>



// State, input, vertex_data, and edge_data definitions
//typedef state_dubins state_t;
//typedef input_dubins input_t;

//typedef state_single_integrator<3> state_t;
//typedef input_single_integrator input_t;

typedef state_pos state_t;
typedef input_pos input_t;

//typedef state_splines state_t;
//typedef input_splines input_t;

// typedef state_discrmp state_t;
// typedef input_discrmp input_t;

typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;

// typedef smoothness_cost_reachability_vertex_data vertex_data_t;
// typedef smoothness_cost_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
    typedef state_t state;
    typedef input_t input;
    typedef vertex_data_t vertex_data;
    typedef edge_data_t edge_data;
} typeparams;

//// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;





// Define all planner component types
//typedef sampler_opra<typeparams,3> sampler_t;
typedef sampler_uniform<typeparams,3> sampler_t;
//typedef sampler_uniformc<typeparams,3> sampler_t;
//typedef sampler_gauss<typeparams,3> sampler_t;



//typedef sampler_uniformc<typeparams,3> sampler_t;
//typedef sampler_gauss<typeparams,3> sampler_t;


typedef distance_evaluator_kdtree<typeparams,3> distance_evaluator_t;
//typedef extender_dubins<typeparams> extender_t;
//typedef extender_single_integrator<typeparams,3> extender_t;
typedef extender_pos<typeparams,3> extender_t;
//typedef extender_splines<typeparams,3> extender_t;
// typedef extender_discrmp<typeparams,3> extender_t;


//typedef collision_checker_standard<typeparams,2> collision_checker_t;
typedef collision_checker_circle<typeparams,2> collision_checker_t;

typedef minimum_time_reachability<typeparams,2> min_time_reachability_t;
// typedef smoothness_cost<typeparams,2>  smoothness_cost_t;

// Define all algorithm types
typedef rrtstar<typeparams>  rrtstar_t;

// Types to save the tree information
typedef vertex<typeparams> vertex_t;
typedef edge<typeparams> edge_t;


#define HUMANMAX 20






typedef struct ObstaclePoint {
    
    double x;
    double y;
    double z;
    double cell_width;
    double cell_height;

    ObstaclePoint() { x = 0.0; y = 0.0; z = 0.0; cell_width=0; cell_height=0;}
    ObstaclePoint(double xx, double yy) : x(xx), y(yy) { z = 0.0; cell_width=0; cell_height=0;}
    ObstaclePoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz)  { cell_width=0; cell_height=0; }
    ObstaclePoint(double xx, double yy, double zz, double cw, double ch) : x(xx), y(yy), z(zz), cell_width(cw), cell_height(ch)  { }
    ObstaclePoint(double* p) { x = p[0]; y = p[1]; z = p[2]; cell_width=p[3]; cell_height=p[4];}

    ObstaclePoint& operator=(ObstaclePoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        cell_width=copy.cell_width;
        cell_height=copy.cell_height;

        return *this;
    }

} Tobstacle;







typedef struct HumanPoint {
    
    // pose of a human being (agent)
    double x;
    double y;
    double z;
    // agent id and type
    int id;
    int type;

    // cell_width and cell_height used as agent's sizes
    double cell_width;
    double cell_height;

    HumanPoint() { x = 0.0; y = 0.0; z = 0.0; cell_width=0.3; cell_height=0.4; id=0; type=0;}
    HumanPoint(double xx, double yy) : x(xx), y(yy) { z = 0.0; cell_width=0.3; cell_height=0.4; id=0; type=0;}
    HumanPoint(double xx, double yy, double zz, double ii, double tt) : x(xx), y(yy), z(zz), id(ii), type(tt) { cell_width=0.3; cell_height=0.4; }
    HumanPoint(double* p) { x = p[0]; y = p[1]; z = p[2]; id=p[3]; type=p[4]; cell_width=0.3; cell_height=0.4;}

    HumanPoint& operator=(HumanPoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        cell_width=copy.cell_width;
        cell_height=copy.cell_height;
        id=copy.id;
        type=copy.type;

        return *this;
    }

} Thuman;










class SRL_smp
{





private:
    

    ros::NodeHandle nh_;
    // Publishers
    ros::Publisher pub_path_;    // WARNING: TO PUBLISH A PATH
    ros::Publisher pub_goal_;
    ros::Publisher pub_tree_;
    ros::Publisher pub_tree_dedicated_;
    ros::Publisher pub_path_dedicated_;
    // ros::Publisher pub_sensor_range_;
    // subscribers
    ros::Subscriber sub_obstacles_;    // to read the obstacles' positions
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_steer_state_;






public:

   

    RRT_globalplanner(const ros::NodeHandle& node);

    /// subscriber setGoal
    void callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /// subscriber to the map message, where obstacles are read
    void callbackObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /// subscriber to the current robot pose
    void callbackSetRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /// to read the Steer Robot Node State
    void callbackReadSteerState(const std_msgs::Bool::ConstPtr& msg);

    /// Publish the  Path
    void publishPath(Trajectory *t);
    /// Publish the Goal
    void publishGoal();
    /// Initialize the node
    bool initialize();




    /// Loop to plan 
    int plan(Trajectory *traj,int type);

    /// Set Goal Region
    void setGoal(double x, double y, double theta,double toll);

    /// method to save for each iteration iter the final Trajectory and the tree Structure.
    void saveData(Trajectory *traj, int iter);
    /// method to save for each time slot the obtained cost
    void saveCost(double c, double t);
    /// Reading the human poses from a text file
    int readHumanPoses(int hi);

    // /// method to save the tree when no solution is found
    int saveTree(list <vertex_t *> *list_vertices);

    void publishTree();
    std::vector<size_t> getnExpansions() {return nexpasions_;}


    /// Cellwidth of the scene
    double cellwidth_;  
    /// cell height of the scene
    double cellheight_; 

  

    /// To define the goal region 
    double goal_x_;
    double goal_y_;
    double goal_theta_;
    double toll_goal_;

    /// to save the cost 
    ofstream  cost_;




    /// Number of max iterations
    int Nit_;

    /// Trajectory-Tree information
    Trajectory *traj_;

    /// Size of the scene 
    double xscene_;
    double yscene_;



  /// number of the humans in the scene (to use when the social cost need to be added)
    int n_hum_;
    double hposes_[HUMANMAX][3];


     /// tree and expansions occured
    std::vector<Tpoint> tree_;
    std::vector<size_t> nexpasions_;



    float numVertices_;
    float timeIter_;
    float timeSolution_;
    double nrew_;


    double xsupp_;
    double ysupp_;

    bool coll_;

    /// Obstacles 
    std::vector<Tobstacle> obstacle_positions;
    /// agents --> To use it one we have a People Tracker Running
    std::vector<Thuman> agents_position;
    /// Robot Pose
    geometry_msgs::PoseStamped robot_pose_;

    std::vector<geometry_msgs::Quaternion> orientations_;
    std::vector<geometry_msgs::Point> positions_;

    /// Parameters
    /// Flag to terminate the iterations after a certain amount of seconds
    int TIMECOUNTER ;
    /// Flag to stop the planner once you found the first solution
    int FIRSTSOLUTION ;
    /// If you want to show all the debug info
    int DEB_RRT;
    /// One if you want to select the neighbours in a Box which respects the diff-drive kinematics
    int BOX;

    /// Radius where to select the nearest vertex in plane RRT
    double RADIUS;

    /// End condition to the position controller, in the paper named as gamma.
    double RHO_END;


    //! Need to choose the right distance metric
    /*!
     *  if DISTANCE_METRIC==0, use POSQ extender
     *  if DISTANCE_METRIC==1, use basis function model
     *  if DISTANCE_METRIC==2, use neural network model
    */
    int DISTANCE_METRIC;

    //!  DT
    /*!
        choose the integration time step during execution of RRT/RRT*
    */
    double DT;
    //!  final end of the pose controller
    /*!
        choose the integration time step during computation of the DISTANCE METRIC USING POSQ
    */
    double RES_POSQ;



    int first_problem_solved;

    bool goal_init_;
    bool reset_traj;


};

