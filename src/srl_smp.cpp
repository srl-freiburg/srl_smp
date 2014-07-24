#include "srl_smp.h"


using namespace smp;

using namespace std;

//extern Config config;



SRL_smp::SRL_smp(const ros::NodeHandle& node) : nh_(node)
{

    // number of RRT* iterations

    // ** RRT* Section **
    // 1 Create the planner algorithm -- Note that the min_time_reachability variable acts both
    //                                       as a model checker and a cost evaluator.
    // cout<<"Debug: Initiliaze planner"<<endl;
    /// Need to be set 
    this->goal_theta_=0;
    this->goal_x_=0;
    this->goal_y_=0;
    this->cellheight_=1.5;
    this->cellwidth_=1.5;
    this->xsupp_=0;
    this->ysupp_=0;
    coll_=false;
    this->timeIter_=0;
    this->nrew_=0;
    this->goal_init_=false;
    this->reset_traj=false;
    this->RHO_END=0.15;
    first_problem_solved=0;

    /// to be used once we have a running People Tracker
    agents_position.clear();

    /// File stream to save the cost
    string dirTree="cost";
    string extension=".data";
    string filePath;
    std::stringstream sstm;
    sstm << dirTree<<extension;
    filePath = sstm.str();
    cost_.open(filePath.c_str());
    if (cost_.fail())
    {     ROS_DEBUG("Failed to open cost file!!!\n");
        exit(0)  ;
    }


}



/// ==================================================================================
/// setGoal(double x, double y, double theta,double toll)
/// Method to store the Goal region description into the instance of the class
/// ==================================================================================

void SRL_smp::setGoal(double x, double y, double theta,double toll){

    this->goal_theta_=theta;
    this->goal_x_=x;
    this->goal_y_=y;
    this->toll_goal_=toll;
    this->goal_init_=true;
    


}
/// ==================================================================================
/// publishTree()()
/// Method to publish the current Tree
/// ==================================================================================

void SRL_smp::publishTree(){

    visualization_msgs::Marker tree_marker_;

 
    tree_marker_.header.frame_id = "map";
    tree_marker_.header.stamp = ros::Time();
    tree_marker_.ns = "SRL_smp";
    tree_marker_.id = 1;

    tree_marker_.type = visualization_msgs::Marker::POINTS;
    tree_marker_.color.a = 0.20;
    tree_marker_.color.r = 1.0;
    tree_marker_.color.g = 0.0;
    tree_marker_.color.b = 0.0;

    tree_marker_.scale.x = 0.05;
    tree_marker_.scale.y = 0.05;
    tree_marker_.scale.z = 0.05;

        
    tree_marker_.action = 0;  // add or modify


    for (vector<Tpoint>::const_iterator iter =this->tree_.begin(); iter !=this->tree_.end(); ++iter) {
        Tpoint a = (*iter);
        geometry_msgs::Point p;
        p.x = a.x;
        p.y = a.y;
        p.z = 0.0;

        tree_marker_.points.push_back(p);

    }




    pub_tree_.publish(tree_marker_);


    visualization_msgs::Marker dedicated_tree_marker_;

 

    /// pub_tree_dedicated_ publishes a tree on a topic showed by a separate rviz session

    dedicated_tree_marker_.header.frame_id = "map";
    dedicated_tree_marker_.header.stamp = ros::Time();
    dedicated_tree_marker_.ns = "SRL_smp";
    dedicated_tree_marker_.id = 1;

    dedicated_tree_marker_.type = visualization_msgs::Marker::POINTS;
    dedicated_tree_marker_.color.a = 1;
    dedicated_tree_marker_.color.r = 1.0;
    dedicated_tree_marker_.color.g = 0.0;
    dedicated_tree_marker_.color.b = 0.0;

    dedicated_tree_marker_.scale.x = 0.05;
    dedicated_tree_marker_.scale.y = 0.05;
    dedicated_tree_marker_.scale.z = 0.01;

        
    dedicated_tree_marker_.action = 0;  // add or modify


    for (vector<Tpoint>::const_iterator iter =this->tree_.begin(); iter !=this->tree_.end(); ++iter) {
        Tpoint a = (*iter);
        geometry_msgs::Point p;
        p.x = a.x;
        p.y = a.y;
        p.z = 0.05;

        dedicated_tree_marker_.points.push_back(p);

    }

    pub_tree_dedicated_.publish(dedicated_tree_marker_);


}

/// ==================================================================================
/// publishGoal()
/// Method to publish the Goal Marker
/// ==================================================================================

void SRL_smp::publishGoal(){

    visualization_msgs::Marker goal_marker_;

    goal_marker_.header.frame_id = "map";
    goal_marker_.header.stamp = ros::Time();
    goal_marker_.ns = "SRL_smp";
    goal_marker_.id = 1;

    goal_marker_.type = visualization_msgs::Marker::CUBE;
    goal_marker_.color.a = 0.20;
    goal_marker_.color.r = 1.0;
    goal_marker_.color.g = 0.10;
    goal_marker_.color.b = 0.10;

    goal_marker_.scale.x = this->toll_goal_;
    goal_marker_.scale.y = this->toll_goal_;
    goal_marker_.scale.z = 0.01;

        
    goal_marker_.action = 0;  // add or modify

    goal_marker_.pose.position.x =  this->goal_x_;
    goal_marker_.pose.position.y =  this->goal_y_;
    goal_marker_.pose.position.z = 0; /// needed and

    goal_marker_.pose.orientation.x = 0;
    goal_marker_.pose.orientation.y = 0;
    goal_marker_.pose.orientation.z = 0;
    goal_marker_.pose.orientation.w = 0;


    pub_goal_.publish(goal_marker_);


}


/// ==================================================================================
/// callbackSetGoal(const geometry_msgs::PointStamped& msg)
/// Method to set the new Goal
/// ==================================================================================

void SRL_smp::callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){


        ROS_INFO("Goal pose updated");

        double xgoal,ygoal,thetagoal;

        double toll=3;
        nh_.getParam("goal_radius", toll);

        xgoal=msg->pose.position.x;
        ygoal=msg->pose.position.y;

        tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        thetagoal=yaw;

        if(first_problem_solved==0)
        ROS_INFO("%f %f %f new goal pose.",xgoal,ygoal,thetagoal);

        setGoal(xgoal, ygoal, thetagoal, toll);

}

/// ==================================================================================
/// initialize()
/// Method to initialize all the publishers and subscribers
/// ==================================================================================
bool SRL_smp::initialize(){


    ///  all the ROS DATA
    // setup publishers

    pub_path_ = nh_.advertise<nav_msgs::Path>("global_path", 1000);
    pub_goal_ = nh_.advertise<visualization_msgs::Marker>("SRL_smp_globalplanner_goal",1000);
    pub_tree_=nh_.advertise<visualization_msgs::Marker>("SRL_smp_planner_tree",1000);
    pub_tree_dedicated_=nh_.advertise<visualization_msgs::Marker>("SRL_smp_planner_tree_dedicated",1000);
    pub_path_dedicated_=nh_.advertise<visualization_msgs::Marker>("path_dedicated",1000);

    // subscribers to the Hector Mapping defined messages
    sub_obstacles_=nh_.subscribe("map",1, &SRL_smp::callbackObstacles,this);    // to read OccupancyGrid and the obstacles' positions
    sub_robot_pose_=nh_.subscribe("slam_out_pose",1,&SRL_smp::callbackSetRobotPose,this); // to read the current robot pose 

    sub_goal_=nh_.subscribe("global_goal",1,&SRL_smp::callbackSetGoal,this);
    
    sub_steer_state_=nh_.subscribe("/steer_robot/end_track",1,&SRL_smp::callbackReadSteerState,this);


}


/// ==================================================================================
/// callbackReadSteerState(const std_msgs::Bool::ConstPtr& msg)
/// callback to read the steer robot state, if the data is true, the goal position has been achieved 
/// ==================================================================================
void SRL_smp::callbackReadSteerState(const std_msgs::Bool::ConstPtr& msg){

    if(msg->data==true &&   this->reset_traj==false){
       this->goal_init_=false;
       this->reset_traj=true;
    }
    else
        this->reset_traj=false;


}
/// ==================================================================================
/// callbackSetRobotPose(Const geometry_msgs::PoseStamped::ConstPtr& msg)
/// callback to read the robot pose
/// ==================================================================================

void SRL_smp::callbackSetRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg){

    if(first_problem_solved<0)
       ROS_INFO("Robot pose updated");
    robot_pose_.header=msg->header;
    robot_pose_.pose=msg->pose;
    



    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw,or_curr;
    m.getRPY(roll, pitch, yaw);
    or_curr=yaw;



    if(first_problem_solved<0)
        ROS_INFO("Robot pose %f %f %f ",msg->pose.position.x,msg->pose.position.y,or_curr);


}

/// ==================================================================================
/// callbackObstacles(const nav_msgs::GridCells::ConstPtr& msg)
/// callback to read the obstacles positions
/// ==================================================================================
void SRL_smp::callbackObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    /// Read the obstacles in the messagge and save them in the vector obstacle_positions
    obstacle_positions.clear();
    ROS_INFO("Reading Obstacles positions..");

    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    ROS_INFO("Got map %d %d", info.width, info.height);
    
    int occup_prob,n_occupied;
    n_occupied=0;
    double xo, yo;
    double min_prob=0.1;
    for (unsigned int x = 0; x < info.width; x++)
        for (unsigned int y = 0; y < info.height; y++){
            /// acces the data in the map, in row-major order 
                       occup_prob=msg->data[x+ info.width * y];
                       if(occup_prob>min_prob){
                            xo= (x+0.5)*info.resolution+info.origin.position.x;
                            yo= (y+0.5)*info.resolution+info.origin.position.y;
                            obstacle_positions.push_back(Tobstacle(xo,yo,0,info.resolution,info.resolution));
                            n_occupied++;

                            if(first_problem_solved<0)
                                ROS_INFO("Obstacle added at position (x,y,theta,width, height): (%f,%f,%f,%f,%f)",xo,yo,0.000001,info.resolution,info.resolution);
                       }

                            

        }

 if(first_problem_solved<0)
    ROS_INFO("Size of obstacles vector: %d",(int)obstacle_positions.size());

}
   


/// ==================================================================================
/// publishPath(Trajectory *t)
/// method to publish the trajectory t 
/// ==================================================================================

void SRL_smp::publishPath(Trajectory *t){

/// To IMPLEMENT
    nav_msgs::Path path_;
    ROS_INFO("Publishing a path");

    std::vector<Tpoint> path = t->getPath();
    ROS_INFO("Path Size :%d",(int)path.size());
    
    path_.header.frame_id = "map";
    path_.header.stamp = ros::Time();
    path_.poses.resize((int)path.size());



    visualization_msgs::Marker path_marker_;
    path_marker_.header.frame_id = "map";
    path_marker_.header.stamp = ros::Time();
    path_marker_.ns = "SRL_smp";
    path_marker_.id = 1;

    path_marker_.type = visualization_msgs::Marker::POINTS;
    path_marker_.color.a = 1;
    path_marker_.color.r = 0.0;
    path_marker_.color.g = 1.0;
    path_marker_.color.b = 0.0;

    path_marker_.scale.x = 0.25;
    path_marker_.scale.y = 0.25;
    path_marker_.scale.z = 0.25;

        
    path_marker_.action = 0;  // add or modify


    for (size_t i = 0; i < path.size(); i++) {


        geometry_msgs::PoseStamped posei;
        
        path_.poses[i].header.stamp = ros::Time();
        path_.poses[i].header.frame_id="map";

    
        path_.poses[i].pose.position.x = path[i].x;
        path_.poses[i].pose.position.y = path[i].y;
        path_.poses[i].pose.position.z = 0;
        ROS_INFO("Path point %f, %f, %f",path[i].x,path[i].y,path[i].z);
        OrientationHandler orientation(path[i].z);
        
        
        path_.poses[i].pose.orientation.x=orientation.getQx();
        path_.poses[i].pose.orientation.y=orientation.getQy();
        path_.poses[i].pose.orientation.z=orientation.getQz();
        path_.poses[i].pose.orientation.w=orientation.getQw();
        // ROS_INFO("Quaternion: (%f,%f,%f,%f)",orientation.getQx(),orientation.getQy(),orientation.getQz(),orientation.getQw());


        /// Path_marker
        geometry_msgs::Point p;
        p.x = path[i].x;
        p.y = path[i].y;
        p.z = 0.05;
        path_marker_.points.push_back(p);


    }
    
    if(path.size()>0){
    
        pub_path_.publish(path_);
        pub_path_dedicated_.publish(path_marker_);

        ROS_INFO("Path Published");

    }
}




/// ==================================================================================
/// readHumanPoses(int hi)
/// method to readHumanPoses written in a txt file 
/// ==================================================================================

int SRL_smp::readHumanPoses(int hi){

    /// Number of human beings' poses

    n_hum_=hi;



    /// Reading human poses from the file
    std::ifstream in;
    in.open("humanposes.txt");
    ROS_DEBUG("Number of Human Beings: %d \n",n_hum_);
    if (!in) {
        ROS_DEBUG_STREAM ("Cannot open Human poses file.");
        return 1;
    }
    int i=0;
    ROS_DEBUG_STREAM("DEBUG: Reading Human being poses");
    while( i<n_hum_) {

        for(int j=0;j<3;j++){
            in>>hposes_[i][j];
            ROS_DEBUG_STREAM("DEBUG: "<<hposes_[i][j]<<" ");

        }
    
    i++;
    }



    in.close();


    return 0;

}

/// ==================================================================================
/// saveData(Trajectory *traj, int iter)
/// method to save for each iteration iter the final Trajectory and the tree Structure.
/// Each file will end up with the iter index
/// ==================================================================================
void SRL_smp::saveData(Trajectory *traj, int iter){

    /// SAVING THE TREE FOR EACH ITERATION

    /// open all the files

    ofstream  rrtTreeIt;
    string dirTree="trees/rrtTree";
    string extension=".data";
    string filePath;
    std::stringstream sstm;
    sstm << dirTree << iter;
    sstm<<extension;
    filePath = sstm.str();
    rrtTreeIt.open(filePath.c_str());
    if (rrtTreeIt.fail())
    {     ROS_DEBUG_STREAM("Failed to open rrtTreeIt!!!");
        exit(0)  ;
    }


    ofstream  rcellxIt;
    string dirrcellx="trees/rcellx";
    string filePathrcellx;
    std::stringstream sstmrcellx;
    sstmrcellx << dirrcellx << iter;
    sstmrcellx<<extension;
    filePathrcellx = sstmrcellx.str();
    rcellxIt.open(filePathrcellx.c_str());

    if (rcellxIt.fail())
    {     ROS_DEBUG_STREAM("Failed to open rcellxIt!!!");
        exit(0)  ;
    }



    ofstream  rcellyIt;
    string dirrcelly="trees/rcelly";
    string filePathrcelly;
    std::stringstream sstmrcelly;
    sstmrcelly << dirrcelly << iter;
    sstmrcelly<<extension;
    filePathrcelly = sstmrcelly.str();
    rcellyIt.open(filePathrcelly.c_str());

    if (rcellyIt.fail())
    {     ROS_DEBUG_STREAM("Failed to open rcellyIt!!!");
        exit(0)  ;
    }




    ofstream  indIt;
    string dirindIt="trees/ind";
    string filePathindIt;
    string extensionInd= ".txt";
    std::stringstream sstmindIt;
    sstmindIt << dirindIt << iter;
    sstmindIt<<extensionInd;
    filePathindIt = sstmindIt.str();
    indIt.open(filePathindIt.c_str());



    if (indIt.fail())
    {     ROS_DEBUG_STREAM("Failed to open indIt!!!");
        exit(0)  ;
    }


    /// Saving all the points in the tree

    for (vector<Tpoint>::const_iterator iter =this->tree_.begin(); iter !=this->tree_.end(); ++iter) {
        Tpoint a = (*iter);
        rrtTreeIt<<a.x<<" "<<a.y<<" "<<a.z<<endl;
        rrtTreeIt<<std::flush;

    }


    /// save the  returned path

    std::vector<Tpoint> path = traj->getPath();
    for (size_t i = 0; i < path.size(); i++) {
        rcellxIt<<path[i].x<<endl;
        rcellyIt<<path[i].y<<endl;
        rcellxIt<<std::flush;
        rcellyIt<<std::flush;
    }



    /// save the number of element added for each expansion
    std::vector<size_t> nexp=this->getnExpansions();
    for (size_t j = 0; j < nexp.size(); j++) {
        indIt<<nexp[j]<<endl;
        indIt<<std::flush;
    }




    /// close all the files
    indIt.close();
    rrtTreeIt.close();
    rcellxIt.close();
    rcellyIt.close();




}

/// ==================================================================================
/// saveCost(double c,double t)
/// method to save for each time t the computed cost
/// ==================================================================================
void SRL_smp::saveCost(double c,double t){


    cost_<<t<<" "<<c<<endl;


}

/// ==================================================================================
/// saveTree(list <vertex_t *> *list_vertices)
/// method to save for the tree 
/// =================================================================================
int SRL_smp::saveTree(list <vertex_t *> *list_vertices){
/// WARNING THE TRAJECTORY OBJECT NEED TO BE DEFINED as TREE

  //  construct the graph and save all the trajectories samples and the vertice
        

        int i = 0;
        int num_edges = 0;
        for (typename list<vertex_t *>::iterator iter = list_vertices->begin();
             iter != list_vertices->end(); iter++) {

            vertex_t *vertex_curr = *iter;
            state_t &state_ref = *(vertex_curr->state);

            i++;
            num_edges += (*iter)->incoming_edges.size();//
        }

        i = 0;

        // file stream to save the number of cartesian points for each edge
        ofstream ind;
        ind.open("ind.txt",std::fstream::in | std::fstream::out | std::fstream::app);

        for (typename list<vertex_t *>::iterator it_vertex = list_vertices->begin();
             it_vertex != list_vertices->end(); it_vertex++) {

            vertex_t *vertex_curr = *it_vertex;

            list< edge_t* > *incoming_edges_curr = &(vertex_curr->incoming_edges);
            for (typename list<edge_t*>::iterator it_edge = incoming_edges_curr->begin();
                 it_edge != incoming_edges_curr->end(); it_edge++) {

                edge_t *edge_curr = *it_edge;

                list<state_t*> *list_states_curr = &(edge_curr->trajectory_edge->list_states);

                if (list_states_curr->size () == 0) {
                    {
                        ind<<list_states_curr->size ()<<endl;
                        this->nexpasions_.push_back(list_states_curr->size ());
                    }


                }
                else {


                    {
                        ind<<list_states_curr->size()<<endl;
                        this->nexpasions_.push_back(list_states_curr->size ());
                    }

                    int j = 0;
                    for (typename list<state_t*>::iterator it_state = list_states_curr->begin();
                         it_state != list_states_curr->end(); it_state++) {

                        state_t *state_traj_curr = *it_state;
                        state_t &state_traj_ref = *state_traj_curr;
                        this->tree_.push_back(Tpoint(state_traj_ref[0],state_traj_ref[1],state_traj_ref[2]));

                        j++;

                    }
                }

                i++;
            }
        }


        ind.close();



}







/// ==================================================================================
/// plan(Trajectory *traj,int type)
/// method to solve a planning probleme.
/// ==================================================================================

int SRL_smp::plan(Trajectory *traj,int type){




    // RRT* section
    // for each movement clear the obstacle list and refill it with the static ones and the agents
    // 1. CREATE PLANNING OBJECTS
    //cout<<"Debug: Initiliaze components";
    // 1.a Create the components
    sampler_t sampler;
    // State, input, vertex_data, and edge_data definitions
    distance_evaluator_t distance_evaluator;

    extender_t extender;
    collision_checker_t collision_checker;

    min_time_reachability_t min_time_reachability;



    rrtstar_t planner (sampler,distance_evaluator,extender,collision_checker,
                          min_time_reachability,min_time_reachability);




    /// setting params of the motion plannner
    planner.RHO_END =  this->RHO_END;
    planner.DISTANCE_METRIC = this->DISTANCE_METRIC;
    planner.DT= this->DT;
    planner.RES_POSQ= this->RES_POSQ;

    /// Initialize the robot pose
    double rx,ry,rz;


    rx = robot_pose_.pose.position.x;
    ry = robot_pose_.pose.position.y;


    tf::Quaternion q(robot_pose_.pose.orientation.x,robot_pose_.pose.orientation.y,robot_pose_.pose.orientation.z,robot_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw);
    rz=robot_yaw;

 


    //added for single_integrator_extender
    //    extender.set_max_length(2);
    double side;
    int multiplier_side=sqrt(5);
    //considering a square inscribed in a circle, let's calculate a radius and then multiplie it for 2 (conservative way)
    side=(sqrt((goal_x_-rx)*(goal_x_-rx)+(goal_y_-ry)*(goal_y_-ry)))*multiplier_side;


    planner.parameters.set_phase (type);   // The phase parameter can be used to run the algorithm as an RRT,
    // See the documentation of the RRG algorithm for more information.
    if(type==0){

            planner.parameters.set_fixed_radius(RADIUS);

    }
    planner.BOX=0;

    planner.parameters.set_gamma (side);    // Set this parameter should be set at least to the side length of
    //   the (bounded) state space. E.g., if the state space is a box
    //   with side length L, then this parameter should be set to at
    //   least L for rapid and efficient convergence in trajectory space.
    planner.parameters.set_dimension (3);
    planner.parameters.set_max_radius (2*side);  // This parameter should be set to a high enough value. In practice,
    //   one can use smaller values of this parameter to get a good
    //   solution quickly, while preserving the asymptotic optimality.




    //! 2 Set the Sampler Support
    float x_sup,y_sup,th_sup,th_size;
    th_size=0.2;
    x_sup=rx;
    y_sup=ry;
    th_sup=goal_theta_;
    x_sup=(rx+goal_x_);
    y_sup=(ry+goal_y_);

#if DEB_RRT>0
   ROS_INFO_STREAM("GoalTheta:"<<goal_theta_<<" CurrentTheta:"<<th_sup);
   ROS_INFO_STREAM("xsup: "<<x_sup<<"ysup: "<<y_sup);
#endif


    /// For a Rectangle as Sampling Support
    region<3> sampler_support;
    double xs,ys;

    /// Size of the Scene!!!double xscene, double yscene
    xs=x_sup;
    ys=y_sup;

    sampler_support.center[0] =xs/2;
    sampler_support.center[1] =ys/2;
    sampler_support.center[2] =goal_theta_;

    sampler_support.size[0] = 2*xs;
    sampler_support.size[1] = 2*ys;
    sampler_support.size[2] = M_PI*2;









    sampler.set_support(sampler_support);
if(DEB_RRT>0){
    ROS_INFO_STREAM("Sampler support center...."<< sampler_support.center[0]<< " "<<sampler_support.center[1]<< " "<< sampler_support.center[2]<< " ");
    ROS_INFO_STREAM("Sampler Sides :"<< sampler_support.size[0]<<" " <<sampler_support.size[1]<<" " <<sampler_support.size[2]);

}





if(DEB_RRT>0)
    ROS_INFO_STREAM("Debug: collision checker");


    double obx,oby;
    //    double rubber=-0.10;
    double rubber=+0.30;

    //add all the static obstacles
if(DEB_RRT>0)
    ROS_INFO_STREAM("Debug: Adding static Obstacles");




    for (size_t i = 0; i < obstacle_positions.size(); i++) {
        Tobstacle l = obstacle_positions[i];
        region<2> obstaclei;

        // obx=floor((l.x)/l.cell_width)*l.cell_width;
        // oby=ceil((l.y)/l.cell_height)*l.cell_height;

        // obstaclei.center[0]=obx+l.cell_width/2;

        // if(oby>l.y)
        //     obstaclei.center[1]=oby-l.cell_height/2;
        // else
        //     obstaclei.center[1]=oby+l.cell_height/2;

        obstaclei.center[0]=l.x;
        obstaclei.center[1]=l.y;

        obstaclei.size[0]=2*(l.cell_width)+rubber;
        obstaclei.size[1]=2*(l.cell_height)+rubber;
        collision_checker.add_obstacle(obstaclei);

    }







    // 3.b Initialize the model checker and the cost evaluator
if (DEB_RRT>0)
    ROS_INFO_STREAM("Debug: goal region");

    region<2> region_goal;
    region_goal.center[0] =goal_x_ ;
    region_goal.center[1] =goal_y_ ;
    region_goal.size[0] = toll_goal_/2;
    region_goal.size[1] = toll_goal_/2;


if(DEB_RRT>0)
    ROS_INFO_STREAM("Goal Region: x:"<<region_goal.center[0]<<" y:"<<region_goal.center[1]);


    min_time_reachability.set_goal_region (region_goal);




    // 3.c Initialize the planner to commanded Agent position ** TODO ** We can get the initial position
    state_t *state_initial = new state_t;
    
    state_initial->state_vars[0] = robot_pose_.pose.position.x;
    state_initial->state_vars[1] = robot_pose_.pose.position.y;
    state_initial->state_vars[2] = robot_yaw;

    ROS_INFO("Robot position provided into the planner -> %f, %f, %f .",state_initial->state_vars[0],state_initial->state_vars[1],state_initial->state_vars[2]);

    


    planner.initialize (state_initial);





    ROS_DEBUG_STREAM("Debug: RRT* iterations");

    //** 4. RRT* Planning iterations
    /// Set to zero the clock counters for:
    /// min_time_reachability.begin_time (and .end_time) --> the Time needed to find the first Trajectory;
    /// timeIter_ --> average time for each iteration;
    min_time_reachability.begin_time=clock();
    // min_smoothness.begin_time=clock();
    float timep=0;
    double rw=0;

    int iterations=1;
    clock_t begin_time=0;
    int it=0;


 if(TIMECOUNTER>0){
    ROS_DEBUG_STREAM("Time To Plan: "<<Nit_<<"[s]");
    while(timep<Nit_){ 
    
        begin_time = clock();
        planner.iteration ();
        timep+= float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        it++;

        saveCost(min_time_reachability.cost,timep);
        // saveCost(min_smoothness.cost,timep);

        rw+=planner.nrewiring;
        if (it%1000 == 0) {
            ROS_INFO("Seconds: %f", timep);
            iterations=it;

        }

        if(FIRSTSOLUTION>0){
        /// If solution is found then stop to iterate
           if(min_time_reachability.foundTraj==true){
            // if(min_smoothness.foundTraj==true){
            break;
            }
        }
    }
}

else {
    for (int i = 0; i < Nit_; i++){
        begin_time = clock();
        planner.iteration ();
        timep+= float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        it++;

        saveCost(min_time_reachability.cost,timep);
        // saveCost(min_smoothness.cost,timep);

        rw+=planner.nrewiring;


        if (i%10000 == 0) {
            ROS_INFO("Iteration: %d" , i);
            iterations=i;

        }



        if(FIRSTSOLUTION>0){
        /// If solution is found then stop to iterate
            if(min_time_reachability.foundTraj==true){
        // if(min_smoothness.foundTraj==true){
            break;
            }
        }

    }
}




    

    nrew_=rw/iterations;
    timeIter_=timep/iterations;
    ROS_DEBUG_STREAM( "Average time per Iteration:"<<timeIter_);
    ROS_DEBUG_STREAM("Average numbers of rewiring:"<<nrew_);
    //    sam.close();



    // obtain the final trajectory
    trajectory_t trajectory_final;
    ROS_INFO( "Try to Get Solution " );
    min_time_reachability.get_solution (trajectory_final);
    ROS_DEBUG_STREAM("Time needed to get the first solution: "<<min_time_reachability.end_time);
    // cout<<"Time needed to get the first solution: "<<min_smoothness.end_time<<endl;



    /// close cost file
    cost_.close();






    /// IF NO TRAJECTORY SAVE THE TREE
    if(trajectory_final.list_states.size()==0){
        typedef rrtstar_t planner_t;


        planner_t *planner_int;
        planner_int =&planner;
        //    ofstream gr_st;
        //    gr_st.open("st.txt");
        //    ofstream ver;
        //    ver.open("ver.txt");
        ofstream ind1;
        ind1.open("ind1.txt",std::fstream::in | std::fstream::out | std::fstream::app);

        list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
        ROS_DEBUG_STREAM("Number of Vertex for this iterations >> "<<list_vertices->size());
        ind1<<list_vertices->size()<<endl;
        /// Metrics (timeIter_ saved before)
        numVertices_=list_vertices->size();

        timeSolution_=min_time_reachability.end_time;
        ind1.close();
        // timeSolution_=min_smoothness.end_time;
        saveTree(list_vertices);

        return 0;

    }





    float x,y,theta;
    traj->reset();
    first_problem_solved=1;


if (DEB_RRT>0)
    ROS_DEBUG_STREAM("trajectory evaluation");


    for (typename list<state_t*>::iterator it_state = trajectory_final.list_states.begin();
         it_state != trajectory_final.list_states.end(); it_state++){
        state_t *state_curr = *it_state;
        x = (state_curr)->state_vars[0];
        y = (state_curr)->state_vars[1];
        theta = (state_curr)->state_vars[2];
        traj->addPointEnd(Tpoint(x,y,theta));

    }


    /// Publish the obtained path
    publishPath(traj);



    double v,w ;

    ofstream v_file,w_file;
    v_file.open("v.data",std::fstream::in | std::fstream::out | std::fstream::app);
    w_file.open("w.data",std::fstream::in | std::fstream::out | std::fstream::app);


    for (typename list<input_t*>::iterator it_input = trajectory_final.list_inputs.begin();
         it_input != trajectory_final.list_inputs.end(); it_input++){
        input_t *input_curr = *it_input;
        v=(input_curr)->input_vars[0];
        w=(input_curr)->input_vars[1];
        v_file<<v<<endl;
        w_file<<w<<endl;
        //       cout<<"v: "<<v<<" w: "<<w<<endl;
        traj->addVelocities(v,w);

    }
    v_file.close();
    w_file.close();



    typedef rrtstar_t planner_t;
    planner_t *planner_int;
    planner_int =&planner;

    ofstream ind1;
    ind1.open("ind1.txt",std::fstream::in | std::fstream::out | std::fstream::app);
    list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
    ROS_DEBUG_STREAM("Number of Vertex for this iterations >> "<<list_vertices->size());
    ind1<<list_vertices->size()<<endl;
    ind1.close();
    /// Metrics
    numVertices_=list_vertices->size();
    timeSolution_=min_time_reachability.end_time;
    // timeSolution_=min_smoothness.end_time;
    saveTree(list_vertices);
    publishTree();




    return 1;

}








int main(int argc, char** argv)
{
    // initialize resources
    ros::init(argc, argv, "srl_smp");

    ROS_DEBUG(" srl_smp node initialized");


    ros::NodeHandle node;

    ros::Rate loop_rate(10);


    SRL_smp global_planner(node);
    
    /// ==================
    /// READING PARAMETERS
    /// ==================
    /// define dim of scene
    double x1,x2, y1,y2, csx,csy;
    node.getParam("x1", x1);
    node.getParam("x2", x2);
    node.getParam("y1", y1);
    node.getParam("y2", y2);
    /// cell sizes
    node.getParam("cell_size_x", csx);
    node.getParam("cell_size_y", csy);

    int tcount,firstsol,deburrt;
    node.getParam("TIMECOUNTER", global_planner.TIMECOUNTER);
    node.getParam("FIRSTSOLUTION", global_planner.FIRSTSOLUTION);
    node.getParam("DEB_RRT", global_planner.DEB_RRT);
    node.getParam("BOX",global_planner.BOX);
    node.getParam("RADIUS",global_planner.RADIUS);

    node.getParam("RHO_END",global_planner.RHO_END);
    node.getParam("DISTANCE_METRIC",global_planner.DISTANCE_METRIC);
    node.getParam("DT",global_planner.DT);
    node.getParam("RES_POSQ",global_planner.RES_POSQ);


    /// store dim of scene
    global_planner.xscene_=x2-x1;
    global_planner.yscene_=y2-y1;
    /// store sizes
    global_planner.cellwidth_=csx;
    global_planner.cellheight_=csy;


  
   
    int typepar;
    node.getParam("type_planner", typepar);
    
    int GOAL_PARAM=0;
    node.getParam("GOALPARAM",GOAL_PARAM);

     /// Reading the goal from the PARAMETERS
    double gx,gy,gth,gr;

    node.getParam("goal_x", gx);
    node.getParam("goal_y", gy);
    node.getParam("goal_theta", gth);
    node.getParam("goal_radius", gr);
    
    /// If the goal is read from the params set the Goal properly
    if(GOAL_PARAM)
        global_planner.setGoal(gx,gy,gth,gr);




    /// reading maximum number of iterations
    int nit;
    node.getParam("max_iterations", nit);
    global_planner.Nit_=nit;
    Trajectory *trajectory_= new Trajectory();

    /// ==================
    int result_planning;
    result_planning=0;


    if (global_planner.initialize()) 
    {



        ROS_DEBUG("loaded parameters, starting simulation...");
        
        loop_rate.sleep();
        ros::spinOnce();
 
    while (ros::ok())
        {


        global_planner.publishGoal();
        loop_rate.sleep();
        ros::spinOnce();
        global_planner.publishGoal();
        loop_rate.sleep();
        ros::spinOnce();

        if(global_planner.reset_traj){
                    trajectory_->reset();
                    global_planner.reset_traj=false;

        }


        if(global_planner.obstacle_positions.size()!=0 && (trajectory_->getPath()).size()==0 && global_planner.goal_init_){
            result_planning=global_planner.plan(trajectory_, typepar);
            
            if(!result_planning){
                global_planner.goal_init_=false;
            }
        }


        loop_rate.sleep();
        ros::spinOnce();
        global_planner.publishGoal();

        loop_rate.sleep();
        ros::spinOnce();

    }


   
    }
    else
    return EXIT_FAILURE;



    return EXIT_SUCCESS;    

}

