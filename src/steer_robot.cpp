#include "steer_robot.h"
#define TRANS_VEL_ABS_LIMIT 1.2
#define ROT_VEL_ABS_LIMIT 1.57
#define THRS_ERR_X 1
#define THRS_ERR_Y 1


tf::TransformListener* listener;




/// ========================================================================================
/// ========================================================================================

Steer_robot::Steer_robot(const ros::NodeHandle& node) : nhs_(node)
{

  	//nh_.param("scale_angular", a_scale_, a_scale_);
  	//nh_.param("scale_linear", l_scale_, l_scale_);

	current_path_.clear();
	orientations_.clear();



	/// Initial Values for the controller
	k_one_= .51;
	k_two_= .51;
	b_=	 0.15;
    ts_=.1;


 	
}


/// ========================================================================================
/// compute_sigma(double vsig, double wsig), function to compute sigma, used as scaling
/// factor in the velocities computation when one of them is higher than the hardware limits
/// ========================================================================================
double Steer_robot::compute_sigma(double vsig, double wsig) {
	


	double max = vsig; 

	if (wsig > max) { 
		max = wsig;
	} 

	if (1 > max) { 
		max = 1;
	} 

	return max; 
}

/// =======================================================================================
/// set_angle_to_range(double alpha, double min), function used to map the angle beetween a 
/// range of [min , min+2pi]
/// =======================================================================================

double Steer_robot::set_angle_to_range(double alpha, double min)
{

    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}


/// =======================================================================================
/// =======================================================================================
int Steer_robot::initialize(){

  	path_listener_ = nhs_.subscribe("global_path", 5, &Steer_robot::callbackReadPath,this);
  	odom_listener_ = nhs_.subscribe("slam_out_pose",5,&Steer_robot::callbackSetRobotPose,this);

   	vel_publisher_ = nhs_.advertise<daryl_msgs::SetDriveVelocity>("/daryl/control/set_drive_velocity",5);
   	end_track_publisher = nhs_.advertise<std_msgs::Bool>("/steer_robot/end_track",5);

	driving_=0;
  	return 1;
}


/// ============================================================================================
/// callbackSetRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg), callback to store
/// the current robot pose
/// ============================================================================================

void Steer_robot::callbackSetRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg){



    
    robot_pose_.pose=msg->pose;
    robot_pose_.header=msg->header;

    // ROS_INFO("Robot pose %f %f %f ",msg->pose.position.x,msg->pose.position.y,or_curr);


}

/// =============================================================================================
/// callbackReadPath(const nav_msgs::Path::ConstPtr& path_msg), callback to store
/// the current path to follow, it computes also the feedforward commands used to drive the robot
/// =============================================================================================

void Steer_robot::callbackReadPath(const nav_msgs::Path::ConstPtr& path_msg){

	ROS_INFO("Start reading path..");
	current_path_.clear();
	orientations_.clear();



	current_path_.resize(path_msg->poses.size());
	orientations_.resize(path_msg->poses.size());
	 
	if(driving_==0){
		int j=0;
		int npoints=0;
		for(size_t i=0; i<path_msg->poses.size(); i++){



		

		
		if(i>0){
	 		
				{
				 	current_path_[j].x=path_msg->poses[i].pose.position.x;
				 	current_path_[j].y=path_msg->poses[i].pose.position.y;
				    current_path_[j].z=path_msg->poses[i].pose.position.z;
				    ROS_INFO("Path readed by the controller %d %f %f",j,current_path_[j].x,current_path_[j].y);

					orientations_[j].x=path_msg->poses[i].pose.orientation.x;
					orientations_[j].y=path_msg->poses[i].pose.orientation.y;
					orientations_[j].z=path_msg->poses[i].pose.orientation.z;
					orientations_[j].w=path_msg->poses[i].pose.orientation.w;
					j++;
				}


		}


    	}
    	npoints=j;
    	current_path_.resize(npoints+1);
		orientations_.resize(npoints+1);
        ROS_INFO("Computing the feedforward commands..");

    	/// Computing the feedforward commands
    	///  generate \dot{x_d} \dot{y_d} and theta_d
    	dx_d.resize(current_path_.size()-1);
    	dy_d.resize(current_path_.size()-1);
    	thetad.resize(current_path_.size()-1);

		for(size_t i=0; i<dx_d.size()-1; i++){
			
    			dx_d[i]=(current_path_[i+1].x-current_path_[i].x)/ts_;
    			dy_d[i]=(current_path_[i+1].y-current_path_[i].y)/ts_;

    			if(dx_d[i]==0)
    				dx_d[i]=0.01;
				
				if(dy_d[i]==0)
    				dy_d[i]=0.01;



				thetad[i]=atan2(dy_d[i],dx_d[i]);

				ROS_INFO(" %d dx_d: %f dy_d: %f",(int)i,dx_d[i],dy_d[i]);
		}



		///  generate \dot\dot{x_d} \dot\dot{y_d}
		ddx_d.resize(dx_d.size()-1);
    	ddy_d.resize(dy_d.size()-1);
		for(size_t i=0; i<ddx_d.size(); i++){
			
    			ddx_d[i]=(dx_d[i+1]-dx_d[i])/ts_;
    			ddy_d[i]=(dy_d[i+1]-dy_d[i])/ts_;
		}


		///  generate v_d and w_d
	    vd.resize(dx_d.size()-1);
		for(size_t i=0; i<vd.size(); i++){
			
				vd[i]=sqrt(dx_d[i]*dx_d[i]+dy_d[i]*dy_d[i]);
    			if(vd[i]==0)
    				vd[i]=0.1;
		}

		wd.resize(ddx_d.size()-1);
		double den;
		for(size_t i=0; i<wd.size(); i++){
				
				den=(dx_d[i]*dx_d[i]+dy_d[i]*dy_d[i]);
				
				if(den==0)
					den=0.1;
    		
    			wd[i]=(ddy_d[i]*dx_d[i]-ddx_d[i]*dy_d[i])/den;
    			
    			if(i>1){
   					 if((wd[i]-wd[i-1])>M_PI)
        				wd[i]=M_PI;
   				}
    			
		}

    	driving_=1;
	}

   


}


/// ========================================================================================
/// steerLoop(), Loop to drive the robot along the received path
/// ========================================================================================
void Steer_robot::steerLoop()
{


  daryl_msgs::SetDriveVelocity cmd_vel;
  double curr_v,feedback_v,vdx,vdy;
  double curr_w,feedback_w;
  double x_curr,y_curr,theta_curr;
  std_msgs::Header header;

  ros::Rate r(1/ts_); // 10 hz
 
  std_msgs::Bool end_track;
  end_track.data=false;
  end_track_publisher.publish(end_track);

  while(ros::ok())
  {


    r.sleep();
    ros::spinOnce();
  	ROS_INFO("Steer_robot size of the path %d",(int)current_path_.size());

    // If a new path to come
    if(current_path_.size()>0 && driving_==1)
    {
     // ROS_INFO("Steer_robot IF publishing state	");


    for(size_t i=0; i<dx_d.size(); i++){
 		


 		if(i==0){

 			curr_v=1;
 			curr_w=.001;


 		} 
 		else
 		{
 			curr_v=feedback_v;
 			curr_w=feedback_w;

 			if((fabs(curr_v)>TRANS_VEL_ABS_LIMIT) || fabs(curr_w)>ROT_VEL_ABS_LIMIT){

				ROS_INFO(" Velocities tooo HIGH!!! v: %f w: %f",curr_v,curr_w);
 				double vsig,wsig,sig;
				vsig=fabs(curr_v)/TRANS_VEL_ABS_LIMIT;
				wsig=fabs(curr_w)/ROT_VEL_ABS_LIMIT;

				sig=compute_sigma(vsig,wsig); 


				int sign_v;
 				int sign_w;
				if(curr_v>0)
 					sign_v=1;
 				else
 					sign_v=-1;

 				if(curr_w>0)
 					sign_w=1;
 				else
 					sign_w=-1;


				if(sig==1){

					curr_v=curr_v;
					curr_w=curr_w;

				}	

				if(sig==vsig){

					curr_v=sign_v*TRANS_VEL_ABS_LIMIT;
					curr_w=curr_w/sig;

				}	



				if(sig==wsig){

					curr_v=curr_v/sig;
					curr_w=sign_w*ROT_VEL_ABS_LIMIT;

				}	





 				
 			}
 		}

 		ROS_INFO(" Velocities index: %d v: %f w: %f",(int)i,curr_v,curr_w);
 		/// Publish command
 		cmd_vel.header.stamp = ros::Time();
		cmd_vel.trans_vel = curr_v;
		cmd_vel.rot_vel = curr_w;



		vel_publisher_.publish(cmd_vel);
   	    ros::spinOnce();
   	    r.sleep();

   	    


        tf::Quaternion q(robot_pose_.pose.orientation.x,robot_pose_.pose.orientation.y,robot_pose_.pose.orientation.z,robot_pose_.pose.orientation.w);
        tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		x_curr=robot_pose_.pose.position.x;
        y_curr=robot_pose_.pose.position.y;
        theta_curr=yaw;
        theta_curr=set_angle_to_range(theta_curr,0);


 		ROS_INFO(" robot pose x: %f y: %f theta:%f ",x_curr,y_curr,theta_curr);
		//ROS_INFO("  dx_d: %f dy_d: %f",dx_d[i],dy_d[i]);
        double err_x;
       	double err_y;
       	err_y=(current_path_[i].y-(y_curr+b_*sin(theta_curr)));
       	err_x=(current_path_[i].x-(x_curr+b_*cos(theta_curr)));
        

	   if( err_y> THRS_ERR_Y || err_x>THRS_ERR_X){ 
	        		ROS_ERROR("ERROR, feedback error too high!!! in x %f in y %f",err_x,err_y);
	        		return;
	    }

        
   	    vdx=dx_d[i]+k_one_*err_x;
   	    vdy=dy_d[i]+k_two_*err_y;

		// ROS_INFO(" index= %d vdx: %f vdy: %f",(int)i,vdx,vdy);

   	    feedback_v=vdx*cos(theta_curr)+vdy*sin(theta_curr);
		feedback_w=(-1/b_)*vdx*sin(theta_curr)+(1/b_)*vdy*cos(theta_curr);
		// ROS_INFO(" index= %d feedback_v: %f feedback_w: %f",(int)i,feedback_v,feedback_w);

    }

    /// Waiting for a new path coming
	current_path_.clear();
	driving_=0;
	end_track.data=true;
	end_track_publisher.publish(end_track);
  	
  	}

  }
  return;
}



/// ========================================================================================
/// ========================================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "steer_robot");
    ros::NodeHandle node;
    Steer_robot robot(node);


    node.getParam("Ts",robot.ts_);
    node.getParam("B",robot.b_);
    node.getParam("KONE",robot.k_one_);
    node.getParam("KTWO",robot.k_two_);


    listener = new tf::TransformListener();

  	ROS_INFO("Steer_robot started");
    ros::spinOnce();

    ROS_INFO("Steer_robot first spin");


    if (robot.initialize()) 
    	    robot.steerLoop();


    delete(listener);
  
    return(0);
}