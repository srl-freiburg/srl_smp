// Simple Orientation Handler for a Wheeled Mobile Robot
// Once you construct the object, you can get the quaternion's elements
// Normalized Quaternion Elements:
//  qx_,  qy_,  qz_,  qw_;
// Rotation Matrix
// rotation_matrix (Eigen::Matrix3f)
//	
// v 0.1.0 Luigi Palmieri,Social Robotics Lab, Uni Freiburg
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>






class OrientationHandler
{

private:

	/// Quaternion elements 
	double qx_;
	double qy_;
	double qz_;
	double qw_;

	/// robot's heading angle
	double theta_;

	/// Rotation Matrix
	Eigen::Matrix3f rotation_matrix_;



public:

	/// Constructor to call if you know the heading of the robot in a xy plane 
	OrientationHandler(double theta);
	/// Constructor to call if you know the velocity's components of the robot in a xy plane
	OrientationHandler(double vx, double vy);
	/// Constructor to call if you know the quaternion's components 
	OrientationHandler(double qw, double qx, double qy, double qz);

	~OrientationHandler();
	/// Computes and returns the robot's heading angle Theta (in a xy plane) given its quaternion
	double getTheta(Eigen::Quaternionf quaternion);

	/// get the robot's heading angle Theta computed by the constructor
	double gett();

	/// get the quaternion x component of the current robot orientation
	double getQx();
	/// get the quaternion y component of the current robot orientation
	double getQy();
	/// get the quaternion z component of the current robot orientation
	double getQz();
	/// get the quaternion w component of the current robot orientation
	double getQw();








};