// INCLUIR LA LIBRERIA DE ROS
#include "ros/ros.h"

// INCLUIR LOS TIPOS DE DATOS A USAR
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <std_srvs/Empty.h>
#include "tf/transform_broadcaster.h"

// INCLUIR RESTO DE LIBERIAS A UTILIZAR
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>



/** Normalize an angle to within +/_ M_PI. */
inline double normalize( double a )
{
	 while( a < -M_PI ) a += 2.0*M_PI;
	 while( a >  M_PI ) a -= 2.0*M_PI;	 
	 return a;
};

// DECLARAR LA CLASE
class local_odom{
public:
	// DECLARAR EL CONSTRUCTOR
	local_odom();
	// DECLARAR LAS FUNCIONES Y CALLBACKS A UTILIZAR
	void odomCallback(const boost::shared_ptr<nav_msgs::Odometry const>& msg);
	bool reset_odom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

private:
	// DECLARAR LOS SUBSCRIBER Y PUBLISHER
	ros::Subscriber odom_sub_;
	ros::Publisher  error_odom_pub_;
	// DECLARAR EL TIMER
	ros::Timer timer_;
	// DECLARAR LAS VARIABLES A USAR
	nav_msgs::Odometry actualOdom,  errorOdom;
	ros::ServiceServer reset_server;
	tf::TransformBroadcaster tf_broadcast;

	double yaw_ant, x_ant, y_ant;
	double int_error_x, int_error_y, int_error_a;
	double errorx, errory, errora;

	
};

// CALLBACK
void local_odom::odomCallback(const boost::shared_ptr<nav_msgs::Odometry const>& msg)
{
	// TIME DIF
	double dt= msg->header.stamp.toSec() - actualOdom.header.stamp.toSec();
	actualOdom=*msg;
	double yaw=tf::getYaw(actualOdom.pose.pose.orientation);
	// NEW ANGULAR POSITION FROM VELOCITY + ERROR
	double est_pose_a = normalize( yaw_ant + (actualOdom.twist.twist.angular.z * dt) * (1.0 + errora));
	double cosa = cos(est_pose_a);
	double sina = sin(est_pose_a);
        // NEW X-Y POSITION FROM VELOCITY + ERROR
	double dx = (actualOdom.twist.twist.linear.x * dt) * (1.0 + errorx);
	double dy = (actualOdom.twist.twist.linear.y * dt) * (1.0 + errory);
	double est_posex = x_ant + dx * cosa + dy * sina;
	double est_posey = y_ant - dy * cosa + dx * sina;

	// CREATE NEW POSITION MESSAGE
	errorOdom=*msg;
	errorOdom.pose.pose.position.x=est_posex;
	errorOdom.pose.pose.position.y=est_posey;
	errorOdom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(est_pose_a);

	yaw_ant=est_pose_a;
	x_ant = est_posex;
	y_ant = est_posey;
	errorOdom.header.frame_id="odom";
	error_odom_pub_.publish(errorOdom);

	// GENERAR TRANSFORMADA

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(est_posex,est_posey,0));
	tf::Quaternion q;
	q.setRPY(0, 0, est_pose_a);
	transform.setRotation(q);
	tf_broadcast.sendTransform(tf::StampedTransform(transform,msg->header.stamp,"odom","robot0"));

}

bool local_odom::reset_odom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {

   ROS_INFO_STREAM("ODOM RESET!");
   errorOdom.pose.pose.position.x=0;
   errorOdom.pose.pose.position.y=0;
   errorOdom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(0);
   yaw_ant= 0;
   x_ant = 0;
   y_ant = 0;

  return true;
}


// CONSTRUCTOR
local_odom::local_odom(){
  // MANEJADOR DE NODOS DE ROS
  ros::NodeHandle n("~");
  // SUBSCRIBERS
  odom_sub_ = n.subscribe<nav_msgs::Odometry> ("/robot0/odom", 1, &local_odom::odomCallback, this);
  
  // PUBLISHERS
  error_odom_pub_ = n.advertise<nav_msgs::Odometry> ("/robot0/local_odom", 1, true);

  reset_server = n.advertiseService("reset_odom", &local_odom::reset_odom, this);

  // INICIALIZAR VARIABLES
  if (!n.getParam("error_x", int_error_x))
  {
  int_error_x = 0.05;
  }
  if (!n.getParam("error_y", int_error_y))
  {
  int_error_y = 0.05;
  }
  if (!n.getParam("error_a", int_error_a))
  {
  int_error_a = 5*M_PI/180;
  }
  errorx = drand48() * int_error_x - int_error_x / 2.0;
  errory = drand48() * int_error_y - int_error_y / 2.0;
  errora = drand48() * int_error_a - int_error_a / 2.0;

   yaw_ant= 0;
   x_ant = 0;
   y_ant = 0;

}

// PROGRAMA PRINCIPAL
int main(int argc, char **argv)
{
  // INICIALIZAR ROS
  ros::init(argc, argv, "local_odom");
  // CARGAR LA CLASE
  local_odom pr2_node;
  // LANZAR EL NODO
  ros::spin();
}
