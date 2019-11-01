#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

double normalizeAngle(double angle){
  while(angle > M_PI) angle -= 2*M_PI;
  while(angle < -M_PI) angle += 2*M_PI;
  return angle;
}
class Robot{
public:
  Robot(ros::NodeHandle& nh): nh_(nh){
    initSubs();
  }

  void initSubs(){
    odom_sub_ = nh_.subscribe("/odom",1, &Robot::odomClk, this);
  }

  bool isInitialized(){return is_initialized_;}
  const double& x() const {return x_;}
  const double& y() const {return y_;}
  const double& yaw() const {return yaw_;}
  const double& v() const {return vx_;}
  const double& yaw_rate() const {return yaw_rate_;}

  bool isNear(const double& x, const double& y, const double& th) const{
    const double dx = x - x_;
    const double dy = y - y_;
    const double d = hypot(dx,dy);

    const double dyaw =  normalizeAngle(th - yaw_);

    ROS_INFO_STREAM("(d , dyaw) : ("<< d << ", " << dyaw << "); \n");
    return d < 0.2 && dyaw < 20*M_PI/180.;
  }

  bool isMoving() const{
    return std::abs(v() > 0.01 || std::abs(yaw_rate()) > 0.01 );
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  double x_, y_, yaw_;
  double vx_, yaw_rate_;
  bool is_initialized_;
  tf::TransformListener listener_;

  void odomClk(const nav_msgs::Odometry::ConstPtr& msg){    
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped map_pose;
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;
    try{
      // listener_.transformPose("/map", msg->pose.pose, msg->header.frame_id, 
      //                          msg->header.stamp, map_pose);
      listener_.transformPose("/map", odom_pose, map_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    x_ = map_pose.pose.position.x;
    y_ = map_pose.pose.position.y;

    vx_ = msg->twist.twist.linear.x;
    yaw_rate_ = msg->twist.twist.angular.z;
    const double& quatx = map_pose.pose.orientation.x;
    const double& quaty = map_pose.pose.orientation.y;
    const double& quatz = map_pose.pose.orientation.z;
    const double& quatw = map_pose.pose.orientation.w;
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);

    is_initialized_ = true;
  }
};

int main( int argc, char** argv )
{

  float pickup_pose[3]  = {0, 3.4, 1};
  float drop_off_pose[3] = {-4, -4, 1};
  
  ros::init(argc, argv, "add_markers");
  ROS_INFO_STREAM("add_markers node started");
  ros::NodeHandle n;  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  Robot robot(n);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickup_pose[0];
  marker.pose.position.y = pickup_pose[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;


  // Publish the marker at the pickup zone
  marker_pub.publish(marker);

  ros::Rate r(25.0);
  //Wait to init the robot
  while(n.ok() && !robot.isInitialized()){
    r.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM_THROTTLE(1,"Waiting to init robot pose!");
  }

  // Wait the robot to pick the object
  ROS_INFO_STREAM("Wait the robot to reach objec to pick !");  
  while(n.ok()){
    r.sleep();
    ros::spinOnce();

    if(robot.isNear(pickup_pose[0], pickup_pose[1], pickup_pose[2]) &
        !robot.isMoving()){
      break;
    }
  }

  ROS_INFO_STREAM("Picking up the object !");  
  if(!n.ok()) return -1;

  // Wait to load the marker
  ros::Duration(5).sleep();

  // Hide the marker
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  // Wait the robot to reach the drop off zone
  r.reset();
  while(n.ok()){
    r.sleep();
    ros::spinOnce();    
    if(robot.isNear(drop_off_pose[0], drop_off_pose[1], drop_off_pose[2]) &
        !robot.isMoving()){
      break;
    }
  }
  ROS_INFO_STREAM("Dropping off the object !");  
  // Publish the marker at the drop off zone
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD; 
  marker.pose.position.x = drop_off_pose[0];
  marker.pose.position.y = drop_off_pose[1];  
  marker_pub.publish(marker);

}
