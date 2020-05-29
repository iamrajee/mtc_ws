#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";


int main(int argc, char** argv){
    ros::init(argc, argv, "tfBroadcaster");
    ros::NodeHandle pnh("~");




	std::string object_name;
	geometry_msgs::Pose object_pose, place_pose;
	std::size_t errors = 0;
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", object_pose);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "mesh_name", object_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "mesh_pose", object_pose); 
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors); //exit if no. error > 0

	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
	// tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	// transform.setRotation(q);
	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));





    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while(pnh.ok()){
      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(object_pose.orientation.x,object_pose.orientation.y,object_pose.orientation.z,object_pose.orientation.w), tf::Vector3(object_pose.position.x, object_pose.position.y, object_pose.position.z)),
          ros::Time::now(),"world_new", "start"));
	  
	  broadcaster.sendTransform(
		tf::StampedTransform(
          tf::Transform(tf::Quaternion(place_pose.orientation.x,place_pose.orientation.y,place_pose.orientation.z,place_pose.orientation.w), tf::Vector3(place_pose.position.x, place_pose.position.y, place_pose.position.z)),
          ros::Time::now(),"world_new", "end"));
      r.sleep();
    }
}