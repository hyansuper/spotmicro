#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "static_stability");
  ros::NodeHandle n, priv_nh("~");
  double r, scale, support_foot_height_max;
  priv_nh.param("rate", r, 10.0);
  priv_nh.param("marker_scale", scale, 0.05);
  priv_nh.param("support_foot_height_max", support_foot_height_max, 0.001);
  ros::Rate rate(r);

  visualization_msgs::Marker cog, support_plain, text;
  cog.type = visualization_msgs::Marker::POINTS;
  support_plain.type = visualization_msgs::Marker::LINE_STRIP;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  cog.header.frame_id = support_plain.header.frame_id = text.header.frame_id = "odom";
  cog.ns = support_plain.ns = text.ns = "static_stability";
  text.id = 0; cog.id = 1; support_plain.id = 2;
  cog.action = support_plain.action = visualization_msgs::Marker::ADD;
  cog.pose.orientation.w = support_plain.pose.orientation.w = text.pose.orientation.w = 1.0;
  cog.scale.x = cog.scale.y = support_plain.scale.x = scale; text.scale.z = scale*2;
  support_plain.color.a = cog.color.a = text.color.a = 0.8;
  support_plain.color.g = cog.color.g = 1.0f;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("stability_marker", 5);
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);

  geometry_msgs::TransformStamped body_tf;
  geometry_msgs::TransformStamped feet_tf[4];
  std::string feet_name[] = {"lfd", "lrd", "rrd", "rfd"};

  cog.points.push_back(geometry_msgs::Point());
  geometry_msgs::Point& cog0 = cog.points.at(0);

  std::vector<tf2::Vector3> vs;
  tf2::Vector3 z_axis(0, 0, 1);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped body_tf;
    try {
      body_tf = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1));
      for(int i=0; i<4; i++) {
        feet_tf[i] = tfBuffer.lookupTransform("odom", feet_name[i], ros::Time(0), ros::Duration(1));
      }
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1).sleep();
      continue;
    }

    geometry_msgs::Vector3& body = body_tf.transform.translation;
    text.pose.position.z = body.z + 0.1;
    cog0.x = text.pose.position.x = body.x;
    cog0.y = text.pose.position.y = body.y;
    cog.header.stamp = text.header.stamp = body_tf.header.stamp;

    support_plain.points.clear();
    vs.clear();
    for(int i=0; i<4; i++) {
      geometry_msgs::Vector3& foot = feet_tf[i].transform.translation;
      if(foot.z <= support_foot_height_max) {
        geometry_msgs::Point p;
        p.x = foot.x;
        p.y = foot.y;
        support_plain.points.push_back(p);
        tf2::Vector3 v(foot.x-body.x, foot.y-body.y, 0);
        vs.push_back(v);
      }
    }
    if(support_plain.points.size()>2)
      support_plain.points.push_back(support_plain.points.at(0));
    support_plain.header.stamp = feet_tf[3].header.stamp;

    // check if cog stays within support_plane
    if(vs.size()>2) {
      double angle_sum = 0;
      double min_dist = 99999;
      for(int i=0; i<vs.size(); i++) {
        angle_sum += vs[i].angle(vs[(i+1)%vs.size()]);
        double dist = (vs[i]-vs[(i+1)%vs.size()]).normalize().cross(vs[i]).length();
        min_dist = min_dist<dist ? min_dist : dist;
      }      
      if(angle_sum >= M_PI*2) {
        text.text = "Stabiliy Margin: "+ std::to_string(min_dist);
        cog.color.g = text.color.g = 1;
        cog.color.r = text.color.r = 0;
      } else {
        text.text = "Stabiliy Margin: "+ std::to_string(-min_dist);
        cog.color.g = text.color.g = 0;
        cog.color.r = text.color.r = 1;
      }
    } else {
      //when only less than 3 legs is in support mode
    }
    marker_pub.publish(text);
    marker_pub.publish(cog);
    marker_pub.publish(support_plain);
    
    rate.sleep();
  }

  return 0;
}

