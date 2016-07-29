#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "icub_tf_publisher");
  ros::NodeHandle n;
  tf::TransformBroadcaster tf_broadcaster;
  tf::Transform icub_tf;
  static tf::TransformBroadcaster br;
  ros::Rate loop_rate(60);
  // ros::Duration sleeper(75.0/1000.0);

  while( ros::ok() )
    {
      icub_tf.setRotation(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), 0));
      icub_tf.setOrigin(tf::Vector3(0, 0, 0.95));
      br.sendTransform(tf::StampedTransform(icub_tf, ros::Time::now(), "world", "base_link"));
      // sleeper.sleep();

      ros::spinOnce();

      loop_rate.sleep();
    }

  return 0;
}
