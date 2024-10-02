#include "robot.h"
#include "types.h"

#include <iostream>
#include <string>

#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace std;

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, shared_ptr<World> w_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(w_, pose_, frame_id_), nh("~") {
    id = id_;
    type = type_;
    frame_id = frame_id_;
    namespc = namespace_;
    radius = radius_;
    max_rv = max_rv_;
    max_tv = max_tv_;
    parent = parent_;
    w = w_;
    parentFrameID = w_->worldFrameID; // "map"
}

void Robot::draw() {
  int int_radius = radius * world->inv_res;

  IntPoint point = world->world2grid(poseInWorld().translation());
  cv::circle(world->_display_image, cv::Point(point.y(), point.x()), int_radius,
             cv::Scalar::all(0), -1);
}

// Function for clamping velocities based on max_rv and max_tv
void clampVelocity(float& vel, float maxVel, const string& message) {
  if (vel > maxVel) {
    vel = maxVel;
    ROS_WARN_STREAM(message << " Maximum speed reached: " << vel);
  } else if (vel < -maxVel) {
    vel = -maxVel;
    ROS_WARN_STREAM(message << " Maximum speed reached: " << vel);
  }
}


void Robot::timeTick(float dt) {

  Pose motion = Pose::Identity();
  motion.translate(Point(tv * dt, 0));
  motion.rotate(rv * dt);

  Pose next_pose(pose_in_parent * motion);
  IntPoint ip = IntPoint();

  int int_radius = radius * world->inv_res;

  if (!world->collides(ip, int_radius)) { // We have not collided, so let's update the position
    pose_in_parent = next_pose;
  } else { // We have collided. Here we must implement the collision mechanism for stopping both parent/child, but..
    if(isChild) {
      //cout << "Child collided!" << endl;
    } else {
      //cout << "Parent collided!" << endl;
    }
  }
}

void Robot::transformRobot() {
  Pose transformation = poseInWorld();

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, Rotation(transformation.linear()).angle());
  rotation.normalize();

  tf2::Vector3 translation(transformation.translation().x(), transformation.translation().y(), 0.0); // 2D translation, z = 0

  // Now we create the Transform object, by applying the rotation and translation created before.
  tf2::Transform tf_transform(rotation, translation);

  // Translation part. We get the translation values and put them in the transform_stamped.
  transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
  transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
  transform_stamped.transform.translation.z = tf_transform.getOrigin().z();

  // Rotation part. We get the rotation values and put them in the transform_stamped.
  transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
  transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
  transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
  transform_stamped.transform.rotation.w = tf_transform.getRotation().w();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_stamped);
}