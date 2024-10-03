#include "lidar.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;

Lidar::Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<World> w, const Pose& pose_):
  WorldItem(w,pose_),
  fov(fov_),
  max_range(max_range_),
  num_beams(num_beams_){
  ranges = new float[num_beams];
  }
Lidar::Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<WorldItem> p_, const Pose& pose_):
  WorldItem(p_,pose_),
  fov(fov_),
  max_range(max_range_),
  num_beams(num_beams_){
  ranges = new float[num_beams];
  }
Lidar::~Lidar() {
  if (ranges)
    delete [] ranges;
}
  void Lidar::timeTick(float dt) {
  Pose piw=poseInWorld();
  IntPoint origin=world->world2grid(piw.translation);
  if (! world->inside(origin))
    return;
	  float d_alpha=fov/num_beams;
  float alpha=piw.theta-fov/2;
  float int_range=max_range*world->inv_res;
    
  for (int i=0; i<num_beams; ++i) {
    IntPoint endpoint;
        ranges[i]=max_range;
    bool result=world->traverseBeam(endpoint, origin, alpha, int_range);
    if (result) {
      IntPoint delta = endpoint - origin;
      ranges[i] = sqrt(delta * delta) * world -> res;
    }
    alpha += d_alpha;
  }
}
void Lidar::draw() {
  Pose piw=poseInWorld();
  IntPoint origin=world->world2grid(piw.translation);
  if (! world->inside(origin))
    return;
  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i) {
    float r = ranges[i];
        Point p_lidar(r*cos(alpha), r*sin(alpha));
    Point p_world = piw*p_lidar;
    IntPoint epi=world->world2grid(p_world);
    cv::line(world->_display_image, cv::Point(origin.y, origin.x), cv::Point(epi.y, epi.x), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }
}