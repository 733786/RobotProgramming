#pragma once
#include <cstdint>
#include <opencv2/opencv.hpp>

using namespace std;

struct WorldItem;

class World {
 public:
  World();

  inline uint8_t& at(const IntPoint& p) { return grid[cols * p.x() + p.y()]; }

  inline uint8_t at(const IntPoint& p) const { return grid[cols * p.x() + p.y()]; }

  inline bool inside(const IntPoint& p) const {
    return p.x() >= 0 && p.y() >= 0 && p.x() < rows && p.y() < cols;
  }

  bool collides(const IntPoint& p, const int& radius) const;

  inline IntPoint world2grid(const Point& p) {
    return IntPoint(p.x() * inv_res, p.y() * inv_res);
  }

  inline Point grid2world(const IntPoint& p) {
    return Point(p.x() * res, p.y() * res);
  }

  void loadFromImage(const string filename);

  bool traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                    const float angle, const int max_range);

  void draw();
  void timeTick(float dt);
  void add(WorldItem* item);

  int rows = 0;
  int cols = 0;
  int size = 0;
  float res = 0.05, inv_res = 20.0;

  string worldFrameID = "odom";

  vector<WorldItem*> _items;
  vector<uint8_t> grid;

  cv::Mat _display_image;

};