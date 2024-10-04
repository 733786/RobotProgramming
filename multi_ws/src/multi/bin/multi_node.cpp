#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "misc.h"

#include <opencv2/highgui.hpp>
#include <jsoncpp/json/json.h>


using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_node");
  ros::NodeHandle nh("/");
  // Load the configuration file and initialize the simulator
  // Create a World object

  // Get current working directory
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        cout << "Current working dir: " << cwd << endl;
    } else {
        perror("getcwd() error");
        return 1;
    }

    // Construct the full path using the current working directory
    string imagePath = string(cwd) + "/multi_ws/src/multi/test_data/";
  
  // We might want a function that returns both maps for Lidars/Robots
  //std::map<int, std::shared_ptr<Robot>> robotsPointersMap; 
  //std::map<int, std::shared_ptr<Lidar>> lidarsPointersMap;
  /*
  * Check if a JSON file path is provided as a command-line argument
  * If not, then we simply terminate our node.
  */
  if (argc != 2) {
      ROS_ERROR("Usage: rosrun mrsim mrsim_node <config_file.json>");
      return 1;
  }

  // We get the path of the JSON file from the arguments
  const std::string jsonFilePath = argv[1];

  // Read and parse the JSON file
  Json::Value root;
  Json::Reader reader;
  std::ifstream jsonFile((string(cwd) + "/multi_ws/src/multi/test_data/" + jsonFilePath).c_str(), std::ifstream::binary); // We need a way to remove absolute path

  // Let's check if the JSON is properly written and decoded.
  if (!jsonFile.good()) {
      ROS_ERROR_STREAM("Failed to open JSON file: " << jsonFilePath << "\nPlease check your JSON config.");
      return 1;
  }

  if (!reader.parse(jsonFile, root)) {
      ROS_ERROR_STREAM("Failed to parse JSON file: " << reader.getFormattedErrorMessages() << "\nPlease check your JSON config.");
      jsonFile.close();
      return 1;
  }

  // We access the map file
  std::string mapFileName = root["map"].asString();

  // Create a World object
  World w = World();
  w.loadFromImage((imagePath + mapFileName).c_str());

  /**
   * Create a shared pointer to the World.
   * We instantiate the World in the same way we instantiate Robots/Lidars in getRobotsAndLidars().
   * Check comments there.
   * */
  std::shared_ptr<World> worldSharedPtr(&w,
                                        [](World* w) {
                                            // Custom cleanup actions here, if needed
                                            //delete w; // Clean up the Robot object
                                        });
  // Get Robots and Lidars
  getRobotsAndLidars(worldSharedPtr, root);
  
  float delay = 0.07;
  int k;

  while (ros::ok()) {

    ros::spinOnce();

    // run a simulation iteration
    w.timeTick(delay);
    w.draw();

    k = cv::waitKeyEx(1)&255;
    if (k == 27) {     // Esc
      ros::shutdown(); // Terminate the ROS node
      break; 
    }
  }

  cv::destroyAllWindows(); // Destroy the CV2 window used for drawing
  return 0;
}