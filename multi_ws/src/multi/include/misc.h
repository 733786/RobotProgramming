#pragma once
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include <iostream>
#include <jsoncpp/json/json.h> 
using namespace std;

std::shared_ptr<Robot> getRobotsAndLidars(std::shared_ptr<World> worldSharedPointer, Json::Value root);