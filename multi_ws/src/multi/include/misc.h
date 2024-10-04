#pragma once
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include <iostream>
#include <jsoncpp/json/json.h> 
using namespace std;

void getRobotsAndLidars(shared_ptr<World> worldSharedPointer, Json::Value root);