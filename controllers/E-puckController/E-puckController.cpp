// File:          E-puckController.cpp
// Date:          10.08.21
// Description:   E-Puck Controller for agent Simulation. Connects to Cedar.
// Author:        Stephan Sehring

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <iostream>
#include <map>

// all the webots classes are defined in the "webots" namespace
using namespace webots;
#include "tcp_communication_looper.h"
#include "E_Puck.h"


int main(int argc, char **argv) {
  // load configuration file
  std::string configFileName = "default_config";
  if (argc > 1) //read configuration
  {
    configFileName = std::string(argv[1]);
  }

  // create E_Puck instance
  E_Puck* epuck = new E_Puck(configFileName);

  // get the time step of the current world.
  int timeStep = (int)epuck->getBasicTimeStep();

  // main loop
  while (epuck->step(timeStep) != -1) 
  {
    epuck->update();
  }

  // clean up
  delete epuck;

  return 0;
}
