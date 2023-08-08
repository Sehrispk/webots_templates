// File:          CarenController.cpp
// Date:          03.08.23
// Description:   Caren Controller for agent Simulation. Connects to Cedar.
// Author:        Stephan Sehring

#include <webots/Supervisor.hpp>
using namespace webots;
#include "Caren.h"

int main(int argc, char **argv)
{
  // load configuration file
  std::string configFileName = "default_config";
  if (argc > 1) //read configuration
  {
    configFileName = std::string(argv[1]);
  }

  // create E_Puck instance
  Caren* caren = new Caren(configFileName);

  // get the time step of the current world.
  int timeStep = (int)caren->getBasicTimeStep();

  // main loop
  while (caren->step(timeStep) != -1) 
  {
    caren->update();
  }

  // clean up
  delete caren;

  return 0;
}
