#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/LED.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <iostream>
#include <map>
using namespace webots;
#include "utilities.h"

#include "tcp_communication_looper.h"
#include "E_Puck.h"
#include "MovementAttractor.cpp"

void E_Puck::update()
{
  readSensorValues();
  
  ComThreadUpdate();

  applyMotorCommands();
}


void E_Puck::ComThreadUpdate()
{
// write to Cedar
  if (comThread->doesWriteSocketExist("camera"))
  {
    comThread->setWriteMatrix("camera", sensordata.cameraMat);
  }

  if (comThread->doesWriteSocketExist("receiver"))
  {
    comThread->setWriteMatrix("receiver", sensordata.receiverMat);
  }

  if (comThread->doesWriteSocketExist("LEDs_write"))
  {
      comThread->setWriteMatrix("LEDs_write", cedardata.LEDMat);
  }

//read from Cedar
  if (comThread->doesReadSocketExist("motors"))
  {
    cv::Mat commandMatrix = comThread->getReadCommandMatrix("motors");
    if (commandMatrix.rows == 1)
    {
      cedardata.motorMat = commandMatrix;
    }
    else
    {
      cedardata.motorMat = cv::Mat::zeros(1,1,CV_32F);
      cedardata.motorMat.at<float>(0) = 26.0;
    }
  }
  
  if (comThread->doesReadSocketExist("break"))
  {
    cv::Mat commandMatrix = comThread->getReadCommandMatrix("break");
    if (commandMatrix.rows == 1)
    {
      cedardata.breakMat = commandMatrix;
    }
    else{cedardata.breakMat = cv::Mat::zeros(1,1,CV_32F);}
  }
  
  if (comThread->doesReadSocketExist("LEDs_read"))
  {
    cv::Mat commandMatrix = comThread->getReadCommandMatrix("LEDs_read");
    if (commandMatrix.rows == 9)
    {
        cedardata.LEDMat = commandMatrix;
    }
    else{cedardata.LEDMat = cv::Mat::zeros(9,1,CV_32F);}
  }
}


void E_Puck::readSensorValues()
{
  // read camera picture
  cv::Mat cameraPicture;
  int from_to[] =
  { 0, 0, 1, 1, 2, 2 }; // for mat conversion
  cv::Mat pictureMat(cam->getHeight(), cam->getWidth(), CV_8UC4, const_cast<unsigned char*>(cam->getImage()));
  cameraPicture = cv::Mat(cam->getHeight(), cam->getWidth(), CV_8UC3);
  cv::mixChannels(&pictureMat, 1, &cameraPicture, 1, from_to, 3); // kill the alpha channel
  sensordata.cameraMat = cameraPicture;
  
  // read distance sensor value
  for (std::vector<webots::DistanceSensor*>::size_type i=0; i < sensors.size(); i++)
  {
    sensordata.psMat.at<float>(i) = sensors[i]->getValue();
  }
  transform2Distance(sensordata.psMat);

  // read receiver value
  sensordata.receiverMat = cv::Mat::zeros(10,1,CV_32F);
  sensordata.receiverTaskMat = cv::Mat::zeros(8, 1, CV_32F);
  while (rec->getQueueLength() > 0)
  {
      //std::string p((const char*)rec->getData());
      //float packet(std::stof(p));
      if (rec->getDataSize() == 10*sizeof(float))
      {
          memcpy(sensordata.receiverMat.data, rec->getData(), 10*sizeof(float));
          //sensordata.receiverMat = cv::Mat(10, 1, CV_32F, rec->getData());
      }
      else if (rec->getDataSize() == 1)
      {
          //memcpy(sensordata.receiverTaskMat.data, rec->getData(), 3);
          std::string p((const char*)rec->getData());
          float packet(std::stof(p));
          sensordata.receiverTaskMat.at<float>((int)packet) = 1.;
      }
  rec->nextPacket();
  }
}

void E_Puck::applyMotorCommands()
{
  int width = cam->getWidth();
  float fov = cam->getFov();
  float psi_target = ((float)width / 2 - cedardata.motorMat.at<float>(0)) * 2 * fov / width;
  
  float v[2] = {0, 0};
  
  if (cedardata.breakMat.at<float>(0) < 0.75)
  {
    MovementAttractor(sensordata.psMat, psi_target, v);
  }
  
  for (std::vector<webots::Motor*>::size_type i=0; i<motors.size(); i++)
  {
    motors[i]->setVelocity(v[i]);
  }

  em->send(cedardata.LEDMat.data, cedardata.LEDMat.total() * cedardata.LEDMat.elemSize());
  em->send(cedardata.goalMat.data, cedardata.goalMat.total() * cedardata.goalMat.elemSize());
  
  for (std::vector<webots::LED*>::size_type i = 0; i < LEDs.size(); i++)
  {
    LEDs[i]->set((int)(cedardata.LEDMat.at<float>(i)+0.1));
  }
}

void E_Puck::initFromConfig()
{
  // get and enable camera from webots
  std::cout << "Initialize Camera!" << std::endl;
  cam = getCamera("camera");
  cam->enable(this->getBasicTimeStep());
  
  // get motors from webots
  std::cout << "Initialize Motors!" << std::endl;
  motors.push_back(getMotor("left wheel motor"));
  motors.push_back(getMotor("right wheel motor"));
  for (std::vector<webots::Motor*>::size_type i=0; i<motors.size(); i++)
  {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0);
  }
  
  // get and enable receiver and emitter from webots
  std::cout << "Initialize Microphone!" << std::endl;
  rec = getReceiver("receiver");
  rec->setChannel(1);
  rec->enable(this->getBasicTimeStep());
  em = getEmitter("emitter");
  em->setChannel(2);
  
  // get and enable distance sensors from webots
  std::cout << "Initialize distance Sensors!" << std::endl;
  sensors.push_back(getDistanceSensor("ps0"));
  sensors.push_back(getDistanceSensor("ps1"));
  sensors.push_back(getDistanceSensor("ps2"));
  sensors.push_back(getDistanceSensor("ps3"));
  sensors.push_back(getDistanceSensor("ps4"));
  sensors.push_back(getDistanceSensor("ps5"));
  sensors.push_back(getDistanceSensor("ps6"));
  sensors.push_back(getDistanceSensor("ps7"));
  for (std::vector<webots::DistanceSensor*>::size_type i=0; i < sensors.size(); i++)
  {
    sensors[i]->enable(this->getBasicTimeStep());
  }
  
  // get LEDs from webots
  std::cout << "Initialize LEDs!" << std::endl;
  LEDs.push_back(getLED("led0"));
  LEDs.push_back(getLED("led1"));
  LEDs.push_back(getLED("led2"));
  LEDs.push_back(getLED("led3"));
  LEDs.push_back(getLED("led4"));
  LEDs.push_back(getLED("led5"));
  LEDs.push_back(getLED("led6"));
  LEDs.push_back(getLED("led7"));
  LEDs.push_back(getLED("led9"));
  
  // add write sockets
  if (configMap.find("camera_port_snd") != configMap.end())
  {
    comThread->addWriteSocket("camera", std::stoi(configMap["camera_port_snd"]), configMap["cedar_ip"]);
  }
  if (configMap.find("mic_port_snd") != configMap.end())
  {
    comThread->addWriteSocket("receiver", std::stoi(configMap["mic_port_snd"]), configMap["cedar_ip"]);
  }
  if (configMap.find("LED_port_snd") != configMap.end())
  {
      comThread->addWriteSocket("LEDs_write", std::stoi(configMap["LED_port_snd"]), configMap["cedar_ip"]);
  }
  
  // add read sockets
  if (configMap.find("motor_port_rcv") != configMap.end())
  {
    comThread->addReadSocket("motors", std::stoi(configMap["motor_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
  }
  if (configMap.find("break_port_rcv") != configMap.end())
  {
    comThread->addReadSocket("break", std::stoi(configMap["break_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
  }
  if (configMap.find("LED_port_rcv") != configMap.end())
  {
    comThread->addReadSocket("LEDs_read", std::stoi(configMap["LED_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
  }
}

std::map<std::string, std::string> E_Puck::readConfiguration(std::string configFilePath)
{
  std::map<std::string, std::string> cMap;

  std::ifstream config_file(configFilePath);

  if (config_file.is_open())
  {
      std::string line;
      std::cout << "Read from Config:" << std::endl;
      while (std::getline(config_file, line))
      {
          if (line[0] != '#')
          {
              std::vector<std::string> tokens;
              split(line, ":", tokens);
              if (tokens.size() == 2)
              {
                  std::cout << "\t" << tokens[0] << " >> " << tokens[1] << std::endl;
                  cMap[tokens[0]] = tokens[1];
              }
              else
              {
                  std::cout << "Your Config File seems to be faulty. Each line should look like this >>identifier:value<< . Faulty line: " << line
                      << std::endl;
              }
          }
      }
  }
  else
  {
      std::cout << "ERROR! Could not open config file: " << configFilePath << std::endl;
  }

  config_file.close();


  return cMap;
}
