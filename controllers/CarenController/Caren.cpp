#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Connector.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <iostream>
#include <map>
using namespace webots;

#include "tcp_communication_looper.h"
#include "Caren.h"

#include "kinematics.h"

Caren::Caren(std::string configFilePath) 
{
    std::cout << "Create Caren with ConfigFile: " << configFilePath << std::endl;
    // read config
    configMap = readConfiguration(configFilePath);
    //Create Communication Thread
    comThread = std::make_unique<ComLooper>();
    // init from config
    initFromConfig();
    // start comthread
    comThread->run();
    // init structs
    sensordata = SensorData{cv::Mat(1,1,CV_32F), cv::Mat::zeros(7,1,CV_32F), cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(1,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(100, 100, CV_8UC4), cv::Mat::zeros(100,100, CV_32F)};
    cedardata = CedarData{cv::Mat::zeros(1,1,CV_32F), cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(1,1,CV_32F), cv::Mat::zeros(2,1,CV_32F)};
};

Caren::~Caren()
{
    if (comThread->isRunning())
    {
        comThread->stop();
        comThread = nullptr;
    }
};

void Caren::update()
{
  readSensorValues();
  
  ComThreadUpdate();

  applyMotorCommands();
}

void Caren::ComThreadUpdate()
{
    // write to cedar
    if (comThread->doesWriteSocketExist("camera"))
    {
        comThread->setWriteMatrix("camera", sensordata.cameraPicture);
    }

    if (comThread->doesWriteSocketExist("cube"))
    {
        comThread->setWriteMatrix("cube", sensordata.cubePosition);
    }

    if (comThread->doesWriteSocketExist("arm"))
    {
        sensordata.eefPosition.at<float>(0) = -(sensordata.eefPosition.at<float>(0)-0.4) * 60/0.8;
        sensordata.eefPosition.at<float>(1) = (sensordata.eefPosition.at<float>(1)-0.8) * 20/0.23;
        sensordata.eefPosition.at<float>(2) = (sensordata.eefPosition.at<float>(2)+0.8) * 180/1.6;
        comThread->setWriteMatrix("arm", sensordata.eefPosition);
    }

    if (comThread->doesWriteSocketExist("connector"))
    {
        comThread->setWriteMatrix("connector", sensordata.connectorStatus);
    }

    if (comThread->doesWriteSocketExist("head"))
    {
        sensordata.headFixation.at<float>(0) = -(sensordata.headFixation.at<float>(0)-0.4) * 60/0.8;
        sensordata.headFixation.at<float>(1) = (sensordata.headFixation.at<float>(1)-0.8) * 20/0.23;
        sensordata.headFixation.at<float>(2) = (sensordata.headFixation.at<float>(2)+0.8) * 180/1.6;
        comThread->setWriteMatrix("head", sensordata.headFixation);
    }

    if (comThread->doesWriteSocketExist("caren"))
    {
        sensordata.carenPosition.at<float>(0) = (sensordata.carenPosition.at<float>(0)+0.8) * 180/1.6;
        comThread->setWriteMatrix("caren", sensordata.carenPosition);
    }

    // read from cedar
    if (comThread->doesReadSocketExist("arm"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("arm");
        if (commandMatrix.rows == 3)
        {
            cedardata.eefPosition = commandMatrix;
            cedardata.eefPosition.at<float>(0) = -cedardata.eefPosition.at<float>(0)/60.0 * 0.8 + 0.4; // -0.4 to 0.4 with dim size 60
            cedardata.eefPosition.at<float>(1) = cedardata.eefPosition.at<float>(1)/20.0 * 0.23 + 0.8; //0 in field corresponds to table hight 0.8-1.03
            cedardata.eefPosition.at<float>(2) = cedardata.eefPosition.at<float>(2)/180.0 * 1.6 - 0.8; // -0.8 to 0.8 fith dim size 0f 180
        }
        else
        {
            cedardata.eefPosition = cv::Mat::zeros(3,1,CV_32F);
            cedardata.eefPosition.at<float>(1) = 0.8; //0 in field corresponds to table hight 0.7-1.03
        }
    }

    if (comThread->doesReadSocketExist("connector"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("connector");
        if (commandMatrix.rows == 1)
        {
            cedardata.connectorStatus = commandMatrix;
        }
        else
        {
            cedardata.connectorStatus = cv::Mat::zeros(2,1,CV_32F);
        }
    }

    if (comThread->doesReadSocketExist("head"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("head");
        if (commandMatrix.rows == 2)
        {
            cedardata.headFixation = commandMatrix;
            cedardata.headFixation.at<float>(0) = -cedardata.headFixation.at<float>(0)/60.0 * 0.8 + 0.4; // -0.4 to 0.4 with dim size 60
            cedardata.headFixation.at<float>(1) = cedardata.headFixation.at<float>(1)/180.0 * 1.6 - 0.8; // -0.8 to 0.8 fith dim size 0f 180
        }
        else
        {
            cedardata.headFixation = cv::Mat::zeros(2,1,CV_32F);
        }
    }

    if (comThread->doesReadSocketExist("caren"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("caren");
        if (commandMatrix.rows == 1)
        {
            cedardata.carenPosition = commandMatrix;
            cedardata.carenPosition.at<float>(0) = cedardata.carenPosition.at<float>(0)/180.0 * 1.6 - 0.8;
        }
        else
        {
            cedardata.carenPosition = cv::Mat::zeros(1,1,CV_32F);
        }
    }
}

void Caren::readSensorValues()
{
    // camera
    cv::Mat cameraPicture;
    int from_to[] = { 0, 0, 1, 1, 2, 2 }; // for mat conversion
    cv::Mat pictureMat(camera->getHeight(), camera->getWidth(), CV_8UC4, const_cast<unsigned char*>(camera->getImage()));
    cameraPicture = cv::Mat(camera->getHeight(), camera->getWidth(), CV_8UC3);
    cv::mixChannels(&pictureMat, 1, &cameraPicture, 1, from_to, 3); // kill the alpha channel
    sensordata.cameraPicture = cameraPicture;

    // distance camera
    cv::Mat distancePicture(distance_camera->getHeight(), distance_camera->getWidth(), CV_32F, const_cast<float*>(distance_camera->getRangeImage()));
    sensordata.distancePicture = distancePicture;

    //joint angles
    for (std::vector<webots::PositionSensor*>::size_type i=0; i < joint_sensors.size(); i++)
    {
        sensordata.jointAngles.at<float>(i) = joint_sensors[i]->getValue();
    }

    ////////////////////////////////////// COORDINATE TRANSFORMATIONS MISSING ////////////////////////////////////////////////

    // caren position (caren coordinate system)
    sensordata.carenPosition.at<float>(0) = caren_node->getPosition()[2];

    // cube position
    sensordata.cubePosition.at<float>(0) = cube_sensor->getValue();

    // head angles
    for (std::vector<webots::PositionSensor*>::size_type i=0; i < head_sensors.size(); i++)
    {
        sensordata.headAngles.at<float>(i) = head_sensors[i]->getValue();
    }

    // connector status
    sensordata.connectorStatus.at<float>(0) = connector->isLocked();
    sensordata.connectorStatus.at<float>(1) = connector->getPresence();

    // endeffector position (allocentric coordinate system)
    const double *arm_base_pos = getFromDef("Arm_Base")->getPosition();
    cv::Vec3f arm_base((float)arm_base_pos[0], (float) arm_base_pos[1], (float) arm_base_pos[2]);
    cv::Vec3f eef_pos_armcoord = ForwardKinematics((cv::Vec<float, 7>)sensordata.jointAngles);
    eef_pos_armcoord[0] = -eef_pos_armcoord[0];
    eef_pos_armcoord[1] = -eef_pos_armcoord[1];
    sensordata.eefPosition = (cv::Mat) (eef_pos_armcoord + arm_base);

    // head fixation (allocentric system) for now leave it like this... actually the camera is not in the rotation center...
    const double *head_pos_ar = getFromDef("Camera_Center")->getPosition();
    Vec3f head_pos(head_pos_ar[0], head_pos_ar[1], head_pos_ar[2]);
    float phi = -sensordata.headAngles.at<float>(1);
    float theta = sensordata.headAngles.at<float>(0)+M_PI;
    Vec3f head_direction(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    float fixation_distance;
    if (isinf(sensordata.distancePicture.at<float>(50,50)))
    {
        fixation_distance = (float) distance_camera->getMaxRange();
    }
    else
    {
        fixation_distance = (float) sensordata.distancePicture.at<float>(50,50);
    }
    sensordata.headFixation =  (cv::Mat) (head_pos + fixation_distance * head_direction);
}

void Caren::applyMotorCommands()
{
    ///////////////////////////////////////////// caren position command /////////////////////////////////////////////////////
    float z_target = cedardata.carenPosition.at<float>(0);
    double z_pos = getFromDef("Camera_Center")->getPosition()[2];
    double z_vel = (z_target-z_pos)/(abs(z_target-z_pos)) * 0.001 * (abs(z_target-z_pos)>=0.001);
    double z_pos_new = caren_node->getPosition()[2] + z_vel;
    double new_pos[3] = {0,0,z_pos_new};

    Field *translation = caren_node->getField("translation");
    translation->setSFVec3f((double*)new_pos);

    ////////////////////////////////////////////// arm commands //////////////////////////////////////////////////////////////
    const double *arm_base_pos = getFromDef("Arm_Base")->getPosition();
    cv::Vec3f arm_base((float)arm_base_pos[0], (float) arm_base_pos[1], (float) arm_base_pos[2]);

    cv::Vec3f p_target(cedardata.eefPosition.at<float>(0),cedardata.eefPosition.at<float>(1),cedardata.eefPosition.at<float>(2));
    p_target = p_target - arm_base;
    p_target[0] = -p_target[0];
    p_target[1] = -p_target[1];

    cv::Vec<float, 7> joint_min_positions;
    cv::Vec<float, 7> joint_max_positions;
    cv::Vec<float, 7> joint_max_velocities;
    cv::Vec<float,7> joint_velocities = ThetaDot(p_target, (cv::Vec<float, 7>)sensordata.jointAngles);
    cv::Vec<float,7> new_joint_angles = (cv::Vec<float, 7>)sensordata.jointAngles + joint_velocities * this->getBasicTimeStep();

    for (std::vector<webots::Motor*>::size_type i = 0; i < joint_motors.size(); i++)
    {
        joint_min_positions[i] = (float) joint_motors[i]->getMinPosition();
        joint_max_positions[i] = (float) joint_motors[i]->getMaxPosition();
        joint_max_velocities[i] = (float) joint_motors[i]->getMaxVelocity();
        new_joint_angles[i] = new_joint_angles[i] * (new_joint_angles[i]>joint_min_positions[i] && new_joint_angles[i]<joint_max_positions[i]) + (joint_min_positions[i]+0.001) * (new_joint_angles[i]<joint_min_positions[i]) + (joint_max_positions[i]-0.001) * (new_joint_angles[i]>joint_max_positions[i]);
        joint_velocities[i] = joint_velocities[i] * (abs(joint_velocities[i])<joint_max_velocities[i]) + (joint_max_velocities[i]-0.001) * (abs(joint_velocities[i])>joint_max_velocities[i]);
        joint_motors[i]->setVelocity(abs(joint_velocities[i]));
        joint_motors[i]->setPosition(new_joint_angles[i]);
    }

    ////////////////////////////////////////////// cube commands /////////////////////////////////////////////////////////////
    cube_motor->setVelocity(0.0);
    cube_motor->setPosition(0.0);

    ////////////////////////////////////////////// head commands /////////////////////////////////////////////////////////////
    const double *head_pos_ar = getFromDef("Camera_Center")->getPosition();
    Vec3f head_pos(head_pos_ar[0], head_pos_ar[1], head_pos_ar[2]);
    Vec3f target_fixation(cedardata.headFixation.at<float>(0), 0.8, cedardata.headFixation.at<float>(1));
    Vec3f target_head_direction = (target_fixation-head_pos)/norm(target_fixation-head_pos);
    target_head_direction[0] = -target_head_direction[0];
    float theta = acos(target_head_direction[1]);
    float phi = atan2(target_head_direction[2], target_head_direction[0]);
    head_motors[0]->setVelocity(0.5);
    head_motors[0]->setPosition(theta-M_PI);
    head_motors[1]->setVelocity(0.5);
    head_motors[1]->setPosition(-phi);

    ///////////////////////////////////////////// connector commands /////////////////////////////////////////////////////////
    if ( sensordata.connectorStatus.at<float>(0) != cedardata.connectorStatus.at<float>(0) && connector->isLocked())
    {
        connector->unlock();
    }
    else if (sensordata.connectorStatus.at<float>(0) != cedardata.connectorStatus.at<float>(0) && connector->isLocked() == false)
    {
        connector->lock();
    }
}

void Caren::initFromConfig()
{
    // the names come from the webots world file
    // get base Caren Node
    caren_node = getFromDef("CAREN");

    // get and enable joint motors and sensors
    std::cout << "Initialize the Arm!" << std::endl;
    joint_motors.push_back(getMotor("arm_motor_0"));
    joint_sensors.push_back(getMotor("arm_motor_0")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_1"));
    joint_sensors.push_back(getMotor("arm_motor_1")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_2"));
    joint_sensors.push_back(getMotor("arm_motor_2")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_3"));
    joint_sensors.push_back(getMotor("arm_motor_3")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_4"));
    joint_sensors.push_back(getMotor("arm_motor_4")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_5"));
    joint_sensors.push_back(getMotor("arm_motor_5")->getPositionSensor());
    joint_motors.push_back(getMotor("arm_motor_6"));
    joint_sensors.push_back(getMotor("arm_motor_6")->getPositionSensor());
    for (int i=0; i<7; i++)
    {
        joint_sensors[i]->enable(this->getBasicTimeStep());
    }

    // get and enable head motors and sensors
    std::cout << "Initialize the Caren Head!" << std::endl;
    head_motors.push_back(getMotor("pan_camera_motor"));
    head_sensors.push_back(getMotor("pan_camera_motor")->getPositionSensor());
    head_motors.push_back(getMotor("rotational_camera_motor"));
    head_sensors.push_back(getMotor("rotational_camera_motor")->getPositionSensor());
    for (int i=0; i<2; i++)
    {
        head_sensors[i]->enable(this->getBasicTimeStep());
    }

    // get and enable cube motor and sensor
    std::cout << "Initialize the Caren Power Cube!" << std::endl;
    cube_motor = getMotor("cube_motor");
    cube_sensor = getMotor("cube_motor")->getPositionSensor();
    cube_sensor -> enable(this->getBasicTimeStep());

    // get and enable camera
    std::cout << "Initialize the Camera" << std::endl;
    camera = getCamera("camera_center");
    distance_camera = getRangeFinder("range-finder");
    camera->enable(this->getBasicTimeStep());
    distance_camera->enable(this->getBasicTimeStep());

    // get and enable connector
    std::cout << "Initialize the Caren Connector!" << std::endl;
    connector = getConnector("connector");
    connector->enablePresence(this->getBasicTimeStep());
    
    // get and enable touch sensor
    std::cout << "Initialize the Touch Sensor!" << std::endl;
    touch_sensor = getTouchSensor("touch sensor");
    touch_sensor->enable(this->getBasicTimeStep());

    // add write sockets
    if (configMap.find("arm_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("arm", std::stoi(configMap["arm_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("connector_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("connector", std::stoi(configMap["connector_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("cube_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("cube", std::stoi(configMap["cube_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("head_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("head", std::stoi(configMap["head_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("camera_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("camera", std::stoi(configMap["camera_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("caren_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("caren", std::stoi(configMap["caren_port_snd"]), configMap["cedar_ip"]);
    }

    // add read sockets
    if (configMap.find("arm_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("arm", std::stoi(configMap["arm_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
    }
    if (configMap.find("connector_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("connector", std::stoi(configMap["connector_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
    }
    if (configMap.find("cube_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("cube", std::stoi(configMap["cube_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
    }
    if (configMap.find("head_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("head", std::stoi(configMap["head_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
    }
    if (configMap.find("caren_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("caren", std::stoi(configMap["caren_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
    }
}

std::map<std::string, std::string> Caren::readConfiguration(std::string configFilePath)
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
                } else
                {
                    std::cout << "Your Config File seems to be faulty. Each line should look like this >>identifier:value<< . Faulty line: " << line
                                << std::endl;
                }
            }
        }
    } else
    {
        std::cout << "ERROR! Could not open config file: " << configFilePath << std::endl;
    }
    config_file.close();
    return cMap;
}
