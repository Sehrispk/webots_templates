#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
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
#include "coordinate_transforms.h"

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
    sensordata = SensorData{cv::Mat(1,1,CV_32F), cv::Mat::zeros(8,1,CV_32F), cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(2,1,CV_32F), cv::Mat::zeros(100, 100, CV_8UC4), cv::Mat::zeros(100,100, CV_32F)};
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

    if (comThread->doesWriteSocketExist("distance_camera"))
    {
        comThread->setWriteMatrix("distance_camera", sensordata.distancePicture);
    }

    if (comThread->doesWriteSocketExist("arm"))
    {
        sensordata.eefPosition = caren_to_field_3d(sensordata.eefPosition);
        comThread->setWriteMatrix("arm", sensordata.eefPosition);
    }

    if (comThread->doesWriteSocketExist("connector"))
    {
        comThread->setWriteMatrix("connector", sensordata.connectorStatus);
    }

    if (comThread->doesWriteSocketExist("head_angles"))
    {
        cv::Mat angles_deg = cv::Mat::zeros(2,1,CV_32F); // Cedar wants degrees...
        angles_deg.at<float>(0) = sensordata.headAngles.at<float>(0)/M_PI*180;
        angles_deg.at<float>(1) = sensordata.headAngles.at<float>(1)/M_PI*180;
        comThread->setWriteMatrix("head_angles", angles_deg);
    }

    if (comThread->doesWriteSocketExist("head"))
    {
        sensordata.headFixation = caren_to_field_2d(sensordata.headFixation);
        comThread->setWriteMatrix("head", sensordata.headFixation);
    }

    // read from cedar
    if (comThread->doesReadSocketExist("arm"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("arm");
        if (commandMatrix.rows == 3)
        {
            cedardata.eefPosition = commandMatrix;
            cedardata.eefPosition = field_3d_to_caren(cedardata.eefPosition);
        }
        else
        {
            cedardata.eefPosition = cv::Mat::zeros(3,1,CV_32F);
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
            cedardata.connectorStatus = cv::Mat::zeros(1,1,CV_32F);
        }
    }

    if (comThread->doesReadSocketExist("head"))
    {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("head");
        if (commandMatrix.rows == 2)
        {
            cedardata.headFixation = commandMatrix;
            cedardata.headFixation = field_2d_to_caren(cedardata.headFixation);
        }
        else
        {
            cedardata.headFixation = cv::Mat::zeros(2,1,CV_32F);
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
    sensordata.jointAngles.at<float>(7) = cube_sensor->getValue();

    // caren position (caren coordinate system)
    sensordata.carenPosition.at<float>(0) = 0;

    // head angles
    for (std::vector<webots::PositionSensor*>::size_type i=0; i < head_sensors.size(); i++)
    {
        sensordata.headAngles.at<float>(i) = head_sensors[i]->getValue();
    }

    // connector status
    sensordata.connectorStatus.at<float>(0) = connector->isLocked();
    sensordata.connectorStatus.at<float>(1) = connector->getPresence();

    // endeffector position (caren coordinate system)
    Vec3f eef_pos_armcoord = ForwardKinematics((cv::Vec<float, 8>)sensordata.jointAngles);
    eef_pos_armcoord = arm_base_to_caren(eef_pos_armcoord);
    sensordata.eefPosition = (cv::Mat) (eef_pos_armcoord);

    // head fixation (caren coordinate system)
    Vec3f rotation_origin(0.,0.,0.);
    rotation_origin = cam_base_to_caren(rotation_origin);
    Vec3f cam_fixation = CameraAnglesToCenterViewPoint(rotation_origin, sensordata.headAngles.at<float>(1), sensordata.headAngles.at<float>(0));
    Vec2f table_pos(cam_fixation[0], cam_fixation[1]);
    sensordata.headFixation =  (cv::Mat) table_pos;
}

void Caren::applyMotorCommands()
{
    ////////////////////////////////////////////// arm commands //////////////////////////////////////////////////////////////
    cv::Vec3f p_target(cedardata.eefPosition.at<float>(0),cedardata.eefPosition.at<float>(1),cedardata.eefPosition.at<float>(2));
    p_target = caren_to_arm_base(p_target);

    float joint_min_position;
    float joint_max_position;
    float joint_max_velocity;
    cv::Vec<float,8> joint_velocities = ThetaDot(p_target, (cv::Vec<float, 8>)sensordata.jointAngles);
    cv::Vec<float,8> new_joint_angles = (cv::Vec<float, 8>)sensordata.jointAngles + joint_velocities * this->getBasicTimeStep();

    for (std::vector<webots::Motor*>::size_type i = 0; i < joint_motors.size(); i++)
    {
        joint_min_position = (float) joint_motors[i]->getMinPosition();
        joint_max_position = (float) joint_motors[i]->getMaxPosition();
        joint_max_velocity = (float) joint_motors[i]->getMaxVelocity();
        new_joint_angles[i] = new_joint_angles[i] * (new_joint_angles[i]>joint_min_position && new_joint_angles[i]<joint_max_position) + (joint_min_position+0.001) * (new_joint_angles[i]<joint_min_position) + (joint_max_position-0.001) * (new_joint_angles[i]>joint_max_position);
        joint_velocities[i] = joint_velocities[i] * (abs(joint_velocities[i])<joint_max_velocity) + (joint_max_velocity-0.001) * (abs(joint_velocities[i])>joint_max_velocity);
        joint_motors[i]->setVelocity(abs(joint_velocities[i]));
        joint_motors[i]->setPosition(new_joint_angles[i]);
    }
    joint_max_velocity = cube_motor->getMaxVelocity();
    cube_motor->setVelocity(abs(joint_velocities[7]) * (abs(joint_velocities[7])<joint_max_velocity) + (joint_max_velocity-0.001) * (abs(joint_velocities[7])>joint_max_velocity));
    cube_motor->setPosition(new_joint_angles[7]);

    ////////////////////////////////////////////// head commands /////////////////////////////////////////////////////////////
    Vec3f cam_base(0.,0.,0.);
    cam_base = cam_base_to_caren(cam_base);
    Vec3f target_fixation(cedardata.headFixation.at<float>(0), cedardata.headFixation.at<float>(1), coord_box[2][0]);
    Vec2f target_angles = CenterViewPointToCameraAngles(target_fixation, cam_base);
    
    head_motors[0]->setVelocity(1.0);
    head_motors[0]->setPosition(target_angles[1]-M_PI_2); // angles should be between 0 and -pi/2
    head_motors[1]->setVelocity(1.0);
    head_motors[1]->setPosition(-target_angles[0]); // - because angle of motor corresponds to -phi

    ///////////////////////////////////////////// connector commands /////////////////////////////////////////////////////////
    if ( cedardata.connectorStatus.at<float>(0) <= 0.5 &&  sensordata.connectorStatus.at<float>(0)==1)
    {
        std::cout << "unlock!" << std::endl;
        sensordata.connectorStatus.at<float>(0)=0;
        connector->unlock();
    }
    else if (cedardata.connectorStatus.at<float>(0) > 0.5 && sensordata.connectorStatus.at<float>(0)==0 && sensordata.connectorStatus.at<float>(1)==1)
    {
        std::cout << "lock!" << std::endl;
        sensordata.connectorStatus.at<float>(0)=1;
        connector->lock();
    }
}

void Caren::initFromConfig()
{
    // the names come from the webots world file
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

    // add write sockets
    if (configMap.find("arm_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("arm", std::stoi(configMap["arm_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("connector_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("connector", std::stoi(configMap["connector_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("head_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("head", std::stoi(configMap["head_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("head_angles_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("head_angles", std::stoi(configMap["head_angles_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("camera_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("camera", std::stoi(configMap["camera_port_snd"]), configMap["cedar_ip"]);
    }
    if (configMap.find("distance_port_snd") != configMap.end())
    {
        comThread->addWriteSocket("distance_camera", std::stoi(configMap["distance_port_snd"]), configMap["cedar_ip"]);
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
    if (configMap.find("head_port_rcv") != configMap.end())
    {
        comThread->addReadSocket("head", std::stoi(configMap["head_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
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
