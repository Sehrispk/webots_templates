#ifndef Caren_H
#define Caren_H

#include <opencv2/core/core.hpp>
#include <map>
class ComLooper;


class Caren : public Supervisor
{
    // IP and Buffersize and ports load from Config
    std::string caren_ip_address;
    int read_buffer_size;
    int arm_port_rcv;
    int arm_port_snd;
    int head_port_rcv;
    int head_port_snd;
    int cam_port_snd;
    int position_rcv;
    int position_snd;
    int connector_port_rcv;
    int connector_port_snd;

    // devices and ComThread
    std::map<std::string, std::string> configMap;
    webots::Node* caren_node;
    std::vector<webots::Motor*> joint_motors;
    std::vector<webots::PositionSensor*> joint_sensors;
    std::vector<webots::Motor*> head_motors;
    std::vector<webots::PositionSensor*> head_sensors;
    webots::Motor* cube_motor;
    webots::PositionSensor* cube_sensor;
    webots::Camera* camera;
    webots::RangeFinder* distance_camera;
    webots::Connector* connector;
    webots::TouchSensor* touch_sensor;
    std::unique_ptr<ComLooper> comThread;
    
    // Cedar and Sensor Structs
    struct SensorData
    {
      cv::Mat carenPosition;
      cv::Mat jointAngles;
      cv::Mat eefPosition;
      cv::Mat connectorStatus;
      cv::Mat cubePosition;
      cv::Mat headAngles;
      cv::Mat headFixation;
      cv::Mat cameraPicture;
      cv::Mat distancePicture;
    };
    struct CedarData
    {
      cv::Mat carenPosition;
      cv::Mat eefPosition;
      cv::Mat connectorStatus;
      cv::Mat headFixation;
    };
  
    SensorData sensordata;
    CedarData cedardata;

private:
  void initFromConfig();
  std::map<std::string, std::string> readConfiguration(std::string configFilePath);
  void ComThreadUpdate();
  void readSensorValues();
  void applyMotorCommands();

public:
  Caren(std::string configFilePath);

  ~Caren();

  void update();
};

#endif
