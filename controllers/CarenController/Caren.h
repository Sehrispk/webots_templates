#ifndef Caren_H
#define Caren_H

//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <map>
class ComLooper;

using cv::Vec2f;
using cv::Vec3f;

class Caren : public Robot
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
    std::vector<webots::Motor*> joint_motors;
    std::vector<webots::PositionSensor*> joint_sensors;
    std::vector<webots::Motor*> head_motors;
    std::vector<webots::PositionSensor*> head_sensors;
    webots::Motor* cube_motor;
    webots::PositionSensor* cube_sensor;
    webots::Camera* camera;
    webots::RangeFinder* distance_camera;
    webots::Connector* connector;
    std::unique_ptr<ComLooper> comThread;
    
    // Cedar and Sensor Structs
    struct SensorData
    {
      cv::Mat carenPosition;
      cv::Mat jointAngles;
      cv::Mat eefPosition;
      cv::Mat connectorStatus;
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

    // saccade status
    int saccade_status=0;
    float distanceBuffer = 1;
    float defaultDistance = 0.6;
    Vec2f fixationOffset;

    // default arm position
    Vec3f defaultArmPos;
    Vec2f defaulCamAngl;

    // arm target fixed buffer
    Vec3f P_target;
    Vec3f P_initial;
    Vec3f delta_P;
    float t=0, delta_t=0.001;
    float x = 0.25;
    float arm_operation_mode = 1;
    float epsilon = 0.001;
    float safety_height = -0.1;
    float move_to_safety = 0;

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
