class E_Puck : public Robot
{
  // IP and Buffersize load from Config
  std::string caren_ip_address;
  int read_buffer_size;
  // Ports loaded from the Config
  int camera_port_snd;
  int mic_port_snd;
  int motor_port_rcv;
  int break_port_rcv;
  int LED_port_rcv;

  // devices and ComThread
  std::map<std::string, std::string> configMap;
  std::vector<webots::Motor*> motors;
  std::vector<webots::DistanceSensor*> sensors;
  std::vector<webots::LED*> LEDs;
  webots::Camera* cam;
  webots::Receiver* rec;
  webots::Emitter* em;
  std::unique_ptr<ComLooper> comThread;
  
  // Cedar and Sensor Structs
  struct SensorData
  {
      cv::Mat psMat;
      cv::Mat receiverMat;
      cv::Mat receiverTaskMat;
      cv::Mat cameraMat;
  };
  struct CedarData
  {
      cv::Mat motorMat;
      cv::Mat breakMat;
      cv::Mat LEDMat;
      cv::Mat goalMat;
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
  E_Puck(std::string configFilePath) 
  {
    std::cout << "Create E-Puck with ConfigFile: " << configFilePath << std::endl;
    // read config
    configMap = readConfiguration(configFilePath);
    //Create Communication Thread
    comThread = std::make_unique<ComLooper>();
    // init from config
    initFromConfig();
    // start comthread
    comThread->run();
    // init structs
    sensordata = SensorData{cv::Mat::zeros(8,1,CV_32F), cv::Mat::zeros(10,1,CV_32F), cv::Mat::zeros(8,1,CV_32F), cv::Mat::zeros(52, 39, CV_8UC4)};
    cedardata = CedarData{cv::Mat::zeros(1,1,CV_32F), cv::Mat::zeros(1,1,CV_32F), cv::Mat::zeros(8,1,CV_32F), cv::Mat::zeros(3,1,CV_32F)};
  };

  ~E_Puck()
  {
    if (comThread->isRunning())
    {
        comThread->stop();
        comThread = nullptr;
    }
  };

  void update();
};
