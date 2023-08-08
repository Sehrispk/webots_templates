// File:          tcp_caren_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Connector.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp> 
#include <webots/Receiver.hpp> 
#include <opencv2/core/core.hpp>
#include <fstream>
#include <map>
#include <cmath>

#include<chrono>
#include<thread>
#include "utilities.h"
#include "kinematics.h"
#include "tcp_communication_looper.h"


#define TIMESTEP 8
#define MATMSG_END "E-N-D!"

// All the webots classes are defined in the "webots" namespace
using namespace webots;


class TCP_Caren_Controller : public Supervisor

{
    // IP and Buffersize load from Config
    std::string caren_ip_address;
    int read_buffer_size;
    // Ports loaded from the Config
    int arm_port_rcv;
    int arm_port_snd;
    int head_port_rcv;
    int head_port_snd;
    int cam_port_snd;
    int position_rcv;
    int position_snd;
    int connector_port_rcv;
    int connector_port_snd;


    webots::Node* lastTransportedNode;

    std::map<std::string, std::string> configMap;
    std::map<std::string, std::vector<webots::Motor*>> motorMap;
    std::map<std::string, webots::Camera*> cameraMap;
    std::vector<webots::PositionSensor*> arm_sensors;
    std::unique_ptr<ComLooper> comThread;
    
    std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
    webots::Connector* connector;
    webots::TouchSensor* touch_sensor;
    webots::Receiver* receiver;

private:
    //****************************************************************************
    // This function has to be tailored to YOUR configFile !!!!!!!!!!!!!!!!!!!!!!!
    //****************************************************************************
    void initFromConfig()
    {
      if (configMap.find("arm_port_snd") != configMap.end() || configMap.find("arm_port_rcv") != configMap.end())
      {
        std::cout << "Initialize the Arm!" << std::endl;

        //The motor names come from the webots world file
        std::vector<webots::Motor*> arm_motors;
        arm_motors.push_back(getMotor("arm_motor_0"));
        arm_sensors.push_back(getMotor("arm_motor_0")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_1"));
        arm_sensors.push_back(getMotor("arm_motor_1")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_2"));
        arm_sensors.push_back(getMotor("arm_motor_2")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_3"));
        arm_sensors.push_back(getMotor("arm_motor_3")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_4"));
        arm_sensors.push_back(getMotor("arm_motor_4")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_5"));
        arm_sensors.push_back(getMotor("arm_motor_5")->getPositionSensor());
        arm_motors.push_back(getMotor("arm_motor_6"));
        arm_sensors.push_back(getMotor("arm_motor_6")->getPositionSensor());

        for (int i=0; i<7; i++)
        {
          arm_sensors[i]->enable(TIMESTEP);
        }

        //You define this identifier by yourself, but there cannot be duplicates!
        std::string armIdentifier = "arm";
        motorMap[armIdentifier] = arm_motors;
        
        if (configMap.find("arm_port_snd") != configMap.end())
        {
            comThread->addWriteSocket(armIdentifier, std::stoi(configMap["arm_port_snd"]), configMap["cedar_ip"]);
        }

        if (configMap.find("arm_port_rcv") != configMap.end())
        {
            comThread->addReadSocket(armIdentifier, std::stoi(configMap["arm_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
        }
      }

      if (configMap.find("head_port_snd") != configMap.end() || configMap.find("head_port_rcv") != configMap.end())
      {
        //The motor names come from the webots world file
        std::cout << "Initialize the Caren Head!" << std::endl;
        std::vector<webots::Motor*> head_motors;
        head_motors.push_back(getMotor("pan_camera_motor"));
        head_motors.push_back(getMotor("rotational_camera_motor"));

        //You define this identifier by yourself, but there cannot be duplicates!
        std::string headIdentifier = "head";
        motorMap[headIdentifier] = head_motors;

        if (configMap.find("head_port_snd") != configMap.end())
        {
            comThread->addWriteSocket(headIdentifier, std::stoi(configMap["head_port_snd"]), configMap["cedar_ip"]);
        }

        if (configMap.find("head_port_rcv") != configMap.end())
        {
            comThread->addReadSocket(headIdentifier, std::stoi(configMap["head_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
        }
      }

      if (configMap.find("caren_position_snd") != configMap.end() || configMap.find("caren_position_rcv") != configMap.end())
      {
        std::cout << "Initialize the Caren Power Cube!" << std::endl;
        std::vector<webots::Motor*> cube_motors;
        cube_motors.push_back(getMotor("cube_motor"));

        std::string trunkIdentifier = "trunk";
        motorMap[trunkIdentifier] = cube_motors;

        if (configMap.find("caren_position_snd") != configMap.end())
        {
            comThread->addWriteSocket("caren position", std::stoi(configMap["caren_position_snd"]), configMap["cedar_ip"]);
        }
        if (configMap.find("caren_position_rcv") != configMap.end())
        {
            comThread->addReadSocket("caren position", std::stoi(configMap["caren_position_rcv"]), std::stoi(configMap["read_buffer_size"]));
        }
      }

      if (configMap.find("camera_center_port_snd") != configMap.end() )
      {
        std::cout << "Initialize the Center Camera" << std::endl;
        //You define this identifier by yourself, but there cannot be duplicates!
        std::string cameraIdentifier = "camera_center";
        //The device names come from the webots world file
        cameraMap[cameraIdentifier] = getCamera("camera_center");
        if (configMap.find("camera_center_port_snd") != configMap.end())
        {
            comThread->addWriteSocket(cameraIdentifier, std::stoi(configMap["camera_center_port_snd"]), configMap["cedar_ip"]);
        }
      }

      if (configMap.find("feature_map_snd") != configMap.end())
      {
        comThread->addWriteSocket("feature maps", std::stoi(configMap["feature_map_snd"]), configMap["cedar_ip"]);
      }

      if (configMap.find("connector_port_snd") != configMap.end() || configMap.find("connector_port_rcv") != configMap.end())
        {
          std::cout << "Initialize the Caren Connector!" << std::endl;
          std::string connectorIdentifier = "connector";
          connector = getConnector("connector");
          if (configMap.find("connector_port_rcv") != configMap.end())
          {
              comThread->addReadSocket(connectorIdentifier, std::stoi(configMap["connector_port_rcv"]), std::stoi(configMap["read_buffer_size"]));
          }
          if (configMap.find("connector_port_snd") != configMap.end())
          {
              comThread->addWriteSocket(connectorIdentifier, std::stoi(configMap["connector_port_snd"]), configMap["cedar_ip"]);
          }
        }
    }

  void sendCurrentMotorStatus()
    {
        for (auto const&[identifier, motorVector] : motorMap)
        {
            if (comThread->doesWriteSocketExist(identifier))
            {
                cv::Mat motorValues = cv::Mat::zeros(motorVector.size(), 1, CV_32F);
                for (std::vector<webots::Motor*>::size_type i = 0; i < motorVector.size(); i++)
                {
                    motorValues.at<float>(i, 0) = motorVector[i]->getTargetPosition();
                }

                comThread->setWriteMatrix(identifier,motorValues);
            }
        }
    }
    
  void sendCurrentTouchSensorStatus()
    {
          // currently does not work because bounding box of touch 
          //sensor and connector/caren arm overlap and cause trouble!
        
          if (comThread->doesWriteSocketExist("touch sensor"))
          {
                
            double TouchSensorStatus = touch_sensor->getValue();
            std::cout << TouchSensorStatus << std::endl;
            cv::Mat TouchSensorMatrix(1,1,CV_32F, TouchSensorStatus);
       
            comThread->setWriteMatrix("touch sensor", TouchSensorMatrix);
            
          }
      
    }

  void readAndApplyMotorCommands()
  {
    for (auto const&[identifier, motorVector] : motorMap)
    {
      if (comThread->doesReadSocketExist(identifier))
      {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix(identifier);

        Node *CarenNode = getFromDef("CAREN");
        const double *CarenPosition = CarenNode->getPosition();

        const double *arm_base_ar = getFromDef("Arm_Base")->getPosition();
        cv::Vec3f arm_base((float)arm_base_ar[0], (float) arm_base_ar[1], (float) arm_base_ar[2]);

        cv::Vec3f p_target(-commandMatrix.at<float>(0)/100.+0.2,commandMatrix.at<float>(2)*0.33/60.+0.7,commandMatrix.at<float>(1)/100.-0.9+(float) CarenPosition[2]);
        p_target = p_target - arm_base;
        p_target[0] = -p_target[0];
        p_target[1] = -p_target[1];

        cv::Vec<float, 7> joint_angles;
        cv::Vec<float, 7> joint_min_positions;
        cv::Vec<float, 7> joint_max_positions;
        cv::Vec<float, 7> joint_max_velocities;
        for (int i = 0; i < 7; i++)
        {
          joint_angles[i] = (float) arm_sensors[i]->getValue();
          joint_min_positions[i] = (float) motorVector[i]->getMinPosition();
          joint_max_positions[i] = (float) motorVector[i]->getMaxPosition();
          joint_max_velocities[i] = (float) motorVector[i]->getMaxVelocity();
        }

        cv::Vec<float,7> joint_velocities = ThetaDot(p_target, joint_angles);
        cv::Vec<float,7> new_joint_angles = joint_angles + joint_velocities * TIMESTEP;

        for (std::vector<webots::Motor*>::size_type i = 0; i < motorVector.size(); i++)
        {
          new_joint_angles[i] = new_joint_angles[i] * (new_joint_angles[i]>joint_min_positions[i] && new_joint_angles[i]<joint_max_positions[i]) + (joint_min_positions[i]+0.001) * (new_joint_angles[i]<joint_min_positions[i]) + (joint_max_positions[i]-0.001) * (new_joint_angles[i]>joint_max_positions[i]);
          joint_velocities[i] = joint_velocities[i] * (abs(joint_velocities[i])<joint_max_velocities[i]) + (joint_max_velocities[i]-0.001) * (abs(joint_velocities[i])>joint_max_velocities[i]);
          motorVector[i]->setVelocity(abs(joint_velocities[i]));
          motorVector[i]->setPosition(new_joint_angles[i]);
        }
      }
    }
  }
    
  void readAndApplyPositionCommand()
    {
      // position along the table    
      if (comThread->doesReadSocketExist("caren position"))
      {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("caren position");
        float z_vel = commandMatrix.at<float>(0,0);
        Node *CarenNode = getFromDef("CAREN");
        double z_pos = CarenNode->getPosition()[2];
        double z_pos_new = z_pos + z_vel* 0.0002;
        //std::cout << z_vel << std::endl;

        if(z_pos >= -0.05 && z_pos < 1.25 )
        {
          Field *translation = CarenNode->getField("translation");
          double new_pos[3] = {0,0,z_pos_new};
          translation->setSFVec3f(new_pos);
        }
      }
    }
    
  void readAndApplyConnectorCommands()
    {
      if (comThread->doesReadSocketExist("connector")) 
      {
        cv::Mat commandMatrix = comThread->getReadCommandMatrix("connector");
        
        int num_grasp_objects = 4;
        //list of graspable objects -> passive connector in robot node
        const char *objects[num_grasp_objects] = { "Tonies_Figure_1", "Tonies_Figure_2", "Tonies_Figure_6", "Brush_Object" };
        //Node *lastTransportedNode = getFromDef("Brush_Object");
        
        int lock = (int) roundf(commandMatrix.at<float>(0,0));
        int transport = 0; //flag to indicate that an object is being transported
          
        if(lock == 1)
        {
          //get current robot position: was 0.08
          Node *ConnectorNode = getFromDef("Connector");
          const double *ConnectorPosition = ConnectorNode->getPosition();
          // get current brush position:
          Node *brushNode = getFromDef("Brush_Object");
          const double *BrushPosition = brushNode->getPosition();
          
          double brush_distance = std::sqrt( (BrushPosition[0]-ConnectorPosition[0])*(BrushPosition[0]-ConnectorPosition[0]) + 
                              (BrushPosition[1]-ConnectorPosition[1])*(BrushPosition[1]-ConnectorPosition[1]) + 
                              (BrushPosition[2]-ConnectorPosition[2])*(BrushPosition[2]-ConnectorPosition[2]));
          
          if (brush_distance < 0.065) //was 0.08
          {
            transport = 1;
            Field *BrushTranslation = brushNode->getField("translation");
            double newObjpos[3] = {ConnectorPosition[0],ConnectorPosition[1]-0.045,ConnectorPosition[2]};
            BrushTranslation->setSFVec3f(newObjpos);
            lastTransportedNode = brushNode;
          }
          
          for(int i = 0; i < num_grasp_objects; i++)
          {
            Node *ObjectNode = getFromDef(objects[i]); 
            const double *ObjectPosition = ObjectNode->getPosition();
            Field *ObjectTranslation = ObjectNode->getField("translation");
            double distance = std::sqrt( (ObjectPosition[0]-ConnectorPosition[0])*(ObjectPosition[0]-ConnectorPosition[0]) + 
                              (ObjectPosition[1]-ConnectorPosition[1])*(ObjectPosition[1]-ConnectorPosition[1]) + 
                              (ObjectPosition[2]-ConnectorPosition[2])*(ObjectPosition[2]-ConnectorPosition[2]));
            if(distance < 0.065 && transport == 0) //was 0.08
            {
              transport = 1;
              //Field* name = ObjectNode->getField("name"); //das war auskommentiert
              double newObjpos[3] = {ConnectorPosition[0],ConnectorPosition[1]-0.045,ConnectorPosition[2]};
              ObjectTranslation->setSFVec3f(newObjpos);
              lastTransportedNode = ObjectNode;   // lastTransportedNode = &getFromDef("Tonies_Figure_2");          
            }
          }
          
        }
        if(lock == 0 && transport == 1)
        {
          transport = 0;
        }
      }
    }
     
  void sendPerceptionMaps()
    {
      auto t_start_send = std::chrono::high_resolution_clock::now();
      const char *objects[13] = { "Tonies_Figure_1", "Tonies_Figure_2", 
                         "Tonies_Figure_6", "Brush_Object", "Paint_Bucket_blue", "Paint_Bucket_red", "Box", "Tonies_Box", "Tonies_Box_2"};
      cv::Mat FeatureMaps;
      cv::Mat SalienceMap = cv::Mat::zeros(60,60, CV_8UC1);
      cv::Mat ObjectColor = cv::Mat::zeros(60,60, CV_8UC1);
      cv::Mat ObjectShape = cv::Mat::zeros(60,60, CV_8UC1);
      cv::Mat ObjectSize = cv::Mat::zeros(60,60, CV_8UC1);
      cv::Mat ObjectPosition3D = cv::Mat::zeros(60,60, CV_8UC1);
      std::vector<cv::Mat> Channels;
      
      cv::Mat ConnObjectMat;
      cv::Mat ConnObject = cv::Mat::zeros(1, 1, CV_8UC1);
      cv::Mat ConnObjectColor = cv::Mat::zeros(1, 1, CV_8UC1);
      cv::Mat ConnObjectShape = cv::Mat::zeros(1, 1, CV_8UC1);
      cv::Mat ConnObjectHeight = cv::Mat::zeros(1, 1, CV_8UC1);
      std::vector<cv::Mat> ConnChannels;

      for(int i = 0; i < 9; i++) 
      {
        //without ToniesBox for now!!!! 
        //for some reason it works for all other objects but the ToniesBox (and its doubles). Should work
        //with the webots version in the lab... last time did not work with the brush, too, but brush works 
        //now perfectly fine...
        Node *ObjectNode = getFromDef(objects[i]); 
        const double *position = ObjectNode->getPosition();
        
        //get current robot position: 
        Node *CarenNode = getFromDef("CAREN");
        const double *CarenPosition = CarenNode->getPosition();
        double z_robot = CarenPosition[2] -0.6 ; //caren position in table space
        
        //check if x and z are within "camera view"
        if(position[0] <= 0.2 && position[0] > -0.4 && position[2] < (z_robot+0.3) && position[2] >= (z_robot-0.3))
        {
          //convert x,z to values between 0 and 60
          //convert y pos to values between 0 and 20
          //int object_id = i + 1.0;
          int x_pos = (int) (-(position[0]-0.2) * 100 );
          int z_pos = (int) ((position[2] - z_robot + 0.3) * 100 );
          int y_pos = (int) ((position[1]-0.7)*60/0.33);
          
          int NodeColor = getNodeColorHue(ObjectNode)/10;
          int NodeShape = getShapeIndex(ObjectNode);
          int NodeSize = getSizeIndex(ObjectNode);

          //std::cout << 60-x_pos << " " << 60-z_pos << " " << y_pos <<std::endl;
          //std::cout << y_pos << std::endl;
          SalienceMap.at<unsigned char>(60-x_pos,60-z_pos) = 1;
          ObjectPosition3D.at<unsigned char>(60-x_pos,60-z_pos) = y_pos; //3DPosition
          ObjectColor.at<unsigned char>(60-x_pos,60-z_pos) = NodeColor+1; //color only
          ObjectShape.at<unsigned char>(60-x_pos,60-z_pos) = NodeShape; //shape only
          ObjectSize.at<unsigned char>(60-x_pos,60-z_pos) = NodeSize; //height only
        }
      }      
      if (comThread->doesWriteSocketExist("feature maps"))
      {
        //Channels.push_back(SalienceMap);
        Channels.push_back(ObjectPosition3D);
        Channels.push_back(ObjectColor);
        Channels.push_back(ObjectShape);
        Channels.push_back(ObjectSize);
        cv::merge(Channels, FeatureMaps);
        comThread->setWriteMatrix("feature maps", FeatureMaps);
      }
      auto t_end_send = std::chrono::high_resolution_clock::now();

      auto ms_last_call = std::chrono::duration_cast<std::chrono::milliseconds>(t_start_send - t_start);
      auto ms_snd = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_send - t_start_send);
      //std::cout << "time to call: " << ms_last_call.count() << std::endl;
      //std::cout << "time to send: " << ms_snd.count() << std::endl;
      t_start = std::chrono::high_resolution_clock::now();
    }
    
  void sendRobotPosition()
  {
    if (comThread->doesWriteSocketExist("caren position"))
    {
      Node *CarenNode = getFromDef("CAREN");
      const double *CarenPosition = CarenNode->getPosition();
        
      double z_pos = CarenPosition[2];
      //std::cout << z_pos << std::endl;
      cv::Mat PositionMatrix(1,1,CV_32F, z_pos*(180.0)/1.2);
  
      comThread->setWriteMatrix("caren position", PositionMatrix);
    }
    if (comThread->doesWriteSocketExist("arm"))
    {
      Node *CarenNode = getFromDef("CAREN");
      const double *CarenPosition = CarenNode->getPosition();
        
      double z_robot = CarenPosition[2]-0.6;

      Node *ConnectorNode = getFromDevice(connector);
      const double *ConnectorPosition = ConnectorNode->getPosition();
      int x_pos = (int) (-(ConnectorPosition[0]-0.2) * 100 );
      int z_pos = (int) ((ConnectorPosition[2] - z_robot + 0.3) * 100 );
      int y_pos = (int) ((ConnectorPosition[1]-0.7)*60/0.33);
      
      cv::Mat ConnectorMap = cv::Mat::zeros(60,60, CV_32F);
      //std::cout << x_pos << " " << z_pos << " " << y_pos << std::endl;
      ConnectorMap.at<float>(60-x_pos,60-z_pos) = (float) y_pos;
      comThread->setWriteMatrix("arm", ConnectorMap);
    }
  }
    
  Node *getMaterialNode(Node *curNode)
    {
      Field *children = curNode->getField("children");
      if (children)
      {
        for (int i = 0; i < children->getCount(); i++)
        {
          //std::cout<< children->getCount() << std::endl;
          Node *currentChild = children->getMFNode(i);
          std::string typeName = currentChild->getTypeName();
          if (typeName == "Group")
          { 
            Field *children = currentChild->getField("children");
            if (children)
            {
              for (int i = 0; i < children->getCount(); i++)
              {
                Node *currentChild = children->getMFNode(i);
                std::string typeName = currentChild->getTypeName();
                if (typeName == "Shape")
                { 
                    
                  Field *appearance = currentChild->getField("appearance");
                  if (appearance)
                  {
                    
                    Node *appearanceNode = appearance->getSFNode();
                    Field *material = appearanceNode->getField("material");
                    if (material)
                    {
                      
                      Node *materialNode = material->getSFNode();
                      return materialNode;
                    }
                  }
                }
              }
            }
          }
          if (typeName == "Shape")
          { 
              
            Field *appearance = currentChild->getField("appearance");
            if (appearance)
            {
              
              Node *appearanceNode = appearance->getSFNode();
              Field *material = appearanceNode->getField("material");
              if (material)
              {
                
                Node *materialNode = material->getSFNode();
                return materialNode;
              }
            }
          }
        }
      }
      return NULL;
    }
    
  int getNodeColorHue(Node *curNode)
    {
      Node *materialNode = getMaterialNode(curNode);
      Field *diffuseColor = materialNode->getField("diffuseColor");
      if (diffuseColor)
      {
        const double *color = diffuseColor->getSFColor();
        int hue = rgbToHue(color);
        return hue;
      }
      return -1;
    }
    
  int getShapeIndex(Node *curNode)
    {
      Field *children = curNode->getField("children");
      if (children)
      {
        for (int i = 0; i < children->getCount(); i++)
        {
          //std::cout<< children->getCount() << std::endl;
          Node *currentChild = children->getMFNode(i);
          std::string DefName = currentChild->getDef();
          if (DefName == "Cross_Shape")
          { 
            return 9;
          }
          else if (DefName == "Flower_Shape")
          { 
            return 7;
          }
          else if (DefName == "Square")
          {
            return 1;
          }
          else if (DefName == "Tonie_Box_Group")
          {
            return 1; //Box and square figures have same shape but different size
          }
          else if (DefName == "PowerButton")
          {
            return 5;
          }
          else if (DefName == "Brush")
          {
            return 3;
          }
          else if (DefName == "Box_Shape") //Brush and Box have same shape but different size
          {
            return 3;
          }
          else if (DefName == "Bucket") //Bucket. Box and square figures have same shape but different size
          {
            return 1;
          }
        }
      return -1;
      }
    }
    
  int getSizeIndex(Node *curNode)
    {
      Field *children = curNode->getField("children");
      if (children)
      {
        for (int i = 0; i < children->getCount(); i++)
        {
          //std::cout<< children->getCount() << std::endl;
          Node *currentChild = children->getMFNode(i);
          std::string DefName = currentChild->getDef();
          if (DefName == "Cross_Shape")
          { 
            return 1;
          }
          else if (DefName == "Flower_Shape")
          { 
            return 1;
          }
          else if (DefName == "Square")
          {
            return 1;
          }
          else if (DefName == "Tonie_Box_Group")
          {
            return 5; //Box and square figures have same shape but different size
          }
          else if (DefName == "PowerButton")
          {
            return 3;
          }
          else if (DefName == "Brush")
          {
            return 1;
          }
          else if (DefName == "Box_Shape") //Brush and Box have same shape but different size
          {
            return 5;
          }
          else if (DefName == "Bucket") //Bucket. Box and square figures have same shape but different size
          {
            return 3;
          }
        }
      return -1;
      }
    }
    
  int rgbToHue(const double *rgbValues)
  {
    double r = rgbValues[0];
    double g = rgbValues[1];
    double b = rgbValues[2];

    double cMax = std::max(
            {r, g, b});
    double cMin = std::min(
            {r, g, b});
    double delta = cMax - cMin;

    if (delta == 0)
      return 0;

    if (cMax == r)
    {
      double factor1 = ((g - b) / delta);
      //  int fac6 = (int)factor1 + 6;
      factor1 = factor1 + 6;
      double dResult = fmod(factor1, 6) * 60;
      int result = (int) dResult;
      return result;
    } else if (cMax == g)
    {
      return (int) (((b - r) / delta) + 2) * 60;
    } else if (cMax == b)
    {
      return (int) (((r - g) / delta) + 4) * 60;
    } else
    {
      std::cout << "Something went wrong during Color Conversion" << std::endl;
      return -1;
    }
  }

  cv::Mat getCameraPicture(webots::Camera* cam)
  {
      int from_to[] =
              {0, 0, 1, 1, 2, 2}; // for mat conversion

      cv::Mat pictureMat(cam->getHeight(), cam->getWidth(), CV_8UC4,
                          const_cast<unsigned char *>(cam->getImage()));
      cv::Mat mat2 = cv::Mat(cam->getHeight(), cam->getWidth(), CV_8UC3);
      cv::mixChannels(&pictureMat, 1, &mat2, 1, from_to, 3); // kill the alpha channel

      return mat2;
  }

  std::map<std::string, std::string> readConfiguration(std::string configFilePath)
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


//Public functions
public:
    TCP_Caren_Controller(std::string
                         configFilePath) : Supervisor()
    {
        std::cout << "Create TCP_Caren_Controller with ConfigFile: " << configFilePath << std::endl;

        configMap = readConfiguration(configFilePath);
        //Create the Communication Thread
        comThread = std::make_unique<ComLooper>();

        auto camCenter = getCamera("camera_center");
        int camTimeStep = 4 * (int) this->getBasicTimeStep();
        camCenter->enable(camTimeStep);
        //int stepSize = camTimeStep;
        connector = getConnector("connector");
        connector->enablePresence(camTimeStep);
        touch_sensor = getTouchSensor("touch sensor");
        touch_sensor->enable(camTimeStep);
        
        receiver = getReceiver("receiver");
        receiver->enable(camTimeStep);
    }

    ~TCP_Caren_Controller()
    {
        if (comThread->isRunning())
        {
            comThread->stop();
            comThread = nullptr;
        }
    }

    void run()
    {
        std::cout << "RUN! >CAREN TCP Controller< RUN!" << std::endl;

        initFromConfig();

        int timeStep = (int) this->getBasicTimeStep();

        comThread->run();
        
   

        while (this->step(timeStep) != -1)
        {
           
          readAndApplyMotorCommands();
          
          readAndApplyConnectorCommands();
          
          readAndApplyPositionCommand();
          
          sendPerceptionMaps();
          
          sendRobotPosition();
            
        }

        comThread->stop();
        comThread = nullptr;

    }

}; //End of Class