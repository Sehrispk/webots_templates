#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include "tcp_socket_reader.h"
#include "tcp_socket_writer.h"

#define MATMSG_END "E-N-D!"

#define MATMSG_CHK "CHK-SM"

class ComLooper
{
public: // Methods
    ComLooper()
    {
         mRunning.store(false);
         mAbortRequested.store(false);
    }
    // Copy denied, Move to be implemented

    ~ComLooper()
    {
        abortAndJoin();
    }

    // To be called, once the looper should start looping.
    bool run()
    {
        try
        {
            mThread = std::thread(&ComLooper::runFunc, this);
        }
        catch (...)
        {
            return false;
        }

        return true;
    }

    void stop()
    {
        abortAndJoin();
    }


    bool isRunning() const
    {
        return mRunning.load();
    }

    void addReadSocket(std::string id,int port, int bufferLength)
    {
        if (!isRunning())
        {
            readSockets.push_back(std::shared_ptr<TCP_Socket_Reader>(new TCP_Socket_Reader(id, port, bufferLength)));
        } else
        {
            std::cout << "ComLooper::addSocket! Looper is already running. Socket was not added!" << std::endl;
        }
    }

    void addWriteSocket(std::string id,int port,std::string IPAddress)
    {
        if (!isRunning())
        {
            writeSockets.push_back(std::shared_ptr<TCP_Socket_Writer>(new TCP_Socket_Writer(id, port,IPAddress)));
        } else
        {
            std::cout << "ComLooper::addSocket! Looper is already running. Socket was not added!" << std::endl;
        }
    }

    cv::Mat getReadCommandMatrix(std::string id)
    {      
        auto socket = getReadSocketByID(id);
        
        if(socket!=nullptr)
          return socket->getMatrix();

        return cv::Mat();
    }

    void setWriteMatrix(std::string id,cv::Mat matrix)
    {
        auto socket = getWriteSocketByID(id);

        if(socket!=nullptr)
           socket->setMatrix(matrix);
    }
    
    bool isReadSocketConnected(std::string id)
    {
      auto socket = getReadSocketByID(id);
        
      if(socket!=nullptr)
          return socket->isConnected();

      return false;
    }

    bool isWriteSocketConnected(std::string id)
    {
        auto socket = getWriteSocketByID(id);

        if(socket!=nullptr)
            return socket->isConnected();

        return false;
    }

    bool doesReadSocketExist(std::string id)
    {
        auto socket = getReadSocketByID(id);

        if(socket!=nullptr)
            return true;
        else
            return false;
    }

    bool doesWriteSocketExist(std::string id)
    {
        auto socket = getWriteSocketByID(id);

        if(socket!=nullptr)
            return true;
        else
            return false;
    }
    
    
    std::shared_ptr<TCP_Socket_Reader> getReadSocketByID (std::string id)
    {
      auto it = std::find_if(readSockets.begin(), readSockets.end(),
                               [id](std::shared_ptr<TCP_Socket_Reader> socketWrapper) -> bool
                               {
                                   return (socketWrapper->getID() == id);
                               });

        if (it != readSockets.end())
        {
          return *it;
        }
        
        return nullptr;
    }

    std::shared_ptr<TCP_Socket_Writer> getWriteSocketByID (std::string id)
    {
        auto it = std::find_if(writeSockets.begin(), writeSockets.end(),
                               [id](std::shared_ptr<TCP_Socket_Writer> socketWrapper) -> bool
                               {
                                   return (socketWrapper->getID() == id);
                               });

        if (it != writeSockets.end())
        {
            return *it;
        }

        return nullptr;
    }

private: // Methods
    // Conditionally-infinite loop doing sth. iteratively
    void runFunc()
    {
        mRunning.store(true);
      
        while (false == mAbortRequested.load())
        {
            try
            {
                // Do something...
                for (unsigned int i = 0; i < readSockets.size(); i++)
                {
                    readSockets[i]->readNextMessage(MATMSG_END,MATMSG_CHK);
                }

                for (unsigned int i = 0; i < writeSockets.size(); i++)
                {
                    writeSockets[i]->writeMessage(MATMSG_END,MATMSG_CHK);
                }
            }
            catch (std::runtime_error &e)
            {
               std::cout<<"Caught an error: " << e.what() <<std::endl;
            }
            catch (...)
            {
                // Make sure that nothing leaves the thread for now...
                std::cout<<"Caught something else while running!" <<std::endl;
            }
        }

        mRunning.store(false);
    }


    void abortAndJoin()
    {
        mAbortRequested.store(true);
        if (mThread.joinable())
        {
            mThread.join();
        }
    }

private: //Members
    std::thread mThread;
    std::atomic_bool mRunning;
    std::atomic_bool mAbortRequested;
    std::vector<std::shared_ptr<TCP_Socket_Reader>> readSockets;
    std::vector<std::shared_ptr<TCP_Socket_Writer>> writeSockets;
};