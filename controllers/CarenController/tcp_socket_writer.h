#include <vector>
#include <opencv2/core/core.hpp>
#include <shared_mutex>
#include <chrono>
#include "utilities.h"

#ifdef _WIN32

#include <winsock2.h>
#include <Ws2tcpip.h>

#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#include <fcntl.h>

class TCP_Socket_Writer
{
public:

    TCP_Socket_Writer(std::string id, int port, std::string IPAddress) :
            mID(id),
            mPort(port),
            mIpAddress(IPAddress),
            currentWriteMatrix(cv::Mat::zeros(42, 42, CV_32F)),
            startMeasurement(std::chrono::steady_clock::now()),
            reconnectTimeOut(100),
            wasCleaned(true)
    {
        mIsConnected.store(false);
        establishConnection();
    }

    void establishConnection()
    {
        socket_handle = 0;

#ifdef _WIN32
        /* initialize the socket api */
        WSADATA info;
        int rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
        if (rc != 0)
        {
            printf("cannot initialize Winsock\n");
            return;
        }
#endif

        if ((socket_handle = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
#ifdef _WIN32
            std::cout << "Socket creation error. Handle is: " << socket_handle << " and WSAGetLastError is: " << WSAGetLastError() << std::endl;
#else
            std::cout << "Socket creation error. Handle is: " << socket_handle << " and errno is: "<< errno << std::endl;
#endif
            return;
        } else
        {


#ifdef _WIN32
            const char opt = 1;
            if (setsockopt(socket_handle, SOL_SOCKET, SO_REUSEADDR,
                           &opt, sizeof(opt)))
            {
                std::cout << "Error Setting Socket Options" << std::endl;
            }

            unsigned long on = 1;
            ioctlsocket(socket_handle, FIONBIO, &on); //make socket Non-blockable in windows
#else
            int opt = 1;
            if (setsockopt(socket_handle, SOL_SOCKET, SO_REUSEPORT,
                   &opt, sizeof(opt)))
            {
                std::cout << "Error Setting Socket Options" << std::endl;
            }
            fcntl(socket_handle, F_SETFL, O_NONBLOCK); //make socket Non-blockable
#endif
//            std::cout << "TCPWriter::establishConnection Socket created!" << std::endl;
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(mPort);

            if (inet_pton(AF_INET, mIpAddress.c_str(), &serv_addr.sin_addr) <= 0)
            {
                std::cout << "Invalid address/ Address not supported" << std::endl;
            } else
            {
               ::connect(socket_handle, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
            }
        }
    }


    ~TCP_Socket_Writer()
    {
        closeSocket();
    }

    void closeSocket()
    {
#ifdef _WIN32
        closesocket(socket_handle);
        WSACleanup();
#else
        close(socket_handle);
#endif

        mIsConnected.store(false);
    }

    void reconnect()
    {
        closeSocket();
        establishConnection();
    }

    void writeMessage(std::string messageEndSequence, std::string checkSumIndicator)
    {
        std::unique_lock<std::shared_timed_mutex> lck(mutex);
        cv::Mat matrixToSend = currentWriteMatrix.clone();
        lck.unlock();

        std::ostringstream stream;
        serialize(stream, matrixToSend);

        unsigned int checkSum = generateChecksum(stream.str().c_str(), stream.str().size());
        std::string streamContent = stream.str() + checkSumIndicator + std::to_string(checkSum) + messageEndSequence;

        //One could retrieve the return of send, but that does not tell us something about the other side
        
#ifdef _WIN32
    send(socket_handle, streamContent.c_str(), streamContent.size(), 0);
#else
    send(socket_handle, streamContent.c_str(), streamContent.size(), MSG_NOSIGNAL);
#endif
        

        const int mBufferLength = 32768;
        char *readBuffer = (char *) malloc(mBufferLength * sizeof(char));
        memset(readBuffer, 0, mBufferLength);
        int msgLength = recv(socket_handle, readBuffer, mBufferLength, 0);


        if (msgLength > 0)
        {
            if (wasCleaned)
            {
                std::string readString = std::string(readBuffer, msgLength);
                std::cout << "\tSocket Writer! Received Message again from Cedar Reader on Port: " << mPort << std::endl;
            }

            mIsConnected.store(true);
            startMeasurement = std::chrono::steady_clock::now();
            wasCleaned = false;
        } else
        {
            //I could not read! The original socket is  not there anymore
            std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
            int elapsedMilliSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startMeasurement).count();

            if (elapsedMilliSeconds > reconnectTimeOut)
            {

                startMeasurement = std::chrono::steady_clock::now();

                if (!wasCleaned)
                    std::cout << "\tSocket Writer! Port: "<< mPort <<" I have heard nothing from my reader for " << elapsedMilliSeconds << " ms! He might be down!" << std::endl;

                wasCleaned = true;
                reconnect();

            }
        }

        free(readBuffer);

    }

    void setMatrix(cv::Mat writeMatrix)
    {
        std::lock_guard<std::shared_timed_mutex> guard(mutex);
        currentWriteMatrix = writeMatrix.clone();
    }

    std::string getID()
    {
        return mID;
    }

    bool isConnected()
    {
        return mIsConnected.load();
    }


private:

    std::string matrixTypeToString(const cv::Mat &matrix)
    {
        switch (matrix.type())
        {
            case CV_8U:
                return "CV_8U";

            case CV_8UC2:
                return "CV_8UC2";

            case CV_8UC3:
                return "CV_8UC3";

            case CV_8UC4:
                return "CV_8UC4";


            case CV_8S:
                return "CV_8S";

            case CV_8SC3:
                return "CV_8SC3";


            case CV_16U:
                return "CV_16U";

            case CV_16UC3:
                return "CV_16UC3";

            case CV_16S:
                return "CV_16S";

            case CV_16SC3:
                return "CV_16SC3";


            case CV_32S:
                return "CV_32S";

            case CV_32SC3:
                return "CV_32SC3";

            case CV_32F:
                return "CV_32F";

            case CV_32FC3:
                return "CV_32FC3";


            case CV_64F:
                return "CV_64F";

            case CV_64FC3:
                return "CV_64FC3";


            default:
                return "Unknown type: " + matrix.type();
        }
    }


    void serialize(std::ostream &stream, cv::Mat matrixToSerialize) //Taken directly from cedar
    {
        stream << "Mat" << ",";
        stream << matrixTypeToString(matrixToSerialize) << ",";
        for (int i = 0; i < matrixToSerialize.dims; i++)
        {
            if (i > 0)
            {
                stream << ",";
            }
            stream << matrixToSerialize.size[i];
        }

        stream << ",compact";

        stream << std::endl;
        // create a string buffer that will hold the binary representation of all elements
        std::string buffer(matrixToSerialize.elemSize() * matrixToSerialize.total(), '\0');
        for (size_t i = 0; i < matrixToSerialize.total() * matrixToSerialize.elemSize(); ++i)
        {
            buffer[i] = matrixToSerialize.data[i];
        }
        stream.write(buffer.c_str(), buffer.size());
    }



private:
    std::string mID;
    int mPort;
    std::string mIpAddress;
    int socket_handle;
    struct sockaddr_in serv_addr; //Server Adress Object
    cv::Mat currentWriteMatrix;
    mutable std::shared_timed_mutex mutex; // is neither copyable nor movable.
    std::chrono::steady_clock::time_point startMeasurement;
    int reconnectTimeOut;
    std::atomic_bool mIsConnected;

    bool hasConnectedOnce;
    bool wasCleaned;
};