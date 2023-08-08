#include <vector>
#include <opencv2/core/core.hpp>
#include <shared_mutex>
#include <chrono>
#include <ctime>
#include "utilities.h"

#ifdef _WIN32

#include <winsock2.h>

#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#include <fcntl.h>

class TCP_Socket_Reader
{
public:

    TCP_Socket_Reader(std::string  id, int port, int bufferLength) :
            mID(id),
            mPort(port),
            mBufferLength(bufferLength),
            mOverflowBuffer(""),
            currentReadMatrix(cv::Mat::zeros(42, 1, CV_32F)),
            timeSinceLastConnectTrial(0),
            waitTime(10),
            numberOfFailedReads(0),
            maxNumberOfFailedReads(20),
            wasCleaned(true)
    {
        mIsConnected.store(false);
        establishConnection();
    }

    void establishConnection()
    {
        wasCleaned = true;
        raw_socket_handle = create_socket_server(mPort);
        std::cout<<"Socket Reader! Port " << mPort << " Establish Connection" << std::endl;
        socket_handle = accept_client(raw_socket_handle);
    }

    void closeSocket()
    {
        std::cout << "Closing the Read Socket on port: " << mPort << std::endl;
#ifdef _WIN32
        closesocket(socket_handle);
        closesocket(raw_socket_handle);
        WSACleanup();
#else
        close(socket_handle);
        close(raw_socket_handle);
#endif
        mIsConnected.store(false);

    }

    ~TCP_Socket_Reader()
    {
        closeSocket();
    }

    void reconnect()
    {
        closeSocket();
        establishConnection();
    }

    void readNextMessage(std::string messageEndSequence, std::string checkSumIndicator)
    {
        if (!isConnected())
        {
            timeSinceLastConnectTrial = timeSinceLastConnectTrial + 1;
            if (timeSinceLastConnectTrial > waitTime)
            {
                timeSinceLastConnectTrial = 0;
                if (raw_socket_handle != -1)
                {
//                    std::cout<<"Socket Reader! Port " << mPort << " ReadNextMessage exceeded Waittime of " << waitTime << std::endl;
                    socket_handle = accept_client(raw_socket_handle);
                } else
                {
                    reconnect();
                }
            }
            return;
        }


        FD_ZERO(&rfds);
        FD_SET(socket_handle, &rfds);
        char *buffer = (char *) malloc(mBufferLength * sizeof(char));
        std::string fullMessageString = "";

        if (mOverflowBuffer != "")
        {
            fullMessageString += mOverflowBuffer;
            mOverflowBuffer = "";
        }

        bool endFound = false;
        int msgLength;


        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        bool timeout = false;

        while (!endFound && !timeout)
        {
            end = std::chrono::steady_clock::now();
            timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > 3000; //3second timeout
            memset(buffer, 0, mBufferLength);
            msgLength = recv(socket_handle, buffer, mBufferLength, 0);

            if (msgLength > 0)
            {
                numberOfFailedReads = 0;
                std::string concatString = std::string(buffer, msgLength); //This should do it properly

                //Let's see, if we found the end of a Msg. I am not sure why this is sometimes omitted.
                if (concatString.find(messageEndSequence) != std::string::npos)
                {
                    endFound = true;
                    std::vector<std::string> splitStrings;
                    split(concatString, messageEndSequence, splitStrings);
                    if (splitStrings.size() == 2)
                    {
                        concatString = splitStrings[0];
                        mOverflowBuffer = splitStrings[1];
                        fullMessageString += concatString; // The final Matrix String
                    } else if (splitStrings.size() > 2)
                    { //"We have more than one Message in this one reading!
                        fullMessageString = splitStrings[splitStrings.size() - 2]; //The last guaranteed full Matrix
                        mOverflowBuffer = splitStrings[splitStrings.size() - 1];
                    }

                } else
                { // We are not at the end! Let's continue concatenating.
                    fullMessageString += concatString;
                }
            } else
            {
                numberOfFailedReads = numberOfFailedReads +1;

                if(numberOfFailedReads > maxNumberOfFailedReads)
                {
                    numberOfFailedReads = 0;
//                    if (raw_socket_handle != -1)
//                    {
////                        std::cout<<"Socket Reader! Port " << mPort << " Exceeded NumberOfFailedReads: " << maxNumberOfFailedReads << std::endl;
//                        socket_handle = accept_client(raw_socket_handle);
//                    } else
//                    {
//                        reconnect();
//                    }
                    mIsConnected.store(false); // This should have the same effect as above. If reading failed, I am obviously not connected anymore...
                }

                //Still need to free the Memory here...
                free(buffer);
                return;
            }

        }
        free(buffer);

        if (timeout)
        {
            std::cout << "TCPSocketReader! Port: " << mPort <<" Exited through Timeout! There seems to be some disconnection!" << std::endl;
            reconnect();
            return;
        }


        std::vector<std::string> chkSumSplit;
        split(fullMessageString, checkSumIndicator, chkSumSplit);

        std::string matrixString = "";
        std::string sentChecksum = "";

        if (chkSumSplit.size() == 2)
        {
            matrixString = chkSumSplit[0];
            sentChecksum = chkSumSplit[1];
        } else
        {
            std::cout << "TCPSocketReader! Port: " << mPort <<" The message had more than one Checksum included. Abort" << std::endl;
            return;
        }

        std::string genChecksum = std::to_string(generateChecksum(matrixString.c_str(), matrixString.size()));

        if (genChecksum != sentChecksum)
        {
            std::cout << "TCPSocketReader! Port: " << mPort <<" Checksums do not match! Retrieved Checksum: >" << sentChecksum << "<\nGenerated Checksum: >" << genChecksum << "<" << std::endl;
            return;
        }

        confirmAliveStatus();

        std::stringstream inputStream; // In the future we might get rid of the stream here
        inputStream << fullMessageString;
        cv::Mat rcvMat = deserialize(inputStream);



        std::lock_guard<std::shared_timed_mutex> guard(mutex);
        currentReadMatrix = rcvMat;
    }

    cv::Mat getMatrix()
    {
        std::lock_guard<std::shared_timed_mutex> guard(mutex);
        cv::Mat retMat = currentReadMatrix.clone();
        return retMat;
    }

    std::string getID()
    {
        return mID;
    }

    bool isConnected()
    {
        return mIsConnected.load();
    }

    int accept_client(int server_fd)
    {
        int cfd;
        struct sockaddr_in client;

#ifndef _WIN32
        socklen_t asize;
#else
        int asize;
#endif

        struct hostent *client_info;

        asize = sizeof(struct sockaddr_in);



        cfd = accept(server_fd, (struct sockaddr *) &client, &asize);
        if (cfd == -1)
        {
#ifdef _WIN32
            if (WSAGetLastError() != 10035)
                std::cout << "Accept Failed due to WSAGetLastError: "<< WSAGetLastError() << std::endl;
#endif
            return -1;
        }


        client_info = gethostbyname((char *) inet_ntoa(client.sin_addr));

//        printf("Accepted connection from: %s \n", client_info->h_name);
        if(wasCleaned)
        {
            std::cout << "TCPSocketReader! Port: " << mPort << " Accepted Connection from: " << client_info->h_name << std::endl;
        }

        mIsConnected.store(true);
        FD_ZERO(&rfds);
        FD_SET(cfd, &rfds);

        wasCleaned = false;

        return cfd;
    }


    int create_socket_server(int port)
    {
        std::cout << "Create a reading Socket for port: " << port << std::endl;
        int sfd, rc;
        struct sockaddr_in address;

#ifdef _WIN32
        /* initialize the socket api */
        WSADATA info;

        rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
        if (rc != 0)
        {
            printf("cannot initialize Winsock\n");
            return -1;
        }
#endif
        /* create the socket */
        sfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sfd == -1)
        {
            printf("cannot create socket\n");
            return -1;
        }
        

#ifdef _WIN32
            const char opt = 1;
            if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR,
                           &opt, sizeof(opt)))
            {
                std::cout << "Error Setting Socket Options" << std::endl;
            }

            unsigned long on = 1;
            ioctlsocket(sfd, FIONBIO, &on); //make socket Non-blockable in windows
#else
            int opt = 1;
            if (setsockopt(sfd, SOL_SOCKET, SO_REUSEPORT,
                   &opt, sizeof(opt)))
            {
                std::cout << "Error Setting Socket Options" << std::endl;
            }
            fcntl(sfd, F_SETFL, O_NONBLOCK); //make socket Non-blockable
#endif

        /* fill in socket address */
        memset(&address, 0, sizeof(struct sockaddr_in));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = INADDR_ANY;



        /* bind to port */
        rc = bind(sfd, (struct sockaddr *) &address, sizeof(struct sockaddr));
        if (rc == -1)
        {
            printf("cannot bind port %d\n", port);
            closeSocket();
            return -1;
        }

        /* listen for connections */
        if (listen(sfd, 50) == -1)
        {
            printf("cannot listen for connections\n");
            closeSocket();
            return -1;
        }
        printf("Socket Reader! Waiting for a connection on port %d...\n", port);


        return sfd;
    }

private: 
//Crazy C++ OpenCV Magic ...
    int matrixTypeFromString(const std::string &typeStr)
    {
#define MAT_TYPE_STR(CV_TYPE) else if (typeStr == #CV_TYPE) { return CV_TYPE; }
        if (false)
        {
            // empty to allow else if blocks in the macro
        } MAT_TYPE_STR(CV_8U)MAT_TYPE_STR(CV_8UC1)MAT_TYPE_STR(CV_8UC2)MAT_TYPE_STR(CV_8UC3)MAT_TYPE_STR(
                CV_8UC4)MAT_TYPE_STR(CV_8S)MAT_TYPE_STR(CV_8SC1)MAT_TYPE_STR(CV_8SC2)MAT_TYPE_STR(CV_8SC3)MAT_TYPE_STR(
                CV_8SC4)MAT_TYPE_STR(CV_16U)MAT_TYPE_STR(CV_16UC1)MAT_TYPE_STR(CV_16UC2)MAT_TYPE_STR(
                CV_16UC3)MAT_TYPE_STR(
                CV_16UC4)MAT_TYPE_STR(CV_16S)MAT_TYPE_STR(CV_16SC1)MAT_TYPE_STR(CV_16SC2)MAT_TYPE_STR(
                CV_16SC3)MAT_TYPE_STR(
                CV_16SC4)MAT_TYPE_STR(CV_32S)MAT_TYPE_STR(CV_32SC1)MAT_TYPE_STR(CV_32SC2)MAT_TYPE_STR(
                CV_32SC3)MAT_TYPE_STR(
                CV_32SC4)MAT_TYPE_STR(CV_32F)MAT_TYPE_STR(CV_32FC3)MAT_TYPE_STR(CV_64F)MAT_TYPE_STR(CV_64FC3)

#undef MAT_TYPE_STR
    }

    cv::Mat deserialize(std::istream &stream)
    {

        std::string header;
        std::getline(stream, header);


        if (header.rfind("Mat", 0) == 0)
        {
            std::vector<std::string> header_entries;
            split(header, ",", header_entries);
            std::string data_type = header_entries.at(0);
            std::string mat_type_str = header_entries.at(1);

            int mat_type = matrixTypeFromString(mat_type_str);
            std::vector<int> sizes;

            size_t offset = 1;

            for (size_t i = 2; i < header_entries.size() - offset; ++i)
            {
                sizes.push_back(std::stoi(header_entries.at(i)));
            }

            cv::Mat mat(static_cast<int>(sizes.size()), &sizes.front(), mat_type);
            if (mat.isContinuous())
            {
                stream.read(reinterpret_cast<char *>(mat.data), static_cast<size_t>(mat.total() * mat.elemSize()));
            } else
            {
                std::cout << "Matrix is somehow not continuous! This shouldn't happen.!" << std::endl;
            }

            return mat;
        } else
        {
            std::cout << "The header did not begin with 'Mat'" << std::endl;
        }

        return cv::Mat::zeros(2, 2, CV_32F);
    }

    void confirmAliveStatus()
    {
        //Why is this chrono interface so hard to convert ...
        std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char buf[20];
        strftime(buf, 20, "%d.%m.%Y %H:%M:%S", localtime(&t));
        std::string timeString(buf);
    
        std::string msgContent = "Reader: " +std::to_string(mPort) + " alive! " + timeString ;
        
#ifdef _WIN32
    send(socket_handle, msgContent.c_str(), msgContent.size(), 0);
#else
    send(socket_handle, msgContent.c_str(), msgContent.size(), MSG_NOSIGNAL);
#endif
    }




private:
    std::string mID;
    int mPort;
    int socket_handle;
    int raw_socket_handle;
    const int mBufferLength;
    std::string mOverflowBuffer;
    fd_set rfds;
    cv::Mat currentReadMatrix;
    std::atomic_bool mIsConnected;
    int timeSinceLastConnectTrial;
    int waitTime;
    int debugCounter = 0;
    int debugBlockError = 0;
    int numberOfFailedReads;
    int maxNumberOfFailedReads;
    bool wasCleaned;
    mutable std::shared_timed_mutex mutex; // is neither copyable nor movable.
};