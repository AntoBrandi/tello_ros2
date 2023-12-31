#include "tello_ros2/tello.hpp"

#include <sstream>

#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <memory.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>


namespace
{

// Binds the given socket file descriptor ot the given port.
// Returns whether it succeeds or not and the error message.
std::pair<bool, std::string> BindSocketToPort(const int sockfd, const int port)
{
    sockaddr_in listen_addr{};
    // htons converts from host byte order to network byte order.
    listen_addr.sin_port = htons(port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_family = AF_INET;
    int result = bind(sockfd, reinterpret_cast<sockaddr*>(&listen_addr),
                      sizeof(listen_addr));

    if (result == -1)
    {
        std::stringstream ss;
        ss << "bind to " << port << ": " << errno;
        ss << " (" << strerror(errno) << ")";
        return {false, ss.str()};
    }

    return {true, ""};
}

// Finds the socket address given an ip and a port.
// Returns whether it succeeds or not and the error message.
std::pair<bool, std::string> FindSocketAddr(const char* const ip,
                                            const char* const port,
                                            sockaddr_storage* const addr)
{
    addrinfo* result_list{nullptr};
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    int result = getaddrinfo(ip, port, &hints, &result_list);

    if (result)
    {
        std::stringstream ss;
        ss << "getaddrinfo: " << result;
        ss << " (" << gai_strerror(result) << ") ";
        return {false, ss.str()};
    }

    memcpy(addr, result_list->ai_addr, result_list->ai_addrlen);
    freeaddrinfo(result_list);

    return {true, ""};
}

// Sends a string of bytes to the given destination address.
// Returns the number of sent bytes and, if -1, the error message.
std::pair<int, std::string> SendTo(const int sockfd,
                                   sockaddr_storage& dest_addr,
                                   const std::vector<unsigned char>& message)
{
    const socklen_t addr_len{sizeof(dest_addr)};
    int result = sendto(sockfd, message.data(), message.size(), 0,
                        reinterpret_cast<sockaddr*>(&dest_addr), addr_len);

    if (result == -1)
    {
        std::stringstream ss;
        ss << "sendto: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }

    return {result, ""};
}

// Receives a text response from the given destination address.
// Returns the number of received bytes and, if -1, the error message.
std::pair<int, std::string> ReceiveFrom(const int sockfd,
                                        sockaddr_storage& addr,
                                        std::vector<unsigned char>& buffer,
                                        const int buffer_size = 1024,
                                        const int flags = MSG_DONTWAIT)
{
    socklen_t addr_len{sizeof(addr)};
    buffer.resize(buffer_size, '\0');
    // MSG_DONTWAIT -> Non-blocking
    // recvfrom is storing (re-populating) the sender address in addr.
    int result = recvfrom(sockfd, buffer.data(), buffer_size, flags,
                          reinterpret_cast<sockaddr*>(&addr), &addr_len);
    if (result == -1)
    {
        std::stringstream ss;
        ss << "recvfrom: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }

    return {result, ""};
}
}  // namespace

namespace tello_ros2
{
Tello::Tello()
{
    m_command_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    m_state_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
}

Tello::~Tello()
{
    close(m_command_sockfd);
    close(m_state_sockfd);
}

bool Tello::Bind(const int local_client_command_port)
{
    // UDP Client to send commands and receive responses
    auto result =
        ::BindSocketToPort(m_command_sockfd, local_client_command_port);
    if (!result.first)
    {
        std::cerr << result.second << std::endl;
        return false;
    }
    m_local_client_command_port = local_client_command_port;
    result = ::FindSocketAddr(TELLO_SERVER_IP, TELLO_SERVER_COMMAND_PORT,
                              &m_tello_server_command_addr);
    if (!result.first)
    {
        std::cerr << result.second << std::endl;
        return false;
    }

    // Local UDP Server to listen for the Tello Status
    result = ::BindSocketToPort(m_state_sockfd, LOCAL_SERVER_STATE_PORT);
    if (!result.first)
    {
        std::cerr << result.second << std::endl;
        return false;
    }

    // Finding Tello
    std::cerr << "Finding Tello ..." << std::endl;
    FindTello();
    std::cerr << "Entered SDK mode" << std::endl;

    ShowTelloInfo();

    return true;
}

void Tello::FindTello()
{
    do
    {
        SendCommand("command");
        sleep(1);
    } while (!(ReceiveResponse()));
}

void Tello::ShowTelloInfo()
{
    std::experimental::optional<std::string> response;

    SendCommand("sn?");
    while (!(response = ReceiveResponse()))
        ;
    std::cerr << "Serial Number: " << *response << std::endl;

    SendCommand("sdk?");
    while (!(response = ReceiveResponse()))
        ;
    std::cerr << "Tello SDK:     " << *response << std::endl;

    SendCommand("wifi?");
    while (!(response = ReceiveResponse()))
        ;
    std::cerr << "Wi-Fi Signal:  " << *response << std::endl;

    SendCommand("battery?");
    while (!(response = ReceiveResponse()))
        ;
    std::cerr << "Battery:       " << *response << std::endl;
}

bool Tello::SendCommand(const std::string& command)
{
    const std::vector<unsigned char> message{std::cbegin(command),
                                             std::cend(command)};
    const auto result =
        ::SendTo(m_command_sockfd, m_tello_server_command_addr, message);
    const int bytes{result.first};
    if (bytes == -1)
    {
        std::cerr << result.second << std::endl;
        return false;
    }
    return true;
}

std::experimental::optional<std::string> Tello::ReceiveResponse()
{
    const int size{32};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result = ::ReceiveFrom(
        m_command_sockfd, m_tello_server_command_addr, buffer, size);
    const int bytes{result.first};
    if (bytes < 1)
    {
        return {};
    }
    std::string response{buffer.cbegin(), buffer.cbegin() + bytes};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);
    return response;
}

void Tello::GetState(std::map<std::string,std::string>& tello_stat)
{
    sockaddr_storage addr;
    const int size{1024};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result = ::ReceiveFrom(m_state_sockfd, addr, buffer, size);
    const int bytes{result.first};
    if (bytes < 1)
    {
        return;
    }
    std::string response{std::cbegin(buffer), std::cbegin(buffer) + bytes};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);

    if(!response.empty())
    {
        int start = 0, end1 = 0, end2 = 0;
        while ((end1 = response.find(":", start)) != std::string::npos 
            && (end2 = response.find(";", end1+1)) != std::string::npos) 
        {
            std::string key = response.substr(start, end1 - start);
            std::string val = response.substr(end1 + 1, end2 - end1 - 1);
            tello_stat.insert(std::pair<std::string,std::string>(key, val));
            start = end2 + 1;
        }
    }
}

void Tello::OpenStream()
{
    capture_.open(TELLO_STREAM_URL, cv::CAP_FFMPEG);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, TELLO_CAMERA_WIDTH);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, TELLO_CAMERA_HEIGHT);
}

void Tello::CloseStream()
{
    capture_.release();
}

void Tello::GetFrame(cv::Mat& frame)
{
    if(capture_.isOpened())
    {
        capture_ >> frame;
    }
}
}  // namespace tello_ros2