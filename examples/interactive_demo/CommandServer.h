#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <sstream>
#include <vector>
#include <iostream>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef SOCKET socket_t;
    #define INVALID_SOCK INVALID_SOCKET
    #define CLOSE_SOCKET closesocket
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    typedef int socket_t;
    #define INVALID_SOCK -1
    #define CLOSE_SOCKET close
#endif

namespace arborvia {

struct Command {
    std::string name;
    std::vector<std::string> args;
};

class CommandServer {
public:
    CommandServer(int port = 9999) : port_(port), running_(false), serverSocket_(INVALID_SOCK) {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }

    ~CommandServer() {
        stop();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool start() {
        serverSocket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket_ == INVALID_SOCK) {
            std::cerr << "[CommandServer] Failed to create socket" << std::endl;
            return false;
        }

        // Allow port reuse
        int opt = 1;
#ifdef _WIN32
        setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
#else
        setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);

        if (bind(serverSocket_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[CommandServer] Failed to bind port " << port_ << std::endl;
            CLOSE_SOCKET(serverSocket_);
            serverSocket_ = INVALID_SOCK;
            return false;
        }

        if (listen(serverSocket_, 5) < 0) {
            std::cerr << "[CommandServer] Failed to listen" << std::endl;
            CLOSE_SOCKET(serverSocket_);
            serverSocket_ = INVALID_SOCK;
            return false;
        }

        // Set non-blocking
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(serverSocket_, FIONBIO, &mode);
#else
        fcntl(serverSocket_, F_SETFL, O_NONBLOCK);
#endif

        running_ = true;
        serverThread_ = std::thread(&CommandServer::serverLoop, this);

        std::cout << "[CommandServer] Listening on port " << port_ << std::endl;
        return true;
    }

    void stop() {
        running_ = false;
        if (serverSocket_ != INVALID_SOCK) {
            CLOSE_SOCKET(serverSocket_);
            serverSocket_ = INVALID_SOCK;
        }
        if (serverThread_.joinable()) {
            serverThread_.join();
        }
    }

    bool hasCommand() {
        std::lock_guard<std::mutex> lock(mutex_);
        return !commandQueue_.empty();
    }

    Command popCommand() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (commandQueue_.empty()) {
            return {"", {}};
        }
        Command cmd = commandQueue_.front();
        commandQueue_.pop();
        return cmd;
    }

    void sendResponse(const std::string& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (clientSocket_ != INVALID_SOCK) {
            std::string msg = response + "\n";
            send(clientSocket_, msg.c_str(), msg.size(), 0);
        }
    }

private:
    void serverLoop() {
        while (running_) {
            // Accept new connections
            sockaddr_in clientAddr{};
            socklen_t clientLen = sizeof(clientAddr);
            socket_t newClient = accept(serverSocket_, (sockaddr*)&clientAddr, &clientLen);

            if (newClient != INVALID_SOCK) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (clientSocket_ != INVALID_SOCK) {
                    CLOSE_SOCKET(clientSocket_);
                }
                clientSocket_ = newClient;
                std::cout << "[CommandServer] Client connected" << std::endl;

#ifdef _WIN32
                u_long mode = 1;
                ioctlsocket(clientSocket_, FIONBIO, &mode);
#else
                fcntl(clientSocket_, F_SETFL, O_NONBLOCK);
#endif
            }

            // Read from client
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (clientSocket_ != INVALID_SOCK) {
                    char buffer[1024];
                    int n = recv(clientSocket_, buffer, sizeof(buffer) - 1, 0);
                    if (n > 0) {
                        buffer[n] = '\0';
                        parseCommands(buffer);
                    } else if (n == 0) {
                        // Client disconnected
                        CLOSE_SOCKET(clientSocket_);
                        clientSocket_ = INVALID_SOCK;
                        std::cout << "[CommandServer] Client disconnected" << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void parseCommands(const std::string& data) {
        std::istringstream stream(data);
        std::string line;
        while (std::getline(stream, line)) {
            if (line.empty() || line[0] == '#') continue;

            // Remove carriage return if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            Command cmd;
            std::istringstream lineStream(line);
            lineStream >> cmd.name;

            std::string arg;
            while (lineStream >> arg) {
                cmd.args.push_back(arg);
            }

            if (!cmd.name.empty()) {
                commandQueue_.push(cmd);
                std::cout << "[CommandServer] Received: " << cmd.name;
                for (const auto& a : cmd.args) std::cout << " " << a;
                std::cout << std::endl;
            }
        }
    }

    int port_;
    std::atomic<bool> running_;
    socket_t serverSocket_;
    socket_t clientSocket_ = INVALID_SOCK;
    std::thread serverThread_;
    std::mutex mutex_;
    std::queue<Command> commandQueue_;
};

} // namespace arborvia
