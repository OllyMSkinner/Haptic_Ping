#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

class SampleReceiver
{
public:
    using SampleCallback =
        std::function<void(float ax, float ay, float az, float mx, float my)>;

    explicit SampleReceiver(const std::string& fifoPath);
    ~SampleReceiver();

    void setCallback(SampleCallback cb);
    void start();
    void stop();

private:
    void run();

    std::string path;
    int fd;
    std::atomic<bool> running;
    std::thread worker;
    SampleCallback callback;
};