#include "samplereceiver.hpp"

#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <unistd.h>

SampleReceiver::SampleReceiver(const std::string& fifoPath)
    : path(fifoPath), fd(-1), running(false)
{ 
}

SampleReceiver::~SampleReceiver()
{
    stop();
}

void SampleReceiver::setCallback(SampleCallback cb)
{
    callback = cb;
}

void SampleReceiver::start()
{
    if (running) {
        return;
    }

    fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "Failed to open FIFO: %s\n", path.c_str());
        return;
    }

    running = true;
    worker = std::thread(&SampleReceiver::run, this);
}

void SampleReceiver::stop()
{
    running = false;

    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }

    if (worker.joinable()) {
        worker.join();
    }
}

void SampleReceiver::run()
{
    std::string pending;
    char buf[256];

    while (running) {
        ssize_t n = ::read(fd, buf, sizeof(buf));

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data yet, wait and retry
                usleep(10000); // 10ms
                continue;
            }
            std::fprintf(stderr, "read error on FIFO\n");
            break;
        }

        if (n == 0) {
            // Writer closed its end, wait and retry rather than giving up
            usleep(10000);
            continue;
        }

        pending.append(buf, static_cast<std::size_t>(n));

        std::size_t pos;
        while ((pos = pending.find('\n')) != std::string::npos) {
            std::string line = pending.substr(0, pos);
            pending.erase(0, pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (line.empty() || line.rfind("ts_ms,", 0) == 0) {
                continue;
            }

            std::stringstream ss(line);
            std::string ts, ax, ay, az, gx, gy, gz, mx, my, mz;

            std::getline(ss, ts, ',');
            std::getline(ss, ax, ',');
            std::getline(ss, ay, ',');
            std::getline(ss, az, ',');
            std::getline(ss, gx, ',');
            std::getline(ss, gy, ',');
            std::getline(ss, gz, ',');
            std::getline(ss, mx, ',');
            std::getline(ss, my, ',');
            std::getline(ss, mz, ',');

            if (ts.empty() || ax.empty() || ay.empty() || az.empty() ||
                mx.empty() || my.empty()) {
                continue;
            }

            try {
                if (callback) {
                    callback(
                        std::stof(ax),
                        std::stof(ay),
                        std::stof(az),
                        std::stof(mx),
                        std::stof(my)
                    );
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "parse error: %s  line: %s\n", e.what(), line.c_str());
            }
        }
    }

    running = false;
}