#include "positiondetector.hpp"
#include "ledcallback.hpp"
#include "imureader.hpp"

#include <cstdlib>
#include <signal.h>
#include <sys/signalfd.h>
#include <unistd.h>

int main()
{
    // LEDController led(22, 0);

    // led.set(true);
    // sleep(10);
    // led.set(false);

    PositionDetector detector;
    IMUReader reader(1);

    if (!reader.init()) {
        return EXIT_FAILURE;
    }

    // detector.setCallback([&](bool upright) {
    //     led.set(upright);
    // });

    reader.setCallback([&](float ax, float ay, float az, float mx, float my) {
        detector.onSample(ax, ay, az, mx, my);
    });

    reader.start();

    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGHUP);

    if (sigprocmask(SIG_BLOCK, &mask, nullptr) == -1) {
        reader.stop();
        return EXIT_FAILURE;
    }

    int sfd = signalfd(-1, &mask, 0);
    if (sfd == -1) {
        reader.stop();
        return EXIT_FAILURE;
    }

    bool running = true;
    while (running) {
        signalfd_siginfo fdsi;
        ssize_t s = ::read(sfd, &fdsi, sizeof(fdsi));

        if (s != sizeof(fdsi)) break;

        if (fdsi.ssi_signo == SIGINT) running = false;
    }

    ::close(sfd);
    reader.stop();

    return EXIT_SUCCESS;
}