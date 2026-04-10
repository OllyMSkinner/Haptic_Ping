#include "rpi_pwm.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    int channel = 2;
    int frequency = 100; // Hz
    if (argc > 1) {
	channel = atoi(argv[1]);
    }
    printf("Enabling PWM on channel %d.\n",channel);
    RPI_PWM pwm;
    pwm.start(channel, frequency);
    printf("Duty cycle at 50%%\n");
    pwm.setDutyCycle(50);
    getchar();
    printf("Duty cycle at 25%%\n");
    pwm.setDutyCycle(25);
    getchar();
    printf("Duty cycle at 75%%\n");
    pwm.setDutyCycle(75);
    getchar();
    pwm.stop();
}
