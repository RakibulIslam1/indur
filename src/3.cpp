#include "15.h"

void console_poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '?')
            Serial.println("micromouse: p=status, z=reset distance, !=stop");
        else if (c == 'p')
            Serial.printf("L:%.1f R:%.1f Speed:%d Maze:13x13\n", sL, sR, pwm);
        else if (c == 'z')
            distance_zero();
        else if (c == '!')
            drive(0, 0);
    }
}
