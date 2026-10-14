#include "15.h"

void follow() {
    err1 = 0;
    u = 0;
    oled.clearDisplay();
    put_text("WALL FOLLOW", 5, 10, 1, 1);
    put_text("L/R = STOP", 10, 28, 1, 1);
    oled.display();
    delay(800);
    while (1) {
        sample_pose();
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            drive(0, 0);
            wait_motion(200);
            return;
        }
        int v0 = 0, v2 = 0, v4 = 0;
        for (int r = 0; r < 3; r++) {
            ir_sample();
            v0 += w[0];
            v2 += w[2];
            v4 += w[4];
            delay(4);
        }
        bool front_wall = (v2 >= 2);
        bool left_wall = (v0 >= 2);
        bool right_wall = (v4 >= 2);
        turnFault = false;
        if (front_wall) {
            if (!right_wall)
                rotate(1);
            else if (!left_wall)
                rotate(0);
            else
                rotate(2);
        }
        if (turnFault) {
            drive(0, 0);
            return;
        }
        drive_mm(pitch);
        if (blocked) {
            drive(0, 0);
            return;
        }
        oled.clearDisplay();
        put_text("WF " + String(w[0]) + String(w[2]) + String(w[4]), 0, 0, 1, 1);
        for (byte i = 0; i < NUM_SENSORS; i++) {
            int h = map(s[i], 0, 4095, 0, 25);
            oled.fillRect(14 + 22 * i, 40 - h, 14, h, 1);
        }
        put_text("L:" + String((int)sL), 0, 44, 1, 1);
        put_text("R:" + String((int)sR), 65, 44, 1, 1);
        put_text("L/R=STOP", 28, 55, 1, 1);
        oled.display();
        if (debug) {
            debugf("[MAZE] w:%d%d%d | pid:%.1f | Lmm:%.0f Rmm:%.0f\n", w[0], w[2], w[4], u, sL, sR);
        }
    }
}
