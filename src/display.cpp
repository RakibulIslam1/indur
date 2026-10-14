#include "config.h"
#include "bitmap.h"

void put_text(String t, byte x, byte y, byte col, byte sz) {
    oled.setTextSize(sz);
    oled.setTextColor(col);
    oled.setCursor(x, y);
    oled.print(t);
}

void splash() {
    oled.clearDisplay();
    oled.drawBitmap(48, 5, logo, 32, 32, 1);
    put_text("MICROMOUSE", 34, 42, 1, 1);
    for (byte i = 3; i < 50; i += 2) {
        oled.fillRoundRect(39, 59, i, 4, 2, 1);
        delay(20);
        oled.display();
    }
}

void home() {
    static uint32_t debugTimer = 0;
    while (1) {
        sample_pose();
        console_poll();
        ir_sample();
        oled.clearDisplay();
        for (byte i = 0; i < 3; i++) {
            (i < 2) ? oled.drawRoundRect(41 * i, 0, 40, 11, 3, 1)
                    : oled.drawRoundRect(41 * i, 0, 46, 11, 3, 1);
        }
        put_text("L:" + String((int)sL), 2, 2, 1, 1);
        put_text("SP-" + String(pwm), 43, 2, 1, 1);
        put_text("R:" + String((int)sR), 84, 2, 1, 1);
        for (byte i = 0; i < NUM_SENSORS; i++) {
            dp[i] = map(s[i], 0, 4095, 0, 30);
            oled.fillRect(14 + 22 * i, 45 - dp[i], 14, dp[i], 1);
        }
        oled.fillRoundRect(0, 54, 40, 9, 2, 1);
        oled.fillRoundRect(44, 54, 40, 9, 2, 1);
        oled.fillRoundRect(88, 54, 40, 9, 2, 1);
        put_text(" HAND ", 2, 55, 0, 1);
        put_text(" MAZE", 46, 55, 0, 1);
        put_text(" MENU", 90, 55, 0, 1);
        if (lowPower) {
            oled.fillRect(0, 44, 128, 9, 0);
            put_text("! BATTERY LOW !", 18, 45, 1, 1);
        }
        if (latchedButton) {
            oled.fillRect(0, 34, 128, 9, 0);
            put_text("BTN " + String(latchedButton) + " STUCK LOW", 14, 35, 1, 1);
        }
        if (debug && millis() - debugTimer > 200) {
            debugTimer = millis();
            debugf("[SENS] raw:%4d %4d %4d %4d %4d | dist:%5.1f %5.1f %5.1f %5.1f %5.1f | "
                   "wall:%d%d%d%d%d\n",
                   s[0], s[1], s[2], s[3], s[4], d[0], d[1], d[2], d[3], d[4], w[0], w[1], w[2],
                   w[3], w[4]);
            debugf("[ENC]  Ltk:%ld Rtk:%ld | Lmm:%.1f Rmm:%.1f | Lmm/s:%.1f Rmm/s:%.1f\n", ticksL,
                   ticksR, sL, sR, vL, vR);
            debugf("[MOT]  lbase:%d rbase:%d | kp:%.1f kd:%.1f pid:%.2f\n", baseL, baseR, kp, kd,
                   u);
        }
        byte r = button(BTN_RIGHT);
        if (r == 2) {
            run_direct();
        } else if (r == 1) {
            r = menu();
            if (r == 1)
                settings();
            else if (r == 2)
                raw_view();
            else if (r == 3)
                range_view();
            else if (r == 4)
                drive_test();
            else if (r == 5)
                follow();
            else if (r == 7)
                search();
            else if (r == 8)
                run_saved();
            else if (r == 9)
                map_clear();
            else if (r == 10)
                run_fast();
            else if (r == 6) {
                debug = !debug;
                oled.clearDisplay();
                put_text(debug ? " DEBUG ON " : "DEBUG OFF", 10, 25, 1, 2);
                oled.display();
                delay(900);
            }
        }
        r = button(BTN_LEFT);
        if (r == 1)
            follow_hand(0);
        else if (r == 2)
            follow_hand(1);
        r = button(BTN_CENTER);
        if (r == 2) {
            search();
        } else if (r == 1) {
            run_direct(true);
        }
        oled.display();
    }
}
