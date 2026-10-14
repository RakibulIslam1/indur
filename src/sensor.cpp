#include "config.h"

void ir_read(int index) {
    for (int i = 0; i < NUM_SENSORS; i++)
        digitalWrite(irEnable[i], LOW);
    digitalWrite(irEnable[index], HIGH);
    delayMicroseconds(300);
    int sum = 0;
    for (int k = 0; k < 5; k++) {
        sum += analogRead(irPins[index]);
        delayMicroseconds(80);
    }
    s[index] = sum / 5;
    digitalWrite(irEnable[index], LOW);
}

float ir_range(int index) {
    int raw = s[index];
    if (raw <= b[index])
        return 100.0f;
    return (a[index] / (raw - b[index]) + c[index]) * 10.0f;
}

bool ir_wall(int index) {
    w[index] = (s[index] < wallThr[index]) ? 1 : 0;
    return w[index];
}

void ir_sample() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        ir_read(i);
        d[i] = ir_range(i);
        if (i == 0 && d[i] < 0)
            d[i] = 5.0f;
        ir_wall(i);
    }
}

void raw_view() {
    while (!button(BTN_LEFT) && !button(BTN_RIGHT)) {
        oled.clearDisplay();
        oled.fillRoundRect(0, 55, 40, 9, 3, 1);
        oled.fillRoundRect(88, 55, 40, 9, 3, 1);
        put_text(" BACK ", 2, 56, 0, 1);
        put_text(" BACK ", 90, 56, 0, 1);
        ir_sample();
        const byte xp[5] = {0, 26, 52, 78, 104};
        const byte yp[5] = {40, 25, 15, 25, 40};
        const char *lbl[5] = {"L", "LD", "F", "RD", "R"};
        for (byte i = 0; i < NUM_SENSORS; i++) {
            put_text(lbl[i], xp[i], yp[i] - 9, 1, 1);
            put_text(String(s[i]), xp[i], yp[i], 1, 1);
        }
        delay(50);
        oled.display();
    }
}

void range_view() {
    while (!button(BTN_LEFT) && !button(BTN_RIGHT)) {
        oled.clearDisplay();
        oled.fillRoundRect(0, 55, 40, 9, 3, 1);
        oled.fillRoundRect(88, 55, 40, 9, 3, 1);
        put_text(" BACK ", 2, 56, 0, 1);
        put_text(" BACK ", 90, 56, 0, 1);
        ir_sample();
        const byte xp[5] = {0, 26, 52, 78, 104};
        const byte yp[5] = {40, 25, 15, 25, 40};
        const char *lbl[5] = {"L", "LD", "F", "RD", "R"};
        for (byte i = 0; i < NUM_SENSORS; i++) {
            put_text(lbl[i], xp[i], yp[i] - 9, 1, 1);
            put_text(String(d[i], 1) + "mm", xp[i], yp[i], 1, 1);
        }
        oled.display();
    }
}
