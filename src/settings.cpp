#include "config.h"

String setting_name(byte a) {
    const char *labels[] = {"Drive Speed",     "Motor Balance",  "Turn Speed",      "Move Brake",
                            "Turn Brake",      "Wall Kp",        "Wall Kd",         "Wheelbase mm",
                            "Wall Setpoint",   "Straight Kp",    "Turn Decel deg",  "Wheel Diam x2",
                            "Front Threshold", "Left Threshold", "Right Threshold", "Goal Mode",
                            "Goal X",          "Goal Y"};
    return a >= 1 && a <= 18 ? String(labels[a - 1]) : String("");
}

uint16_t setting_addr(byte a) {
    const uint8_t slots[] = {25, 26, 27, 28, 29, 36, 37, 39, 41,
                             42, 43, 44, 46, 15, 16, 20, 18, 19};
    return a >= 1 && a <= 18 ? slots[a - 1] : 24;
}

static String setting_value(byte a, int raw) {
    if (a == 2)
        return String((raw - 25) * 2);
    if (a == 1 || a == 3 || a == 4 || a == 5 || a == 7)
        return String(raw * 10);
    if (a == 9 || a == 14 || a == 15)
        return String(raw * 16);
    if (a == 12)
        return String(raw * 0.5f, 1);
    if (a == 16)
        return raw == 0 ? "CENTER" : raw == 1 ? "SINGLE" : "2x2";
    return String(raw);
}

void settings() {
    byte ind = 1, shown = 0;
    while (1) {
        sample_pose();
        if (ind != shown) {
            oled.clearDisplay();
            shown = ind;
            put_text("SET " + String(ind) + "/18", 20, 4, 1, 1);
            put_text(setting_name(ind), 4, 22, 1, 1);
            put_text(setting_value(ind, EEPROM.read(setting_addr(ind))), 4, 36, 1, 1);
            put_text("L=PREV R=NEXT", 4, 54, 1, 1);
            oled.display();
        }
        byte p = button(BTN_LEFT);
        if (p == 2) {
            settings_load();
            return;
        }
        if (p == 1)
            ind = ind == 1 ? 18 : ind - 1;
        p = button(BTN_RIGHT);
        if (p == 1)
            ind = ind == 18 ? 1 : ind + 1;
        if (p == 2) {
            setting_edit(ind);
            shown = 0;
        }
    }
}

void setting_edit(int indicator) {
    const uint8_t limits[] = {25,  50,  25,  50,  50,  100, 50, 120, 93,
                              100, 100, 120, 250, 250, 250, 2,  12,  12};
    if (indicator < 1 || indicator > 18)
        return;
    uint16_t addr = setting_addr(indicator);
    int value = constrain((int)EEPROM.read(addr), 0, (int)limits[indicator - 1]);
    int shown = -1;
    while (1) {
        sample_pose();
        if (shown != value) {
            oled.clearDisplay();
            shown = value;
            put_text(setting_name(indicator), 4, 8, 1, 1);
            put_text(setting_value(indicator, value), 4, 28, 1, 2);
            put_text("L=+ R=-", 4, 46, 1, 1);
            put_text("HOLD L=BACK R=SAVE", 4, 56, 1, 1);
            oled.display();
        }
        byte p = button(BTN_LEFT);
        if (p == 2)
            return;
        if (p == 1)
            value = min(value + 1, (int)limits[indicator - 1]);
        p = button(BTN_RIGHT);
        if (p == 1)
            value = max(value - 1, 0);
        if (p == 2) {
            EEPROM.write(addr, value);
            delay(10);
            EEPROM.commit();
            settings_load();
            return;
        }
    }
}
