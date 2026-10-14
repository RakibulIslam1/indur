#include "15.h"

byte button(byte btn) {
    bool flag = true;
    uint32_t b_timer = 0;
    if (!digitalRead(btn)) {
        delay(50);
        while (!digitalRead(btn)) {
            delay(50);
            b_timer += 50;
            if (b_timer > BTN_STUCK_MS) {
                latchedButton = btn;
                break;
            }
            if (b_timer > LONG_PRESS_MS && flag) {
                flag = false;
                oled.clearDisplay();
                if (btn == BTN_LEFT) {
                    put_text("   BACK   ", 5, 27, 1, 2);
                } else if (btn == BTN_RIGHT) {
                    put_text("  SELECT  ", 5, 27, 1, 2);
                } else if (btn == BTN_CENTER) {
                    put_text("  ACTION  ", 5, 27, 1, 2);
                }
                oled.display();
            }
        }
        if (b_timer > LONG_PRESS_MS)
            return 2;
        else if (b_timer > 0)
            return 1;
    }
    return 0;
}

bool held(byte btn, bool indicator) {
    delay(50);
    uint32_t b_timer = millis();
    while (!digitalRead(btn))
        if (millis() - b_timer > 1000 || indicator)
            return true;
    delay(50);
    return false;
}
