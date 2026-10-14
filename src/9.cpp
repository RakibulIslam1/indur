#include "15.h"
#include "16.h"

byte menu() {
    byte temp = 0, ind = 1;
    const byte limit = 10;
    while (1) {
        if (temp != ind) {
            temp = ind;
            oled.clearDisplay();
            oled.drawLine(126, 5, 126, 49, 1);
            oled.fillRoundRect(124, 5 + (ind - 1) * 4, 5, 5, 0, 1);
            oled.drawBitmap(48, 5, icons[(ind - 1) % 8], 32, 32, 1);
            put_text(menu_item(ind), 8, 42, 1, 1);
            oled.fillRoundRect(0, 53, 40, 11, 3, 1);
            oled.fillRoundRect(88, 53, 40, 11, 3, 1);
            put_text(" BACK ", 2, 55, 0, 1);
            put_text("SELECT", 90, 55, 0, 1);
            oled.display();
        }
        byte p = button(BTN_LEFT);
        if (p == 1) {
            ind--;
            if (ind < 1)
                ind = limit;
        } else if (p == 2) {
            return 0;
        }
        p = button(BTN_RIGHT);
        if (p == 1) {
            ind++;
            if (ind > limit)
                ind = 1;
        } else if (p == 2) {
            return ind;
        }
    }
}

String menu_item(byte a) {
    const char *labels[] = {"Settings",    "Analog Read",  "Dist. Read", "Motor Check",
                            "Wall Follow", "Debug Toggle", "Flood Fill", "Run Path",
                            "Clear Maze",  "Fast Run"};
    return a >= 1 && a <= 10 ? String(labels[a - 1]) : String("");
}
