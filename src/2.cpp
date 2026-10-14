#include "15.h"

void sound_init() {
    ledcSetup(CHAN_BUZZ, 2000, 8);
    ledcAttachPin(BUZZER_PIN, CHAN_BUZZ);
    ledcWrite(CHAN_BUZZ, 0);
}

static void tone_out(int freq, int ms) {
    ledcWriteTone(CHAN_BUZZ, freq);
    ledcWrite(CHAN_BUZZ, 128);
    delay(ms);
    ledcWrite(CHAN_BUZZ, 0);
    delay(30);
}

void sound_start() {
    tone_out(600, 80);
    tone_out(1000, 120);
}

void sound_goal() {
    tone_out(800, 80);
    tone_out(1200, 80);
    tone_out(1600, 150);
}

void sound_home() {
    tone_out(500, 100);
    tone_out(1000, 200);
}

void sound_phase(int n) {
    for (int i = 0; i < n && i < 6; i++) {
        tone_out(1000, 60);
        delay(60);
    }
}

void sound_error() {
    tone_out(1200, 60);
    tone_out(900, 60);
    tone_out(600, 60);
    tone_out(400, 60);
    tone_out(200, 120);
}

void sound_done() {
    tone_out(523, 80);
    tone_out(659, 80);
    tone_out(784, 80);
    tone_out(1047, 200);
}
