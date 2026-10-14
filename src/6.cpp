#include "15.h"
float psi = 0.0f;
static float psiTotal = 0.0f;
static uint16_t sampleL = 0;
static uint16_t sampleR = 0;
static uint32_t sampleMs = 0;

uint16_t encoder_read(TwoWire &wire) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG_H);
    if (wire.endTransmission(false) != 0)
        return AS5600_BAD;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2)
        return AS5600_BAD;
    if (wire.available() < 2)
        return AS5600_BAD;
    uint16_t hi = wire.read();
    uint16_t lo = wire.read();
    return ((hi << 8) | lo) & 0x0FFF;
}

uint8_t encoder_status(TwoWire &wire) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG_ST);
    if (wire.endTransmission(false) != 0)
        return 0;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1)
        return 0;
    if (wire.available() < 1)
        return 0;
    return wire.read();
}

uint8_t reg_read8(TwoWire &wire, uint8_t reg) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(reg);
    if (wire.endTransmission(false) != 0)
        return 0;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1)
        return 0;
    if (wire.available() < 1)
        return 0;
    return wire.read();
}

uint16_t reg_read16(TwoWire &wire, uint8_t reg) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(reg);
    if (wire.endTransmission(false) != 0)
        return 0;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2)
        return 0;
    if (wire.available() < 2)
        return 0;
    uint16_t hi = wire.read();
    uint16_t lo = wire.read();
    return (uint16_t)((hi << 8) | lo);
}

void encoder_config(TwoWire &wire) {
    uint8_t confLo = reg_read8(wire, AS5600_REG_CONF + 1);
    confLo = (uint8_t)((confLo & ~0x0C) | AS5600_HYST_2LSB);
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG_CONF + 1);
    wire.write(confLo);
    wire.endTransmission();
}

void encoder_snapshot() {
    statusL = encoder_status(Wire);
    statusR = encoder_status(Wire1);
    agcL = reg_read8(Wire, AS5600_REG_AGC);
    agcR = reg_read8(Wire1, AS5600_REG_AGC);
    magL = reg_read16(Wire, AS5600_REG_MAG);
    magR = reg_read16(Wire1, AS5600_REG_MAG);
}

void encoder_init() {
    uint16_t la = encoder_read(Wire);
    uint16_t ra = encoder_read(Wire1);
    if (la != AS5600_BAD)
        sampleL = la;
    if (ra != AS5600_BAD)
        sampleR = ra;
    sampleMs = millis();
    ticksL = ticksR = 0;
    sL = sR = 0;
    vL = vR = 0;
}

void distance_zero() {
    encoder_init();
}

void pose_zero() {
    encoder_init();
    psiTotal = 0.0f;
    psi = 0.0f;
}

void sample_pose() {
    uint32_t now = millis();
    float dt = (now - sampleMs) / 1000.0f;
    if (dt < 0.001f)
        return;
    const float MM_STEP = (diam * PI) / (float)ENC_CPR;
    uint16_t la = encoder_read(Wire);
    uint16_t ra = encoder_read(Wire1);
    if (la == AS5600_BAD || ra == AS5600_BAD) {
        readErrors++;
        sampleMs = now;
        return;
    }
    int16_t dl = (int16_t)(la - sampleL);
    if (dl > 2048)
        dl -= 4096;
    if (dl < -2048)
        dl += 4096;
    int16_t dr = (int16_t)(ra - sampleR);
    if (dr > 2048)
        dr -= 4096;
    if (dr < -2048)
        dr += 4096;
    float maxTicks = (ENC_MAX_MM_S * dt) / MM_STEP + 4.0f;
    if (fabsf((float)dl) > maxTicks || fabsf((float)dr) > maxTicks) {
        rejected++;
        sampleMs = now;
        return;
    }
    ticksL += dl;
    ticksR -= dr;
    sL = ticksL * MM_STEP;
    sR = ticksR * MM_STEP;
    psiTotal += (float)(dl + dr) * MM_STEP / track * (180.0f / PI);
    psi = wrap_angle(psiTotal);
    vL = (dl * MM_STEP) / dt;
    vR = (-dr * MM_STEP) / dt;
    sampleL = la;
    sampleR = ra;
    sampleMs = now;
}

void wait_motion(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        sample_pose();
        delay(1);
    }
}

bool health_check() {
    encoder_snapshot();
    bool okL = (statusL & 0x20) != 0;
    bool okR = (statusR & 0x20) != 0;
    bool weak = ((statusL | statusR) & 0x10) != 0;
    if (okL && okR && !weak)
        return true;
    if (okL && okR && weak) {
        oled.clearDisplay();
        put_text("MAGNET WEAK", 20, 0, 1, 1);
        put_text("L agc:" + String(agcL) + " mag:" + String(magL), 0, 14, 1, 1);
        put_text("R agc:" + String(agcR) + " mag:" + String(magR), 0, 24, 1, 1);
        put_text("Check encoder mounting", 0, 36, 1, 1);
        put_text("before driving", 0, 45, 1, 1);
        oled.display();
        sound_error();
        uint32_t t0 = millis();
        while (millis() - t0 < 2500 && digitalRead(BTN_LEFT) && digitalRead(BTN_RIGHT) &&
               digitalRead(BTN_CENTER))
            delay(20);
        return false;
    }
    oled.clearDisplay();
    put_text("ENCODER FAULT", 8, 0, 1, 1);
    put_text(String("L: ") + (okL ? "OK" : (statusL ? "NO MAGNET" : "NO REPLY")), 0, 14, 1, 1);
    put_text(String("R: ") + (okR ? "OK" : (statusR ? "NO MAGNET" : "NO REPLY")), 0, 26, 1, 1);
    put_text("Odometry unsafe", 0, 38, 1, 1);
    oled.fillRoundRect(0, 53, 128, 11, 3, 1);
    put_text(" any btn / 8s to skip ", 2, 55, 0, 1);
    oled.display();
    sound_error();
    uint32_t t0 = millis();
    while (millis() - t0 < 8000 && digitalRead(BTN_LEFT) && digitalRead(BTN_RIGHT) &&
           digitalRead(BTN_CENTER))
        delay(20);
    delay(300);
    return false;
}
