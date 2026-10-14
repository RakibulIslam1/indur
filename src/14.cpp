#include "15.h"

void settings_load() {
    for (byte i = 0; i < 5; i++) {
        threshold[i] = EEPROM.read(i) * 16;
        maximum[i] = EEPROM.read(i + 5) * 16;
        minimum[i] = EEPROM.read(i + 10) * 16;
    }
    count0 = EEPROM.read(24);
    pwm = EEPROM.read(25) * 10;
    bias = (EEPROM.read(26) - 25) * 2;
    turnPwm = EEPROM.read(27) * 10;
    brakeMs = EEPROM.read(28) * 10;
    turnBrakeMs = EEPROM.read(29) * 10;
    turnMs = EEPROM.read(30) * 10;
    reverseMs = EEPROM.read(31) * 10;
    resetMs = EEPROM.read(32) * 10;
    stopMs = EEPROM.read(33) * 10;
    nodeMs = EEPROM.read(34);
    inverseMs = EEPROM.read(35);
    kp = EEPROM.read(36);
    kd = EEPROM.read(37) * 10;
    scale = (float)EEPROM.read(38) / 10.0f;
    if (pwm <= 0 || pwm > 250)
        pwm = 150;
    if (turnPwm <= 0 || turnPwm > 250)
        turnPwm = 100;
    if (brakeMs < 0 || brakeMs > 500)
        brakeMs = 50;
    if (turnBrakeMs < 0 || turnBrakeMs > 500)
        turnBrakeMs = 40;
    if (kp <= 0 || kp > 100)
        kp = 20;
    if (kd < 0 || kd > 500)
        kd = 50;
    if (scale <= 0.0f || scale > 5.0f)
        scale = 1.0f;
    if (bias < -50 || bias > 50)
        bias = 0;
    track = (float)EEPROM.read(39);
    if (track < 40.0f || track > 120.0f)
        track = WHEEL_BASE_MM;
    wallRef = (int)EEPROM.read(41) * 16;
    if (wallRef <= 0 || wallRef > 1500)
        wallRef = 205;
    kh = (float)EEPROM.read(42);
    if (kh < 0 || kh > 100)
        kh = 30.0f;
    arcZone = (float)EEPROM.read(43);
    if (arcZone < 5.0f || arcZone > 100.0f)
        arcZone = 30.0f;
    diam = (float)EEPROM.read(44) * 0.5f;
    if (diam < 20.0f || diam > 60.0f)
        diam = 35.5f;
    wallThr[SENS_F] = (int)EEPROM.read(46);
    if (wallThr[SENS_F] < 10 || wallThr[SENS_F] > 250)
        wallThr[SENS_F] = 220;
    wallThr[SENS_L] = (int)EEPROM.read(WALL_THR_L_EEPROM) * 16;
    if (wallThr[SENS_L] < 100 || wallThr[SENS_L] > 4000)
        wallThr[SENS_L] = 1000;
    wallThr[SENS_R] = (int)EEPROM.read(WALL_THR_R_EEPROM) * 16;
    if (wallThr[SENS_R] < 100 || wallThr[SENS_R] > 4000)
        wallThr[SENS_R] = 1000;
    {
        uint8_t rawK = EEPROM.read(MOTOR_K_EEPROM);
        kRef = (rawK == 0x00 || rawK == 0xFF) ? 0.0f : (float)rawK / 100.0f;
        if (kRef < 0.20f)
            kRef = 0.0f;
    }
    tx0 = (int)EEPROM.read(GOAL_X_EEPROM);
    ty0 = (int)EEPROM.read(GOAL_Y_EEPROM);
    targetMode = (int)EEPROM.read(GOAL_MODE_EEPROM);
    if (tx0 < 0 || tx0 >= dim)
        tx0 = 0;
    if (ty0 < 0 || ty0 >= dim)
        ty0 = 0;
    if (targetMode > 2)
        targetMode = 0;
    baseL = pwm;
    baseR = pwm;
}
