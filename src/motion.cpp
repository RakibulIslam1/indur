#include "config.h"

int trim_left(int v) {
    return constrain((bias > 0) ? v - bias : v, 0, 255);
}

int trim_right(int v) {
    return constrain((bias < 0) ? v + bias : v, 0, 255);
}

void drive(int a, int b) {
    if (a > 0) {
        ledcWrite(CHAN_LF, abs(a));
        ledcWrite(CHAN_LB, 0);
    } else if (a < 0) {
        ledcWrite(CHAN_LF, 0);
        ledcWrite(CHAN_LB, abs(a));
    } else {
        ledcWrite(CHAN_LF, 0);
        ledcWrite(CHAN_LB, 0);
    }
    if (b > 0) {
        ledcWrite(CHAN_RF, abs(b));
        ledcWrite(CHAN_RB, 0);
    } else if (b < 0) {
        ledcWrite(CHAN_RF, 0);
        ledcWrite(CHAN_RB, abs(b));
    } else {
        ledcWrite(CHAN_RF, 0);
        ledcWrite(CHAN_RB, 0);
    }
}

void drive_test() {
    int target_spd = 0;
    int temp = -1, limit = 255;
    int l_sp = 0, r_sp = 0;
    while (1) {
        sample_pose();
        if (temp != target_spd) {
            oled.clearDisplay();
            for (byte i = 0; i < 3; i++) {
                (i < 2) ? oled.drawRoundRect(41 * i, 0, 40, 11, 3, 1)
                        : oled.drawRoundRect(41 * i, 0, 46, 11, 3, 1);
            }
            put_text("CT-" + String(count0), 2, 2, 1, 1);
            put_text("SP-" + String(pwm), 43, 2, 1, 1);
            put_text("CP-" + String(scale), 84, 2, 1, 1);
            oled.drawRoundRect(0, 15, 128, 37, 3, 1);
            oled.fillRoundRect(0, 53, 40, 11, 3, 1);
            oled.fillRoundRect(88, 53, 40, 11, 3, 1);
            put_text(" BACK ", 2, 55, 0, 1);
            put_text("START ", 90, 55, 0, 1);
            temp = target_spd;
            put_text("Target speed: " + String(target_spd) + "  ", 4, 30, 1, 1);
            oled.display();
        }
        delay(5);
        byte p = button(BTN_LEFT);
        if (p == 1) {
            target_spd -= 5;
            if (target_spd < 0)
                target_spd = 0;
        } else if (p == 2) {
            drive(0, 0);
            return;
        }
        p = button(BTN_RIGHT);
        if (p == 1) {
            target_spd += 5;
            if (target_spd > limit)
                target_spd = limit;
        } else if (p == 2) {
            l_sp = (bias > 0) ? target_spd - bias : target_spd;
            r_sp = (bias < 0) ? target_spd + bias : target_spd;
            drive(constrain(l_sp, 0, 255), constrain(r_sp, 0, 255));
        }
    }
}

void drive_mm(float targetMm) {
    blocked = false;
    sample_pose();
    float startL = sL;
    float startR = sR;
    err1 = 0;
    u = 0;
    float psi0 = psi;
    ir_sample();
    float f_s0 = (float)s[SENS_L];
    float f_s4 = (float)s[SENS_R];
    float stallRefMm = 0.0f;
    uint32_t stallTimer = millis();
    int stallTries = 0;
    float stopMm = (targetMm >= DECEL_MM) ? targetMm - BRAKE_ADVANCE_MM : targetMm;
    uint32_t lastCtrl = millis();
    static uint32_t dbgTimer = 0;
    while (1) {
        uint32_t nowCtrl;
        do {
            sample_pose();
            delay(1);
            nowCtrl = millis();
        } while (nowCtrl - lastCtrl < (uint32_t)CTRL_PERIOD_MS);
        float ctrl_dt = (nowCtrl - lastCtrl) * 0.001f;
        lastCtrl = nowCtrl;
        ir_sample();
        float traveledL = sL - startL;
        float traveledR = sR - startR;
        float traveled = (traveledL + traveledR) * 0.5f;
        if (traveled >= stopMm)
            break;
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            blocked = true;
            drive(0, 0);
            wait_motion(200);
            return;
        }
        if (traveled - stallRefMm > STALL_MIN_MM) {
            stallRefMm = traveled;
            stallTimer = millis();
            stallTries = 0;
        } else if (millis() - stallTimer > (uint32_t)STALL_TIMEOUT_MS) {
            if (stallTries++ == 0) {
                int kick = constrain((int)pwm + 50, 0, 255);
                drive(trim_left(kick), trim_right(kick));
                wait_motion(80);
                drive(0, 0);
                wait_motion(20);
                stallTimer = millis();
                stallRefMm = traveled;
            } else {
                drive(0, 0);
                blocked = true;
                return;
            }
        }
        f_s0 = (1.0f - WALL_EMA_ALPHA) * f_s0 + WALL_EMA_ALPHA * (float)s[SENS_L];
        f_s4 = (1.0f - WALL_EMA_ALPHA) * f_s4 + WALL_EMA_ALPHA * (float)s[SENS_R];
        float wallMax = (float)wallRef * WALL_VALID_MAX_F;
        bool leftSide = (traveled >= 10.0f && w[SENS_L] && f_s0 > WALL_VALID_MIN && f_s0 < wallMax);
        bool rightSide =
            (traveled >= 10.0f && w[SENS_R] && f_s4 > WALL_VALID_MIN && f_s4 < wallMax);
        float pos_err = 0.0f;
        bool wallMode = false;
        if (leftSide) {
            pos_err = (float)wallRef - f_s0;
            wallMode = true;
        } else if (rightSide) {
            pos_err = f_s4 - (float)wallRef;
            wallMode = true;
        }
        pos_err = constrain(pos_err, -100.0f, 100.0f);
        if (wallMode) {
            float deriv = (pos_err - err1) * ((float)CTRL_PERIOD_MS * 0.001f / ctrl_dt);
            u = (kp / 100.0f) * pos_err + (kd / 1000.0f) * deriv;
            err1 = pos_err;
        } else {
            float epsi = constrain(wrap_angle(psi0 - psi), -15.0f, 15.0f);
            u = (kh / 100.0f) * epsi;
            err1 = 0.0f;
        }
        u = constrain(u, -(float)pwm / PID_AUTHORITY_DIV, (float)pwm / PID_AUTHORITY_DIV);
        {
            float dpsi = wrap_angle(psi - psi0);
            if (dpsi > MAX_HEADING_DEG && u > 0)
                u = 0.0f;
            else if (dpsi < -MAX_HEADING_DEG && u < 0)
                u = 0.0f;
        }
        float remaining = targetMm - traveled;
        float decel_scale = (remaining < DECEL_MM) ? (remaining / (float)DECEL_MM) : 1.0f;
        int base = max(MIN_MOVE_SPEED, (int)(pwm * decel_scale));
        int pid_int = (int)u;
        int lOut = trim_left(base);
        int rOut = trim_right(base);
        drive(constrain(lOut + pid_int, 0, 255), constrain(rOut - pid_int, 0, 255));
        if (decel_scale >= 0.999f && base > 60) {
            float meas = (fabsf(vL) + fabsf(vR)) * 0.5f;
            if (meas > 1.0f) {
                float k = meas / (float)base;
                kDrive = (kDrive <= 0.0f) ? k : (0.98f * kDrive + 0.02f * k);
                lowPower = (kRef > 0.2f && kDrive < 0.65f * kRef);
            }
        }
        if (debug && millis() - dbgTimer > 100) {
            dbgTimer = millis();
            debugf("[MOVE] %.0fmm %s err:%.1f pid:%.0f hdg:%.1f L:%.0f R:%.0f\n", traveled,
                   wallMode ? (leftSide ? "LWAL" : "RWAL") : "ENC ", pos_err, u, psi, f_s0, f_s4);
        }
    }
    drive(0, 0);
    wait_motion(brakeMs);
}

void arc_turn(int dir, int runSpeed) {
    turnFault = false;
    sample_pose();
    float targetHdg = wrap_angle(psi + (dir == 1 ? 90.0f : -90.0f));
    int outer = constrain(runSpeed, TURN_MIN_SPEED, 255);
    int inner = constrain((int)(runSpeed * 0.25f), 0, 80);
    int ol = trim_left(outer), or_ = trim_right(outer);
    int il = trim_left(inner), ir = trim_right(inner);
    uint32_t timeout = millis() + 1500;
    bool reached = false;
    while (millis() < timeout) {
        sample_pose();
        delay(1);
        if (fabsf(wrap_angle(targetHdg - psi)) < 3.0f) {
            reached = true;
            break;
        }
        if (dir == 1)
            drive(ol, ir);
        else
            drive(il, or_);
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            turnFault = true;
            drive(0, 0);
            wait_motion(200);
            return;
        }
    }
    drive(0, 0);
    wait_motion(50);
    if (!reached)
        turnFault = true;
}

void rotate(int dir) {
    turnFault = false;
    sample_pose();
    float targetDeg = psi;
    if (dir == 1)
        targetDeg += 90.0f;
    else if (dir == 0)
        targetDeg -= 90.0f;
    else
        targetDeg += 180.0f;
    targetDeg = wrap_angle(targetDeg);
    float decelZone = (arcZone < 5.0f) ? 30.0f : arcZone;
    uint32_t timeout = millis() + TURN_TIMEOUT_MS;
    bool reached = false;
    int driveDir = 0;
    while (millis() < timeout) {
        sample_pose();
        delay(1);
        float err = wrap_angle(targetDeg - psi);
        if (fabsf(err) < 2.5f) {
            reached = true;
            break;
        }
        float absErr = fabsf(err);
        int spd = (absErr < decelZone) ? max(TURN_MIN_SPEED, (int)(turnPwm * absErr / decelZone))
                                       : turnPwm;
        driveDir = (err > 0) ? 1 : -1;
        if (err > 0)
            drive(trim_left(spd), -trim_right(spd));
        else
            drive(-trim_left(spd), trim_right(spd));
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            turnFault = true;
            drive(0, 0);
            wait_motion(200);
            return;
        }
    }
    if (driveDir > 0)
        drive(-trim_left(80), trim_right(80));
    else if (driveDir)
        drive(trim_left(80), -trim_right(80));
    wait_motion(turnBrakeMs);
    drive(0, 0);
    wait_motion(POST_TURN_SETTLE_MS);
    if (!reached)
        turnFault = true;
    if (debug) {
        debugf("[TURN] dir:%d tgt:%.1f act:%.1f err:%.1f %s\n", dir, targetDeg, psi,
               wrap_angle(targetDeg - psi), reached ? "ok" : "FAILED");
    }
}
