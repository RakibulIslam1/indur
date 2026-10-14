#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <math.h>
#define LMF 16
#define LMB 35
#define RMF 15
#define RMB 38
#define MOTOR_FREQ 1000
#define MOTOR_RES 8
#define CHAN_LF 0
#define CHAN_LB 1
#define CHAN_RF 2
#define CHAN_RB 3
#define NUM_SENSORS 5
#define SENS_L 0
#define SENS_F 2
#define SENS_R 4
#define MOTOR_MAX 255
#define BTN_LEFT 33
#define BTN_RIGHT 37
#define BTN_CENTER 18
#define LONG_PRESS_MS 300
#define BTN_STUCK_MS 4000
#define OLED_SDA 3
#define OLED_SCL 2
#define ENC_R_SDA 13
#define ENC_R_SCL 12
#define I2C_HZ 400000
#define AS5600_ADDR 0x36
#define AS5600_REG_H 0x0C
#define AS5600_REG_ST 0x0B
#define AS5600_REG_CONF 0x07
#define AS5600_REG_AGC 0x1A
#define AS5600_REG_MAG 0x1B
#define AS5600_BAD 0xFFFF
#define AS5600_HYST_2LSB 0x08
#define ENC_CPR 4096
#define ENC_MAX_MM_S 2000.0f
#define WHEEL_BASE_MM 63.0f
#define CELL_SIZE_MM 180.0f
#define DECEL_MM 60.0f
#define MIN_MOVE_SPEED 40
#define TURN_MIN_SPEED 50
#define CTRL_PERIOD_MS 6
#define PID_AUTHORITY_DIV 4
#define CELL_ENTRY_MM 50.0f
#define WALL_VALID_MIN 50
#define WALL_VALID_MAX_F 2.0f
#define WALL_EMA_ALPHA 0.20f
#define POST_TURN_SETTLE_MS 100
#define STALL_TIMEOUT_MS 400
#define STALL_MIN_MM 2.0f
#define BRAKE_ADVANCE_MM 6.0f
#define MAX_HEADING_DEG 40.0f
#define TURN_TIMEOUT_MS 2500
#define BUZZER_PIN 9
#define CHAN_BUZZ 4
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3
#define WALL_N 0x01
#define WALL_E 0x02
#define WALL_S 0x04
#define WALL_W 0x08
#define EEPROM_SIZE 1024
#define MOTOR_K_EEPROM 17
#define GOAL_X_EEPROM 18
#define GOAL_Y_EEPROM 19
#define GOAL_MODE_EEPROM 20
#define WALL_THR_L_EEPROM 15
#define WALL_THR_R_EEPROM 16
#define MAZE_MAGIC_ADDR 49
#define MAZE_MAGIC 0xA5
#define MAZE_SIZE_ADDR 50
#define MAZE_WALLS_EEPROM 51
#define PATH_EEPROM_BASE 307
#define SOLVE_PATH_MAX 200
#define PATH_LEN_ADDR 508
#define MAZE_KNOWN_EEPROM 512
#define MAZE_CRC_ADDR 510
extern Adafruit_SSD1306 oled;
extern short int dp[6];
extern short int threshold[5], maximum[5], minimum[5];
extern short int pwm, bias, baseL, baseR;
extern short int count0, reverseMs, brakeMs, turnPwm, turnMs, turnBrakeMs;
extern short int resetMs, stopMs, nodeMs, inverseMs, sr;
extern int tsc, sensor;
extern float err0, err1, scale, kp, kd, u;
extern float kh;
extern float arcZone;
extern float diam;
extern const float pitch;
extern char turn;
extern uint32_t m1;
extern bool debug;
extern byte latchedButton;
extern const char *resetCause;
extern bool blocked;
extern bool turnFault;
extern uint32_t readErrors;
extern uint32_t rejected;
extern float kDrive;
extern float kRef;
extern bool lowPower;
extern uint8_t agcL, agcR;
extern uint16_t magL, magR;
extern uint8_t statusL, statusR;
extern String labels0[31];
extern const int irPins[NUM_SENSORS];
extern const int irEnable[NUM_SENSORS];
extern int s[NUM_SENSORS];
extern float d[NUM_SENSORS];
extern int w[NUM_SENSORS];
extern float a[NUM_SENSORS], b[NUM_SENSORS], c[NUM_SENSORS];
extern int wallThr[NUM_SENSORS];
extern long ticksL, ticksR;
extern float sL, sR;
extern float vL, vR;
extern float psi;
extern const int dim;
extern int cx0, cy0, dir0;
extern int wallRef;
extern int tx0, ty0, targetMode;
extern float track;
extern uint8_t edges[16][16];
extern uint8_t field[16][16];
extern uint8_t seen[16][16];
extern uint8_t quality[16][16];
extern bool visited[16][16];
extern uint8_t route[SOLVE_PATH_MAX];
extern int routeLen;
extern const float angles[4];
extern const int8_t dx[4], dy[4];
extern const uint8_t edgeMask[4];
void debugf(const char *fmt, ...);
void debugln(const char *s);

static inline float wrap_angle(float v) {
    while (v > 180.0f)
        v -= 360.0f;
    while (v < -180.0f)
        v += 360.0f;
    return v;
}

static inline int origin_cell() {
    return 0;
}

byte button(byte btn);
bool held(byte btn, bool indicator);
void console_poll();
void put_text(String t, byte x, byte y, byte col, byte sz);
void splash();
void home();
String menu_item(byte a);
byte menu();
String setting_name(byte a);
void settings();
void setting_edit(int indicator);
uint16_t setting_addr(byte indicator);
void drive(int pwmL, int pwmR);
int trim_left(int v);
int trim_right(int v);
void drive_test();
void drive_mm(float targetMm);
void arc_turn(int dir, int runSpeed);
void rotate(int dir);
void ir_read(int index);
float ir_range(int index);
bool ir_wall(int index);
void ir_sample();
void raw_view();
void range_view();
void sound_init();
void sound_start();
void sound_goal();
void sound_home();
void sound_phase(int n);
void sound_error();
void sound_done();
void encoder_init();
uint16_t encoder_read(TwoWire &wire);
uint8_t encoder_status(TwoWire &wire);
uint8_t reg_read8(TwoWire &wire, uint8_t reg);
uint16_t reg_read16(TwoWire &wire, uint8_t reg);
void encoder_config(TwoWire &wire);
void encoder_snapshot();
void sample_pose();
void pose_zero();
void distance_zero();
void wait_motion(uint32_t ms);
bool health_check();
void settings_load();
void follow();
void map_init();
void target_box(int &x0, int &y0, int &x1, int &y1);
void propagate();
void propagate_to(int gx, int gy);
void map_sample(int x, int y);
int choose_dir();
void face(int newDir);
void advance();
void trace(int sx, int sy);
bool route_valid();
void search();
void follow_hand(int rule);
void run_saved();
void run_fast();
void run_direct(bool autoModerate = false);
void map_save();
bool map_load();
void map_clear();

static inline void sensor_dirs(int out[3]) {
    out[0] = (dir0 + 3) & 3;
    out[1] = dir0;
    out[2] = (dir0 + 1) & 3;
}

static String setting_value(byte a, int raw);
