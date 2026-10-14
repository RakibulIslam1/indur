#include "15.h"
#include "16.h"
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
short int dp[6] = {0};
short int threshold[5] = {0};
short int maximum[5] = {0};
short int minimum[5] = {0};
short int sr = 0;
int tsc = 0;
int sensor = 0;
short int pwm = 0;
short int bias = 0;
short int baseL = 0;
short int baseR = 0;
short int count0 = 0;
short int reverseMs = 0;
short int brakeMs = 0;
short int turnPwm = 0;
short int turnMs = 0;
short int turnBrakeMs = 0;
short int resetMs = 0;
short int stopMs = 0;
short int nodeMs = 0;
short int inverseMs = 0;
float err0 = 0;
float err1 = 0;
float scale = 0;
float kp = 0;
float kd = 0;
float u = 0;
float kh = 0;
float arcZone = 0;
float diam = 0;
const float pitch = 180.0f;
char turn = 's';
uint32_t m1 = 0;
bool blocked = false;
bool turnFault = false;
uint32_t readErrors = 0;
uint32_t rejected = 0;
float kDrive = 0.0f;
float kRef = 0.0f;
bool lowPower = false;
uint8_t agcL = 0, agcR = 0;
uint16_t magL = 0, magR = 0;
uint8_t statusL = 0, statusR = 0;
const int irPins[NUM_SENSORS] = {6, 5, 4, 11, 10};
const int irEnable[NUM_SENSORS] = {7, 8, 8, 7, 1};
int s[NUM_SENSORS] = {0};
float d[NUM_SENSORS] = {0};
int w[NUM_SENSORS] = {0};
float a[NUM_SENSORS] = {-653.1188, -30997.72, -355129.8, -653.1188, -168702.9};
float b[NUM_SENSORS] = {-29.15369, -2976.892, -12626.86, -29.15369, -8425.789};
float c[NUM_SENSORS] = {9.142345, 13.00264, 30.39615, 9.142345, 22.84494};
int wallThr[NUM_SENSORS] = {1000, 600, 220, 600, 1000};
long ticksL = 0;
long ticksR = 0;
float sL = 0;
float sR = 0;
float vL = 0;
float vR = 0;
const int dim = 13;
int cx0 = 0;
int cy0 = 0;
int dir0 = DIR_N;
int wallRef = 205;
int tx0 = 0;
int ty0 = 0;
int targetMode = 0;
float track = WHEEL_BASE_MM;
uint8_t edges[16][16] = {};
uint8_t field[16][16] = {};
uint8_t seen[16][16] = {};
uint8_t quality[16][16] = {};
bool visited[16][16] = {};
uint8_t route[SOLVE_PATH_MAX] = {};
int routeLen = 0;
const float angles[4] = {0.0f, 90.0f, 180.0f, -90.0f};
const int8_t dx[4] = {0, 1, 0, -1};
const int8_t dy[4] = {1, 0, -1, 0};
const uint8_t edgeMask[4] = {WALL_N, WALL_E, WALL_S, WALL_W};
bool debug = false;
byte latchedButton = 0;
const char *resetCause = "?";
String labels0[31] = {"",  "A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
                      "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U",
                      "V", "W", "X", "Y", "Z", "a", "b", "c", "d"};

static const char *reset_name() {
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "POWERON";
    case ESP_RST_EXT:
        return "EXT-RESET";
    case ESP_RST_SW:
        return "SW-RESTART";
    case ESP_RST_PANIC:
        return "PANIC/EXCEPTION";
    case ESP_RST_INT_WDT:
        return "INT-WATCHDOG";
    case ESP_RST_TASK_WDT:
        return "TASK-WATCHDOG";
    case ESP_RST_WDT:
        return "OTHER-WATCHDOG";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT";
    case ESP_RST_SDIO:
        return "SDIO";
    case ESP_RST_DEEPSLEEP:
        return "DEEPSLEEP";
    default:
        break;
    }
    static char buf[24];
    snprintf(buf, sizeof(buf), "code-%d", (int)esp_reset_reason());
    return buf;
}

void setup() {
    Serial.begin(115200);
    resetCause = reset_name();
    EEPROM.begin(EEPROM_SIZE);
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire1.begin(ENC_R_SDA, ENC_R_SCL);
    Wire.setClock(I2C_HZ);
    Wire1.setClock(I2C_HZ);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(irEnable[i], OUTPUT);
        digitalWrite(irEnable[i], LOW);
    }
    ledcSetup(CHAN_LF, MOTOR_FREQ, MOTOR_RES);
    ledcAttachPin(LMF, CHAN_LF);
    ledcSetup(CHAN_LB, MOTOR_FREQ, MOTOR_RES);
    ledcAttachPin(LMB, CHAN_LB);
    ledcSetup(CHAN_RF, MOTOR_FREQ, MOTOR_RES);
    ledcAttachPin(RMF, CHAN_RF);
    ledcSetup(CHAN_RB, MOTOR_FREQ, MOTOR_RES);
    ledcAttachPin(RMB, CHAN_RB);
    drive(0, 0);
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_CENTER, INPUT_PULLUP);
    bool oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (!oledOk && debug)
        debugln("[BOOT] OLED not found at 0x3C");
    settings_load();
    sound_init();
    encoder_config(Wire);
    encoder_config(Wire1);
    encoder_init();
    health_check();
    if (esp_reset_reason() != ESP_RST_POWERON && esp_reset_reason() != ESP_RST_EXT) {
        oled.clearDisplay();
        put_text("REBOOTED BY", 22, 8, 1, 1);
        put_text(resetCause, 4, 24, 1, 1);
        put_text("Check power and wiring", 4, 44, 1, 1);
        oled.display();
        sound_error();
        delay(2500);
    }
    splash();
}

void loop() {
    home();
}
