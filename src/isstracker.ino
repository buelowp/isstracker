#include "Adafruit_mfGFX.h"
#include "Adafruit_SSD1351_Photon.h"

#define APP_ID              51

#define cs                  A5
#define rst                 A3
#define dc                  A4
#define DISTANCE            A2

#define mtr_ms2             D0
#define mtr_en              D1
#define mtr_dir             D2
#define mtr_step            D3
#define mtr_ms1             D4
#define mtr_slp             D5
#define SERVO               D6
#define AZIMUTH             D7
#define GLOBE_POWER         D8

#define FULL_STEP           1
#define HALF_STEP           2
#define QUARTER_STEP        3
#define EIGHTH_STEP         4

#define CLOCKWISE           1
#define CCLOCKWISE          2

#define INC_CAL_ADDRESS     0

#define ONE_SECOND          1000
#define FIVE_SECONDS        (ONE_SECOND * 5)
#define TEN_SECONDS         (ONE_SECOND * 10)
#define TWENTY_SECONDS      (ONE_SECOND * 20)
#define ONE_MINUTE          (ONE_SECOND * 60)
#define FIVE_MINUTES        (ONE_MINUTE * 5)
#define TEN_MINUTES         (ONE_MINUTE * 10)
#define THIRTY_MINUTES      (ONE_MINUTE * 30)

#define ISS_BLACK           0x0000
#define ISS_BLUE            0x001F
#define ISS_RED             0xF800
#define ISS_GREEN           0x07E0
#define ISS_CYAN            0x07FF
#define ISS_MAGENTA         0xF81F
#define ISS_YELLOW          0xFFE0  
#define ISS_WHITE           0xFFFF

Timer issUpdate(5000, run_location_update);
SerialLogHandler logHandler;
Serial1LogHandler logHandler2(115200);

Servo servo;
double g_longitude;
double g_latitude;
int g_lon;
int g_lat;
int g_appId;
int g_azimuthPosition;
int g_incOffset;
int g_currentResolution;
int g_motorAngle;
int g_servoAngle;
int g_motorDirection;
int g_lastResetReason;
int g_displayTimeout;
int g_globeTimeout;
int g_distance;
int g_proximity;
system_tick_t g_displayTimeoutMillis;
system_tick_t g_globeTimeoutMillis;
system_tick_t g_dutyCycleTimeout;
system_tick_t g_dutyCycleRestart;
bool g_querySuccess;
bool g_runLocationQuery;
bool g_inCalibration;
bool g_motorDecCalibrated;
bool g_motorHome;
bool g_inclineHome;
bool g_displayEnabled;
bool g_cycleColors;
bool g_globeEnabled;
bool g_globePowerControl;
String g_version = System.version() + "." + APP_ID;

Adafruit_SSD1351 display = Adafruit_SSD1351(cs, dc, rst);

void set_motor_sleep(bool slp)
{
    if (slp) {
        Log.info("%s: Putting motor to sleep", __FUNCTION__);
        digitalWrite(mtr_slp, LOW);
        delay(250);
    }
    else {
        Log.info("%s: Waking motor up", __FUNCTION__);
        digitalWrite(mtr_slp, HIGH);
        delay(250);
    }
}

void set_motor_home()
{
    set_motor_step_resolution(FULL_STEP);
    set_motor_enabled(true);
    set_motor_sleep(false);

    // If we seem like we are home, make a
    // quarter turn so we can rotate and hit
    // the leading edge of the sensor to be consistent
    Log.info("%s: Moving motor away from sensor...", __FUNCTION__);
    set_motor_dir(CLOCKWISE);
    if (digitalRead(AZIMUTH) == LOW) {
        for (int i = 0; i < 50; i++) {
            digitalWrite(mtr_step, HIGH);
            delay(1);
            digitalWrite(mtr_step, LOW);
            delay(1);
        }
    }

    // This will be slow!
    Log.info("%s: Moving motor back to home!", __FUNCTION__);
    set_motor_dir(CCLOCKWISE);
    for (int i = 0; i < 200; i++) {
        digitalWrite(mtr_step, HIGH);
        delay(1);
        digitalWrite(mtr_step, LOW);
        delay(200);
        if (digitalRead(AZIMUTH) == LOW) {
            break;
        }
        Particle.process();
    }
    set_motor_sleep(true);
    g_azimuthPosition = 0;
    g_motorHome = true;
}

void reset_motor()
{
    set_motor_sleep(false);
    set_motor_enabled(true);
    set_motor_dir(CLOCKWISE);
    set_motor_step_resolution(FULL_STEP);
    set_motor_home();
    set_motor_sleep(true);
    Log.info("%s: motor reset complete", __FUNCTION__);
}

void set_motor_dir(int dir)
{
    switch (dir) {
        case CCLOCKWISE:
            digitalWrite(mtr_dir, HIGH);
            g_motorDirection = CCLOCKWISE;
            Log.info("%s: Set motor direction to clockwise", __FUNCTION__);
            break;
        case CLOCKWISE:
        default:
            digitalWrite(mtr_dir, LOW);
            g_motorDirection = CLOCKWISE;
            Log.info("%s: Set motor direction to counter clockwise", __FUNCTION__);
            break;
    }
}

void set_motor_step_resolution(int step)
{
    switch (step) {
        case HALF_STEP:
            digitalWrite(mtr_ms1, HIGH);
            digitalWrite(mtr_ms2, LOW);
            g_currentResolution = 400;
            Log.info("%s: Set stepper resolution to half steps (400)", __FUNCTION__);
            break;
        case QUARTER_STEP:
            digitalWrite(mtr_ms1, LOW);
            digitalWrite(mtr_ms2, HIGH);
            g_currentResolution = 800;
            Log.info("%s: Set stepper resolution to quarter steps (800)", __FUNCTION__);
            break;
        case EIGHTH_STEP:
            digitalWrite(mtr_ms1, HIGH);
            digitalWrite(mtr_ms2, HIGH);
            g_currentResolution = 1600;
            Log.info("%s: Set stepper resolution to eighth steps (1600)", __FUNCTION__);
            break;
        case FULL_STEP:
        default:
            digitalWrite(mtr_ms1, LOW);
            digitalWrite(mtr_ms2, LOW);
            g_currentResolution = 200;
            Log.info("%s: Set stepper resolution to single steps (200)", __FUNCTION__);
            break;
    }
}

void set_motor_enabled(bool en)
{
    if (en) {
        digitalWrite(mtr_en, LOW);
        Log.info("%s: motor is disabled", __FUNCTION__);
    }
    else {
        digitalWrite(mtr_en, HIGH);
        Log.info("%s: motor is enabled", __FUNCTION__);
    }
}
 
 /**
  * When we need to move the motor, turn the outputs
  * on which is what SLP going HIGH does. This should
  * help keep the motor cooler during operation.
  */
void set_motor_position(int angle)
{
    g_motorAngle = angle;

    if ((g_azimuthPosition == g_currentResolution - 1) && angle >= 0) {
        g_azimuthPosition = -1;
    }
    int steps = angle - g_azimuthPosition;
    if (steps > 0) {
        set_motor_sleep(false);
        Log.info("%s: Moving motor %d steps", __FUNCTION__, steps);
        g_motorHome = false;
    }
    for (int i = 0; i < steps; i++) {
        digitalWrite(mtr_step, HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(mtr_step, LOW); //Pull step pin low so it can be triggered again
        delay(1);
        g_azimuthPosition++;
    }

    if (steps > 0)
        set_motor_sleep(true);
}

void iss_location(const char *event, const char *data) 
{
    JSONValue outerObj = JSONValue::parseCopy(data);

    g_querySuccess = false;

    JSONObjectIterator iter(outerObj);
    while (iter.next()) {
        if (iter.value().isObject()) {
            JSONObjectIterator location(iter.value());
            while (location.next()) {
                if (location.name() == "longitude") {
                    g_longitude = location.value().toDouble();
                    g_lon = 180 - g_longitude;
                }
                if (location.name() == "latitude") {
                    g_latitude = location.value().toDouble();
                    set_servo_angle();
                }
            }
        }
        else {
            if (iter.name() == "message") {
                if (iter.value().toString() == "success") {
                    g_querySuccess = true;
                }
            }
        }
    }

    if (g_querySuccess) {
        set_inclination();
        set_azimuth();
        Log.info("%s: %f, %f, inclination %d, azimuth %d, motor position %d", __FUNCTION__, g_latitude, g_longitude, g_servoAngle, g_motorAngle, g_azimuthPosition);
    }
    else {
        Log.info("JSON did not work right");
    }
}

int set_servo_angle()
{
    int angle = static_cast<int>(g_latitude + .5);
    angle += 90;
    angle += g_incOffset;
    g_servoAngle = angle;
    Log.info("%s: Angle set to %d", __FUNCTION__, angle);
    return angle;
}

void run_location_update()
{
    if (!g_inCalibration)
        g_runLocationQuery = true;
}

void set_inclination(int angle)
{
    if (angle > 20 && angle < 160) {
        servo.write(angle);
        g_servoAngle = angle;
    }
    if (g_servoAngle == 90)
        g_inclineHome = true;
    
    Log.info("%s: Inclination set to %d", __FUNCTION__, g_servoAngle);
}

void set_inclination()
{
    static int lastAngle = 200;

    if (g_inCalibration)
        return;

    if (lastAngle != g_servoAngle) {
        if (g_globeEnabled)
            servo.write(g_servoAngle);

        lastAngle = g_servoAngle;
    }
    Log.info("%s: Inclination set to %d", __FUNCTION__, g_servoAngle);
}

void set_azimuth()
{  
    int angle = 0;
    int degrees = 0;

    if (g_inCalibration)
        return;

    degrees = static_cast<int>(g_longitude * 10);
    angle = map(degrees, 0, 3600, 0, g_currentResolution);

    if (angle < 0)
        angle = g_currentResolution + angle;

    set_motor_position(angle);
}

int web_calibrate(String p)
{    
    if (p.toInt() != 0) {
        digitalWrite(GLOBE_POWER, HIGH);
        g_inCalibration = true;
        Log.info("%s: starting calibration", __FUNCTION__);
    }
    else {
        g_inCalibration = false;
        Log.info("%s: ending calibration", __FUNCTION__);
    }

    return p.toInt();
}

int web_rotate_clockwise(String p)
{
    int steps = p.toInt();
    steps *= g_currentResolution;

    if (!g_inCalibration)
        return -1;

    if (steps >= 0 && steps < 360) {
        Log.info("%s: Moving azimuth motor %d steps clockwise", __FUNCTION__, steps);
        set_motor_dir(CLOCKWISE);
        set_motor_position(steps);
        return steps;
    }

    return 0;
}

int web_rotate_cclockwise(String p)
{
    int steps = p.toInt();
    steps *= g_currentResolution;

    if (!g_inCalibration)
        return -1;

    if (steps > 0 && steps < 180) {
        Log.info("%s: Moving azimuth motor %d steps counter clockwise", __FUNCTION__, steps);
        set_motor_dir(CCLOCKWISE);
        set_motor_position(steps);
        return steps;
    }

    return 0;
}

int web_set_motor_resolution(String p)
{
    int res = p.toInt();

    set_motor_step_resolution(res);
    return g_currentResolution;
}

int web_jog_inclination(String p)
{
    int jog = p.toInt();

    if (g_inCalibration && g_inclineHome) {
        if (jog > 0) {
            g_servoAngle++;
            servo.write(g_servoAngle);
            g_incOffset++;
        }
        if (jog < 0) {
            g_servoAngle--;
            servo.write(g_servoAngle);
            g_incOffset--;
        }
        EEPROM.put(INC_CAL_ADDRESS, g_incOffset);
    }

    Log.info("%s: Inclination offset is now %d", __FUNCTION__, g_incOffset);
    return g_incOffset;
}

int web_set_incline_offset(String p)
{
    int offset = p.toInt();

    if (offset > -11 && offset < 11) {
        g_incOffset = offset;
        EEPROM.put(INC_CAL_ADDRESS, offset);
    }

    Log.info("%s: Inclination offset set to %d", __FUNCTION__, g_incOffset);
    return offset;
}

int web_set_display_timeout(String p)
{
    int timeout = p.toInt() * 1000;

    if (timeout >= TEN_SECONDS && timeout <= TEN_MINUTES) {
        g_displayTimeout = timeout;
        return timeout;
    }
    return g_displayTimeout;
}

void display_update()
{
    if (!g_displayEnabled) {
        return;
    }

    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(ISS_WHITE, ISS_BLACK);
    display.printf("Version: %s\n\n", g_version.c_str());
    display.setTextColor(ISS_CYAN, ISS_BLACK);
    display.printf("ISS Location\n\n");
    if (g_latitude < 0) {
        display.setTextColor(ISS_BLUE, ISS_BLACK);
        display.printf("Lat: %10.06f  \n", g_latitude);
    }
    else {
        display.setTextColor(ISS_GREEN, ISS_BLACK);
        display.printf("Lat: %10.06f  \n", g_latitude);
    }
    if (g_longitude < 0) {
        display.setTextColor(ISS_BLUE, ISS_BLACK);
        display.printf("Lon: %10.06f  \n", g_longitude);
    }
    else {
        display.setTextColor(ISS_GREEN, ISS_BLACK);
        display.printf("Lon: %10.06f  \n", g_longitude);
    }
//    Log.info("%s: Lat: %f, Lon: %f", __FUNCTION__, g_latitude, g_longitude);
}

bool detect_motion()
{
    if (digitalRead(DISTANCE) == HIGH) {
        if (g_displayTimeoutMillis < millis()) {
            g_displayEnabled = true;
            g_displayTimeoutMillis = millis() + g_displayTimeout;
            return true;
        }
        if (g_globeTimeoutMillis < millis() && g_globePowerControl) {
            g_globeEnabled = true;
            g_globeTimeoutMillis = millis() + g_globeTimeout;
            digitalWrite(GLOBE_POWER, HIGH);
        }
    }
    else {
        if (g_displayTimeoutMillis < millis()) {
            g_displayEnabled = false;
            display.fillScreen(0);
        }
        if (g_globeTimeoutMillis < millis() && g_globePowerControl) {
            g_globeEnabled = false;
            digitalWrite(GLOBE_POWER, LOW);
        }
    }
    return false;
}

int web_set_proximity_distance(String p)
{
    int distance = p.toInt();
    if (distance > 12 && distance < 150) {
        g_proximity = distance;
    }
    return g_proximity;
}

int web_set_globe_power_ctl(String p)
{
    int flag = p.toInt();
    if (flag) {
        g_globePowerControl = true;
        return 1;
    }

    g_globePowerControl = false;
    return 0;
}

void print_reset_reason()
{
    switch (System.resetReason()) {
        case RESET_REASON_UNKNOWN:
            Log.info("setup: Last Reset: Unknown reset!");
            display.println("Unknown reset!");
            break;
        case RESET_REASON_PIN_RESET:
            Log.info("setup: Last Reset: Reset due to reset button press!");
            display.println("Pushbutton reset!");
            break;
        case RESET_REASON_POWER_MANAGEMENT:
            Log.info("setup: Last Reset: Reset due to power management!");
            display.println("Reset due to PM!");
            break;
        case RESET_REASON_WATCHDOG:
            Log.info("setup: Last Reset: Watchdog reset!");
            display.println("Watchdog reset!");
            break;
        case RESET_REASON_UPDATE:
            Log.info("setup: Last Reset: Update finished!");
            display.println("Update finished!");
            break;
        case RESET_REASON_UPDATE_ERROR:
            Log.info("setup: Last Reset: Update failed!");
            display.println("Update failed!");
            break;
        case RESET_REASON_UPDATE_TIMEOUT:
            Log.info("setup: Last Reset: Update timed out!");
            display.println("Update timed out!");
            break;
        case RESET_REASON_FACTORY_RESET:
            Log.info("setup: Last Reset: Factory reset performed!");
            display.println("actory reset");
            break;
        case RESET_REASON_SAFE_MODE:
            Log.info("setup: Last Reset: Reset into safe mode!");
            display.println("Safe mode!");
            break;
        case RESET_REASON_DFU_MODE:
            Log.info("setup: Last Reset: Reset from DFU!");
            display.println("DFU!");
            break;
        case RESET_REASON_PANIC:
            Log.info("setup: Last Reset: System panic!");
            display.println("System panic!");
            break;
        case RESET_REASON_USER:
            Log.info("setup: Last Reset: User code called for a reset!");
            display.println("User reset!");
            break;
        default:
        case RESET_REASON_NONE:
            Log.info("setup: No reset reason given, resetReason returned %d!", System.resetReason());
            display.println("Unknown");
            break;
    }
}

void setup() 
{
    g_longitude = 0.0;
    g_latitude = 0.0;
    g_lat = 90;
    g_lon = 0;
    g_runLocationQuery = true;
    g_inCalibration = false;
    g_motorDecCalibrated = true;
    g_incOffset = 0;
    g_azimuthPosition = 0;
    g_appId = APP_ID;
    g_currentResolution = 800;
    g_motorHome = false;
    g_inclineHome = false;
    g_servoAngle = 90;
    g_displayEnabled = true;
    g_displayTimeout = ONE_MINUTE;
    g_globeTimeout = FIVE_MINUTES;
    g_displayTimeoutMillis = millis() + ONE_MINUTE;
    g_globeTimeoutMillis = millis() + FIVE_MINUTES;
    g_proximity = 60;
    g_cycleColors = false;
    g_globePowerControl = false;

    System.enableFeature(FEATURE_RESET_INFO);

    display.begin();
    display.fillScreen(0);
    display.setTextColor(ISS_WHITE, ISS_BLACK);
    display.setCursor(0,0);

    EEPROM.get(INC_CAL_ADDRESS, g_incOffset);
    Log.info("%s: inclination offset is %d", __FUNCTION__, g_incOffset);
    display.printf("Servo offset: %d\n", g_incOffset);

    pinMode(mtr_ms1, OUTPUT);
    pinMode(mtr_ms2, OUTPUT);
    pinMode(mtr_en, OUTPUT);
    pinMode(mtr_step, OUTPUT);
    pinMode(mtr_dir, OUTPUT);
    pinMode(AZIMUTH, INPUT);
    pinMode(DISTANCE, INPUT);
    pinMode(GLOBE_POWER, OUTPUT);
 
    g_lastResetReason = System.resetReason();

    Particle.subscribe("hook-response/iss_location", iss_location, MY_DEVICES);
    Particle.function("calibrate", web_calibrate);
    Particle.function("clockwise", web_rotate_clockwise);
    Particle.function("cclockwise", web_rotate_cclockwise);
    Particle.function("incline", web_jog_inclination);
    Particle.function("resolution", web_set_motor_resolution);
    Particle.function("offset", web_set_incline_offset);
    Particle.function("disptimeout", web_set_display_timeout);
    Particle.function("proximity", web_set_proximity_distance);
    Particle.function("globepower", web_set_globe_power_ctl);
    Particle.variable("version", g_appId);
    Particle.variable("inc_cal", g_incOffset);
    Particle.variable("azimuth", g_azimuthPosition);
    Particle.variable("reset", g_lastResetReason);
    Particle.variable("distance", g_distance);

    display.printf("Cloud complete\n");
    display.printf("Motor Cal...");
    reset_motor();
    set_motor_step_resolution(QUARTER_STEP);        // 800 steps per revolution, .45 deg per step
    set_motor_dir(CCLOCKWISE);
    display.printf("Enabling globe...\n");
    digitalWrite(GLOBE_POWER, HIGH);
    delay(10);
    display.printf("Starting servo...\n");
    servo.attach(SERVO);
    g_servoAngle = 90;
    servo.write(g_servoAngle);
    display.printf("Done!\n");

    print_reset_reason();
    issUpdate.start();

    Log.info("%s: Done with setup for app id %d", __FUNCTION__, g_appId);
    display.printf("Setup ver %d\n", g_appId);
    delay(3000);
    display.fillScreen(0);
}

void loop() 
{
    static int lastHour = 24;

    if (g_runLocationQuery) {
        Particle.publish("iss_location", PRIVATE);
        g_runLocationQuery = false;
    }

    if (Time.hour() != lastHour) {
        Particle.syncTime();
        waitUntil(Particle.syncTimeDone);
        lastHour = Time.hour();
    }
    if (g_inCalibration) {
        if (!g_motorHome)
            set_motor_home();

        if (!g_inclineHome)
            set_inclination(90);
    }
    detect_motion();
    display_update();
}
