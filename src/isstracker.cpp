/* 
 * Project ISS Tracker
 * Author: Peter Buelow
 * Date: 8/2/2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <Adafruit_GFX_PB.h>
#include <Adafruit_SSD1351_PB.h>
#include <Adafruit-MotorShield-V2.h>

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

#define APP_ID              11

#define MOTION              A1
#define CS                  S3
#define DC                  S4
#define RST                 A5
#define MOTOR_ERR           D6
#define WIFI_ERR            D7
#define SERVO               MISO
#define ZLON                D4
#define LASER               D3
#define LED                 D2

#define ONE_SECOND          1000
#define FIVE_SECONDS        (ONE_SECOND * 5)
#define ONE_MINUTE          (ONE_SECOND * 60)
#define FIVE_MINUTES        (ONE_MINUTE * 5)
#define TEN_MINUTES         (ONE_MINUTE * 10)
#define LATCAL_INDEX        0
#define LONCAL_INDEX        1

#define ISS_BLACK           0x0000
#define ISS_BLUE            0x001F
#define ISS_RED             0xF800
#define ISS_GREEN           0x07E0
#define ISS_CYAN            0x07FF
#define ISS_MAGENTA         0xF81F
#define ISS_YELLOW          0xFFE0
#define ISS_WHITE           0xFFFF

SerialLogHandler logHandler(LOG_LEVEL_INFO);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
Adafruit_SSD1351 display = Adafruit_SSD1351(CS, DC, RST);
Servo latitude;

String g_version;
double g_issLon;
double g_issLat;
double g_absoluteLat;
double g_absoluteLon;
int g_issLatInt;
int g_latOffset;
int g_lonOffset;
uint16_t g_motorPosition;
bool g_moveLon;
bool g_moveLat;
bool g_displayEnabled;
bool g_laserEnabled;
bool g_firstBoot;
bool g_inCalibration;
bool g_homeMotor;
bool g_motorFailed;

void request_position()
{
    String data = String(10);
    Particle.publish("iss_location", data, PRIVATE);
}

int uptime(char *buf, int size)
{
    Log.info("%s", __PRETTY_FUNCTION__);
    unsigned long seconds = System.uptime();
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
    seconds %= 60;
    minutes %= 60;
    hours %= 24;

    memset(buf, 0, size);
    int bytes = snprintf(buf, size, "%ld days %ld:%02ld:%02ld", days, hours, minutes, seconds);
    return bytes;
}

void display_update()
{
    char t[10];

    if (!g_displayEnabled) {
        return;
    }

    if (g_firstBoot) {
        display.fillScreen(0);
        g_firstBoot = false;
    }

    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(ISS_WHITE, ISS_BLACK);
    display.printf("Ver: %s\n", g_version.c_str());
    display.setTextColor(ISS_CYAN, ISS_BLACK);
    display.printf("ISS Location\n");
    if (g_issLat < 0) {
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("Lat: ");
        display.setTextColor(ISS_BLUE, ISS_BLACK);
        display.printf("%10.05f S\n", g_issLat);
    }
    else {
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("Lat: ");
        display.setTextColor(ISS_GREEN, ISS_BLACK);
        display.printf("%10.05f N\n", g_issLat);
    }
    if (g_issLon < 0) {
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("Lon: ");
        display.setTextColor(ISS_BLUE, ISS_BLACK);
        display.printf("%10.05f W\n", g_issLon);
    }
    else {
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("Lon: ");
        display.setTextColor(ISS_GREEN, ISS_BLACK);
        display.printf("%10.05f E\n", g_issLon);
    }
    display.setTextColor(ISS_WHITE, ISS_BLACK);
    display.printf("Laser State  : ");
    if (g_laserEnabled) {
        display.setTextColor(ISS_GREEN, ISS_BLACK);
        display.printf("ON \n");
    }
    else {
        display.setTextColor(ISS_RED, ISS_BLACK);
        display.printf("OFF\n");
    }
    display.setTextColor(ISS_WHITE, ISS_BLACK);

    if (WiFi.ready()) {
        int rssi = WiFi.RSSI();
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("IP: ");
        display.setTextColor(ISS_CYAN, ISS_BLACK);
        display.printf("%s\n", WiFi.localIP().toString().c_str());
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("SSID: ");
        display.setTextColor(ISS_CYAN, ISS_BLACK);
        display.printf("%s\n", WiFi.SSID());
        display.setTextColor(ISS_WHITE, ISS_BLACK);
        display.printf("RSSI: ");
        if (rssi > -65)
            display.setTextColor(ISS_GREEN, ISS_BLACK);
        else if (rssi > -80)
            display.setTextColor(ISS_YELLOW, ISS_BLACK);
        else
            display.setTextColor(ISS_RED, ISS_BLACK);
        display.printf("%d\n", rssi);
    }
    else {
        display.setTextColor(ISS_RED, ISS_BLACK);
        display.printf("WiFi Disconnected\n");
    }

    display.setTextColor(ISS_WHITE, ISS_BLACK);
    uptime(t, 10);
    display.printf("Uptime: %s\n", t);
}

void iss_position(const char *event, const char *data) 
{
    JSONValue outerObj = JSONValue::parseCopy(data);

    if (outerObj.isValid()) {
        JSONObjectIterator i(outerObj);
        while(i.next()) {
            if (i.name() == "iss_position") {
                JSONValue position = i.value();
                if (position.isValid()) {
                    JSONObjectIterator j(position);
                    while (j.next()) {
                        if (j.name() == "longitude") {
                            g_issLon = j.value().toDouble();
                            if (g_issLon < 0) {
                                g_absoluteLon = 360 + g_issLon;
                            }
                            g_moveLon = true;
                        }
                        if (j.name() == "latitude") {
                            g_issLat = j.value().toDouble();
                            int abslat = round(g_issLat);
                            g_issLatInt = map(abslat, -90, 90, 0, 180);
                            g_issLatInt += g_latOffset;
                            g_moveLat = true;
                        }
                    }
                }
            }
        }
    }
}

void decode_version_string()
{
    g_version = System.version() + "." + String(APP_ID);
    Log.info("%s: %s", __PRETTY_FUNCTION__, g_version.c_str());
}

void display_timeout()
{
    display.fillScreen(0);
    g_displayEnabled = false;
}

void laser_timeout()
{
    digitalWrite(LASER, LOW);
    g_laserEnabled = false;
}

void step_motor(int steps, int direction, int which)
{
    Log.info("%s: %d steps moving %d using mechanism %d", __PRETTY_FUNCTION__, steps, direction, which);
    if (!g_motorFailed)
        myMotor->step(steps, direction, which);
}

int jog_latitude(String param)
{
    int jog = param.toInt();
    if (jog != 0) {
        if (jog > 0) {
            step_motor(jog, BACKWARD, INTERLEAVE);
        }
        if (jog < 0) {
            step_motor(jog, FORWARD, INTERLEAVE);
        }
        g_latOffset += jog;
    }
    return jog;
}

int jog_longitude(String param)
{
    return param.toInt();
}

int enable_calibration(String param) 
{
    int enable = param.toInt();
    if (enable) {
        g_inCalibration = true;
        g_homeMotor = true;
    }
    else {
        EEPROM.write(LONCAL_INDEX, g_lonOffset);
        EEPROM.write(LATCAL_INDEX, g_latOffset);
        g_inCalibration = false;
    }

    return enable;
}

void apply_offsets()
{
    if (g_lonOffset) {
        if (g_lonOffset > 0) {
            step_motor(g_lonOffset, BACKWARD, INTERLEAVE);
        }
        else {
            step_motor((g_lonOffset * -1), FORWARD, INTERLEAVE);
        }
    }
}

void home_motor()
{
    Log.info("%s", __PRETTY_FUNCTION__);
    if (!g_motorFailed) {
        while (digitalRead(ZLON) == HIGH) {
            step_motor(1, FORWARD, SINGLE);
            delay(5);
            Particle.process();
        }
    }

    g_motorPosition = 0;
    g_homeMotor = false;
}

Timer positionTimer(2000, request_position);
Timer laserTimer(FIVE_MINUTES, laser_timeout);
Timer displayTimer(ONE_MINUTE, display_timeout);

void motion_detected()
{
    if (!g_laserEnabled) {
        laserTimer.reset();
        g_laserEnabled = true;
        digitalWrite(LASER, HIGH);
    }

    if (!g_displayEnabled) {
        displayTimer.reset(0);
        g_displayEnabled = true;
    }
}

void setup() 
{
    g_moveLat = false;
    g_moveLon = false;
    g_displayEnabled = true;
    g_laserEnabled = true;
    g_firstBoot = true;
    g_inCalibration = false;
    g_homeMotor = true;
    g_motorFailed = false;

    EEPROM.get(LATCAL_INDEX, g_latOffset);
    EEPROM.get(LONCAL_INDEX, g_lonOffset);

    if (g_latOffset == 255) {
        g_latOffset = 0;
        EEPROM.put(LATCAL_INDEX, 0);
    }

    if (g_lonOffset == 255) {
        g_lonOffset = 0;
        EEPROM.put(LONCAL_INDEX, 0);
    }

    decode_version_string();

    display.begin();
    display.fillScreen(0);
    display.setTextColor(ISS_WHITE, ISS_BLACK);
    display.setCursor(0,0);

    display.println("Setting up GPIO");
    pinMode(LASER, OUTPUT);
    pinMode(MOTION, INPUT);
    pinMode(ZLON, INPUT);
    pinMode(MOTOR_ERR, OUTPUT);
    pinMode(WIFI_ERR, OUTPUT);

    Log.info("Applying offsets: LAT: %d, LON: %d", g_latOffset, g_lonOffset);
    display.printf("CAL LAT %d LON %d\n", g_latOffset, g_lonOffset);

    display.println("Starting servo");
    Log.info("Establishing servo for Latitude");
    latitude.attach(SERVO);
    latitude.write(90);
    g_issLat = 90;

    display.println("Starting motor");
    display.println("Finding 0 lat");
    AFMS.begin();
    myMotor->setSpeed(10);
    home_motor();
    apply_offsets();

    display.println("Establishing Particle callbacks");
    Particle.subscribe("hook-response/iss_location", iss_position, MY_DEVICES);
    Particle.variable("latitude", g_issLat);
    Particle.variable("longitude", g_issLon);
    Particle.variable("laser", g_laserEnabled);
    Particle.variable("display", g_displayEnabled);
    Particle.function("joglat", jog_latitude);
    Particle.function("joglon", jog_longitude);
    Particle.function("calibrate", enable_calibration);

    display.println("Starting timers...");
    laserTimer.start();
    displayTimer.start();
    positionTimer.start();

    attachInterrupt(MOTION, motion_detected, RISING);

    digitalWrite(LASER, HIGH);
    Log.info("Started ISS Tracker version: %s", g_version.c_str());
    display.println("Success...");
}

void loop() 
{
    static int second = 60;

    if ((g_moveLon || g_moveLat) && !g_inCalibration) {
        g_moveLon = false;
        int position = static_cast<int>((g_absoluteLon / 0.9));

        latitude.write(g_issLatInt);
        g_moveLat = false;
        
        int steps = position - g_motorPosition;
        if (steps < 0) {
            steps = 1;
        }

        g_motorPosition = g_motorPosition + steps;
        if (g_motorPosition == 400)
            g_motorPosition = 0;

        Log.info("Lat: %d, Lon: %d", g_issLatInt, g_motorPosition);
        if (steps != 0)
            step_motor(steps, BACKWARD, INTERLEAVE);
        
        display_update();
    }

    if (g_inCalibration && g_homeMotor) {
        home_motor();
    }

    if (Time.second() != second) {
        second = Time.second();
        if (digitalRead(D7) == HIGH) {
            digitalWrite(D7, LOW);
        }
        else {
            digitalWrite(D7, HIGH);
        }
    }
}
