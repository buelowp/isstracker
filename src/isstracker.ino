/*
 * Project isstracker
 * Description:
 * Author:
 * Date:
 */
#include "ArduinoJson.h"
#include "MQTT.h"

#define APP_ID                  21

#define LASER_ENABLE        DAC

#define LASER_MOTOR_STEP          D1
#define LASER_MOTOR_DIR           D0
#define LASER_MOTOR_MS1           D2
#define LASER_MOTOR_ENABLE        D3
#define LASER_MOTOR_MS2           D4
#define LASER_MOTOR_HOME          D5

#define BASE_MOTOR_ENABLE       RX
#define BASE_MOTOR_STEP         A1
#define BASE_MOTOR_DIR          A0
#define BASE_MOTOR_MS1          A2
#define BASE_MOTOR_MS2          A4

#define LATITUDE_HOME           D5
#define LONGITUDE_HOME          D6

#define LAT_CLOCKWISE           LOW
#define LAT_CTRCLKWISE          HIGH
#define LON_CLOCKWISE           LOW
#define LON_CTRCLKWISE          HIGH

#define LON_PRECISION           400
#define LON_PRECISION_ADDR      0x00

#define LAT_PRECISION           144
#define LAT_PRECISION_ADDR      0x02

#define ONE_SECOND          (1000)
#define TEN_SECONDS         (10 * ONE_SECOND)

bool g_calibrated;
bool g_enabled;
bool g_needPosition;
uint16_t g_latPrecision;
uint16_t g_lonPrecision;
int g_denominator;      // 9 for whole step, 18 for half steps, 36 for quarter steps
int g_rounder;          // 5 for whole step, 9 for half steps, 18 for quarter steps
double g_latitude;
double g_longitude;
double g_lastLat;
double g_lastLon;
double g_latResolution;
double g_lonResolution;
int g_appId;
uint32_t g_lastISSRequest;
char server[] = "172.24.1.13";
MQTT g_mqtt(server, 1883, mqttCallback);

void calibrateMotorLocation()
{
    int steps = 0;
    Serial.println("Calibrating Motor");
    digitalWrite(LASER_MOTOR_DIR, LAT_CLOCKWISE);

    while (digitalRead(LATITUDE_HOME) == HIGH) {
        digitalWrite(LASER_MOTOR_STEP, HIGH);
        delay(30);
        digitalWrite(LASER_MOTOR_STEP, LOW);
        delay(30);
        steps++;
        Particle.process();
    }
    Serial.print("Moved ");
    Serial.print(steps);
    Serial.println(" to calibrate");
/*
    digitalWrite(BASE_MOTOR_DIR, LON_CLOCKWISE);

    while (digitalRead(LONGITUDE_HOME) == LOW) {
        digitalWrite(BASE_MOTOR_STEP, HIGH);
        delay(1);
        digitalWrite(BASE_MOTOR_STEP, LOW);
        delay(1);
    }
*/
    g_calibrated = true;
    g_latitude = 0;
    g_longitude = 0;
}

// recieve message
void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    if (strcmp(topic, "/iss/calibrate") == 0) {
        calibrateMotorLocation();
    }
}

// "iss_position": {"longitude": "-74.2342", "latitude": "-3.9524"}, "
void getISSLocation(const char *event, const char *data)
{
    Serial.println(data);
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, data);
    if (!err) {
        g_latitude = doc["iss_position"]["latitude"].as<double>();
        g_longitude = doc["iss_position"]["longitude"].as<double>();
        updateLaserLocation();
    }
    else {
        Serial.print("Cannot deserialze reponse: ");
    }
}

void moveLatitude(int steps)
{
    Serial.print("Moving ");
    Serial.print(steps);
    Serial.println(" steps latitude");
    if (steps < 0) {
        digitalWrite(LASER_MOTOR_DIR, LAT_CLOCKWISE);
        steps = steps * -1;
    }
    else
        digitalWrite(LASER_MOTOR_DIR, LAT_CTRCLKWISE);

    for (int i = 0; i < steps; i++) {
        digitalWrite(LASER_MOTOR_STEP, HIGH);
        delay(1);
        digitalWrite(LASER_MOTOR_STEP, LOW);
        delay(1);
        Particle.process();
    }
}

void moveLongitude(int steps)
{
    if (steps < 0)
        digitalWrite(BASE_MOTOR_DIR, HIGH);

    for (int i = 0; i < steps; i++) {
        digitalWrite(BASE_MOTOR_STEP, HIGH);
        delay(1);
        digitalWrite(BASE_MOTOR_STEP, LOW);
        delay(1);
        Particle.process();
    }
}

void updateLaserLocation()
{
    int steps = 0;

    if (g_needPosition) {
        g_lastLat = g_latitude;
        steps = g_latitude / g_latResolution;
        g_needPosition = false;
    }
    else {
        double delta = g_latitude - g_lastLat;
        Serial.print("distance traveled: ");
        Serial.println(delta);
        if (delta >= 0 && delta >= g_latResolution) {
            g_lastLat += g_latResolution;
            steps = 1;
        }
        if (delta <= 0 && (delta * -1) >= g_latResolution) {
            g_lastLat -= g_latResolution;
            steps = -1;
        }
    }
    
    moveLatitude(steps);
//    moveLongitude(lonSteps);
}

int disableLaser(String)
{
    digitalWrite(LASER_ENABLE, LOW);
    return 0;
}

void setLatitudePrecision()
{
    int ms1;
    int ms2;

    EEPROM.get(LAT_PRECISION_ADDR, g_latPrecision);

    if (g_latPrecision == 0xffff) {
        Serial.print("Unable to retrieve precision from EEPROM, defaulting to 400 and storing");
        g_latPrecision = LAT_PRECISION;
        EEPROM.put(LAT_PRECISION_ADDR, g_latPrecision);
    }

    switch (g_latPrecision) {
        case 48:
            ms1 = LOW;
            ms2 = LOW;
            g_latResolution = 7.5;
            break;
        case 400:
            ms1 = HIGH;
            ms2 = LOW;
            g_latResolution = 3.75;
            break;
        case 800:
            ms1 = LOW;
            ms2 = HIGH;
            g_latResolution = 1.875;
            break;
        case 1600:
        default:
            ms1 = HIGH;
            ms2 = HIGH;
            g_latResolution = .9375;
            break;
    }

    Serial.print("Latitude angular resolution is now 1 step to ");
    Serial.print(g_latResolution);
    Serial.print(" degrees");
    digitalWrite(LASER_MOTOR_MS1, ms1);
    digitalWrite(LASER_MOTOR_MS2, ms2);
}

void setLongitudePrecision()
{
    int ms1;
    int ms2;

    EEPROM.get(LON_PRECISION_ADDR, g_lonPrecision);

    if (g_lonPrecision == 0xffff) {
        Serial.print("Unable to retrieve precision from EEPROM, defaulting to 400 and storing");
        g_lonPrecision = LON_PRECISION;
        EEPROM.put(LON_PRECISION, g_lonPrecision);
    }

    switch (g_lonPrecision) {
        case 200:
            ms1 = LOW;
            ms2 = LOW;
            g_lonResolution = 1.8;
            break;
        case 400:
            ms1 = HIGH;
            ms2 = LOW;
            g_lonResolution = .9;
            break;
        case 800:
            ms1 = LOW;
            ms2 = HIGH;
            g_lonResolution = .45;
            break;
        case 1600:
            ms1 = HIGH;
            ms2 = HIGH;
            g_lonResolution = .225;
            break;
        default:
            ms1 = HIGH;
            ms2 = LOW;
            g_lonResolution = .9;
            break;
    }

    digitalWrite(BASE_MOTOR_MS1, ms1);
    digitalWrite(BASE_MOTOR_MS2, ms2);
}


void setup() 
{
    Serial.begin(115200);

    g_appId = APP_ID;
    g_lastISSRequest = 0;
    g_calibrated = false;
    g_enabled = true;
    g_needPosition = true;
    g_latitude = 0;
    g_longitude = 0;
    g_lastLon = 0;
    g_lastLat = 0;
    
    Serial.print("Starint app with ID ");
    Serial.println(g_appId);
    Serial.print("Ram available: ");
    Serial.println(System.freeMemory());

    pinMode(LASER_ENABLE, OUTPUT);

    pinMode(LASER_MOTOR_DIR, OUTPUT);
    pinMode(LASER_MOTOR_ENABLE, OUTPUT);
    pinMode(LASER_MOTOR_MS1, OUTPUT);
    pinMode(LASER_MOTOR_MS2, OUTPUT);
    pinMode(LASER_MOTOR_STEP, OUTPUT);
    pinMode(LATITUDE_HOME, INPUT);

    pinMode(BASE_MOTOR_DIR, OUTPUT);
    pinMode(BASE_MOTOR_ENABLE, OUTPUT);
    pinMode(BASE_MOTOR_MS1, OUTPUT);
    pinMode(BASE_MOTOR_MS2, OUTPUT);
    pinMode(BASE_MOTOR_STEP, OUTPUT);
    pinMode(LONGITUDE_HOME, INPUT);

    digitalWrite(LASER_ENABLE, HIGH);
    digitalWrite(BASE_MOTOR_ENABLE, HIGH);
    digitalWrite(LASER_MOTOR_ENABLE, LOW);

    Particle.subscribe("hook-response/iss_location", getISSLocation, MY_DEVICES);
//    Particle.function("laser", disableLaser);

    setLatitudePrecision();
    setLongitudePrecision();


    calibrateMotorLocation();
}

void loop() 
{
    if (millis() > (g_lastISSRequest + TEN_SECONDS)) {
        Particle.publish("iss_location", "", PRIVATE);
        g_lastISSRequest = millis();
    }
}