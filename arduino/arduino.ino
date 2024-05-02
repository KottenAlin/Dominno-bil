#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include "ArduinoJson.h"
#include <Servo.h>
#include <VL53L0X.h>

// Servo constants
const int SERVO_PIN = D7;

const unsigned int SERVO_ANGLE_MAX = 180;
const unsigned int SERVO_ANGLE_MIN = 0;

// Motor constants
const unsigned int MOTOR_SPEED_MAX = 1024;
const unsigned int MOTOR_SPEED_MIN = 400;

// Drive constants
const uint8_t DRIVE_DPIN = D3;
const uint8_t DRIVE_SPIN = D1;

const unsigned int DRIVE_DELAY_MAX = 350;
const unsigned int DRIVE_DELAY_MIN = 200;

// Domino constants
const uint8_t DOMINO_DPIN = D4;
const uint8_t DOMINO_SPIN = D2;

// DOMINO_SPEED: The procent of MOTOR_SPEED
const unsigned int DOMINO_SPEED = 100;

const unsigned int DOMINO_DELAY = 2000;

JsonDocument commandJson;

int turnProcent = 50;  // The procent rotation of the drive (0 - 100)
int driveSpeed  = 0; // The speed which to drive forward  (0 - 100)
int dominoSpace = 0; // The space between domino bricks   (0 - 100)

int lastTurnProcent = turnProcent;

ESP8266WebServer server(80);

const unsigned long BAUD_VALUE = 9600;
const unsigned int  LOOP_DELAY = 1000;

// WIFI constants
#define WIFI_SSID     "ABBgym_2.4"
#define WIFI_PASSWORD "mittwifiarsabra"

const unsigned int WIFI_TRIES = 32;
const unsigned int WIFI_DELAY = 1000;

Servo servo;

void setup() 
{
  Serial.begin(BAUD_VALUE);

  if(motor_servo_setup() != 0)
  {
    Serial.println("ERROR: Motor and servo setup");
    exit(1);
  }
  else Serial.println("SUCCESS: Motor and servo setup");

  if(wifi_connect(WIFI_SSID, WIFI_PASSWORD) != 0)
  {
    Serial.println("ERROR: WIFI connect");
    exit(2);
  }
  else Serial.println("SUCCESS: WIFI connect");

  Serial.println(WiFi.localIP());

  server.on("/command", HTTP_POST, command_handler);

  server.begin();
}

/*
 * RETURN (int status)
 * - 0 | Success!
 * - 1 | Failed to attack SERVO_PIN
 */
int motor_servo_setup(void)
{
  pinMode(DRIVE_DPIN, OUTPUT);
  pinMode(DRIVE_SPIN, OUTPUT);

  pinMode(DOMINO_DPIN, OUTPUT);
  pinMode(DOMINO_SPIN, OUTPUT);

  return (servo.attach(SERVO_PIN) == 0) ? 1 : 0;
}

/*
 * Connect to a WIFI using SSID and password
 *
 * PARAMS
 * - const char* ssid     | The WIFI SSID
 * - const char* password | The WIFI password
 *
 * RETURN (int status)
 * - 0 | Success!
 * - 1 | WIFI status is WL_NO_SHIELD
 * - 2 | Failed to connect to WIFI
 */
int wifi_connect(const char* ssid, const char* password)
{
  WiFi.mode(WIFI_STA);

  if (WiFi.status() == WL_NO_SHIELD) return 1;

  WiFi.begin(ssid, password);

  for (int index = 0; index <= WIFI_TRIES; index += 1)
  {
    if (WiFi.status() == WL_CONNECTED) return 0;

    delay(WIFI_DELAY);
  }
  return 1;
}

/*
 *
 */
void command_handler(void)
{
  if(server.hasArg("plain"))
  {
    String string = server.arg("plain");

    string.trim();

    DeserializationError error = deserializeJson(commandJson, string);

    if(error)
    {
      char buffer[1024];

      sprintf(buffer, "ERROR: %s", error.f_str());

      server.send(500, "text/plain", buffer);
    }
    else
    {
      command_json_parse();

      Serial.print("turnProcent : "); Serial.println(turnProcent);
      Serial.print("driveSpeed  : "); Serial.println(driveSpeed);
      Serial.print("dominoSpace : "); Serial.println(dominoSpace);

      server.send(200, "text/plain", "OK");
    }
  }
  else
  {
    server.send(500, "text/plain", "ERROR: No text was sent");
  }
}

/*
 * Parse the command json string, to update the driving values
 *
 * If the json object does not contain a specific value (ex driveSpeed),
 * do not update the value, continue with the existing value
 */
void command_json_parse(void)
{
  // turn procent
  int jsonTurnProcent = commandJson["turnProcent"] | -1;

  if(jsonTurnProcent != -1) turnProcent = jsonTurnProcent;

  // drive speed
  int jsonDriveSpeed = commandJson["driveSpeed"] | -1;

  if(jsonDriveSpeed != -1) driveSpeed = jsonDriveSpeed;

  // domino space
  int jsonDominoSpace = commandJson["dominoSpace"] | -1;

  if(jsonDominoSpace != -1) dominoSpace = jsonDominoSpace;
}

void loop() {
  server.handleClient();

  if(driveSpeed > 0)
  {
    domino_place();
  }
  if(turnProcent >= 0 && lastTurnProcent != turnProcent)
  {
    wheels_turn();
  }

  delay(500);
}

/*
 * Place a domino brick on the floor
 */
void domino_place(void)
{
  // 3. Open the door and return the feeder
  domino_motor_rotate(-DOMINO_SPEED);

  delay(DOMINO_DELAY);

  // 4. Stop the rotation when driving
  domino_motor_rotate(0);

  // 2. Drive forward leaving the domino standing
  wheels_drive();

  // 1. Close the door and rotate the feeder
  domino_motor_rotate(DOMINO_SPEED);

  delay(DOMINO_DELAY);

  // 2. Stop the rotation and let domino stay still
  domino_motor_rotate(0);
}

void domino_motor_rotate(int procent)
{
  motor_rotate(DOMINO_DPIN, DOMINO_SPIN, procent);
}

#define DRIVE_PROCENT_DELAY(PROCENT) (((float) (PROCENT) / 100) * (float) ((DRIVE_DELAY_MAX) - (DRIVE_DELAY_MIN)) + (DRIVE_DELAY_MIN))

/*
 *
 */
void wheels_drive(void)
{
  // 1. Start driving forward
  drive_motor_rotate(driveSpeed);

  // 2. Wait some time, depending on 
  delay(DRIVE_PROCENT_DELAY(dominoSpace));

  // 3. Stop driving any further forward
  drive_motor_rotate(0);
}

void drive_motor_rotate(int procent)
{
  motor_rotate(DRIVE_DPIN, DRIVE_SPIN, procent);
}

#define PROCENT_SERVO_ANGLE(PROCENT) (((float) (PROCENT) / 100) * (float) ((SERVO_ANGLE_MAX) - (SERVO_ANGLE_MIN)) + (SERVO_ANGLE_MIN))

/*
 * 
 */
void wheels_turn(void)
{
  servo.write(PROCENT_SERVO_ANGLE(turnProcent));

  lastTurnProcent = turnProcent;
}

#define PROCENT_MOTOR_SPEED(PROCENT) ((float) (PROCENT) / 100 * (float) ((MOTOR_SPEED_MAX) - (MOTOR_SPEED_MIN)) + (MOTOR_SPEED_MIN))

/*
 * Rotate the motor connected to the inputted dpin and spin,
 * at the inputted procent of max motor speed
 *
 * PARAMS
 * - uint8_t dpin | The motor dpin
 * - uint8_t spin | The motor spin
 * - int procent  | The procent of the speed which to rotate the motor
 *
 * RETURN (int status)
 * - 0 | Success!
 * - 1 | The inputted procent is not valid
 */
int motor_rotate(uint8_t dpin, uint8_t spin, int procent)
{
  if(procent < -100 || procent > 100) return 1;

  digitalWrite(dpin, (procent > 0 ) ? HIGH : LOW);

  int value = (procent == 0) ? 0 : PROCENT_MOTOR_SPEED(abs(procent));

  analogWrite(spin, value);

  return 0;
}