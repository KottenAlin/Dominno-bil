#include "arduino.h"

const unsigned long BAUD_VALUE = 9600;

const char WIFI_SSID[] = "ABBgym_2.4";
const char WIFI_PASSWORD[] = "mittwifiarsabra";

const unsigned int CONNECT_TRIES = 32;
const unsigned int WIFI_DELAY = 1000;

const unsigned int LOOP_DELAY = 1000;

const int SERVO_PIN = D7;

const unsigned int MAX_SANGLE = 180;
const unsigned int MIN_SANGLE = 0;

const uint8_t MOTOR_DPIN = D3;
const uint8_t MOTOR_SPIN = D1;

const unsigned int MAX_MSPEED = 400;
const unsigned int MIN_MSPEED = 120;

const int ROTATE_INCR = 5;
const int ROTATE_TIME = 5000;

#define ANGLE_TO_DECIMAL(ANGLE) ((float) ((ANGLE) - (MIN_SANGLE)) / (float) ((MAX_SANGLE) - (MIN_SANGLE)))

#define DECIMAL_TO_ANGLE(DECIMAL) ((float) (DECIMAL) * (float) ((MAX_SANGLE) - (MIN_SANGLE)) + (MIN_SANGLE))

WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "pool.ntp.org");

Servo servo;

EspMQTTClient client(
  "ABBgym_2.4",      // WIFI SSID
  "mittwifiarsabra", // WIFI Password
  "10.22.4.26",      // MQTT Broker server ip
  "",                // Can be omitted if not needed
  "",                // Can be omitted if not needed
  "domino-car",      // Client name that uniquely identify your device
  1883               // MQTT Broker server port
);

DynamicJsonDocument commandJson(300);
DynamicJsonDocument statusJson(300);
DynamicJsonDocument actionJson(300);

char driveDir[64];
int driveSpeed = 0;

char rotateDir[64];
int rotateSpeed = 0;

double motorValue = 0;
double servoValue = 0;

char inputType[64];

char statusString[256];
char actionString[256];

char buffer[1024];

void setup()
{
  Serial.begin(BAUD_VALUE);

  if(!wifi_setup())
  {
    Serial.println("ERROR: WIFI setup");
    exit(1);
  }
  else Serial.println("SUCCESS: WIFI setup");
  
  if(!time_client_setup())
  {
    Serial.println("ERROR: Time client setup");
    exit(2);
  }
  else Serial.println("SUCCESS: Time client setup");
  
  if(!motor_servo_setup())
  {
    Serial.println("ERROR: Motor and servo setup");
    exit(3);
  }
  else Serial.println("SUCCESS: Motor and servo setup");
  
  if(!mqtt_client_setup())
  {
    Serial.println("ERROR: MQTT client setup");
    exit(4);
  }
  else Serial.println("SUCCESS: MQTT client setup");
}

bool wifi_setup()
{
  if (wifi_ssid_password_connect(WIFI_SSID, WIFI_PASSWORD)) return true;
  
  return false;
}

bool time_client_setup()
{
  timeClient.begin();

  return true;
}

bool motor_servo_setup()
{
  pinMode(MOTOR_DPIN, OUTPUT);
  pinMode(MOTOR_SPIN, OUTPUT);

  servo.attach(SERVO_PIN);

  return true;
}

bool mqtt_client_setup()
{
  sprintf(buffer, "Connecting to broker: (%s:%d)", client.getMqttServerIp(), client.getMqttServerPort());
  Serial.println(buffer);

  client.enableDebuggingMessages();
  client.enableHTTPWebUpdater();
  client.enableOTA();

  alloc_status_string("Currently offline");
  client.enableLastWillMessage("hampus/arduino-status", statusString);

  // return client.isConnected();
  return true;
}

bool wifi_ssid_password_connect(const char ssid[], const char password[])
{
  WiFi.mode(WIFI_STA);

  if (WiFi.status() == WL_NO_SHIELD) return false;

  WiFi.begin(ssid, password);

  for (int index = 0; index <= CONNECT_TRIES; index += 1)
  {
    if (WiFi.status() == WL_CONNECTED) return true;

    delay(WIFI_DELAY);
  }
  return false;
}

void onConnectionEstablished()
{
  publish_status_message("Connected to broker");

  client.subscribe("hampus/command", [](const String & payload)
  {
    DeserializationError error = deserializeJson(commandJson, payload);

    if (error)
    {
      Serial.print("Json Failed: ");
      Serial.println(error.f_str());

      publish_status_message("Error parsing JSON");

      return;
    }

    publish_action_values();

    const char* jsonInputType = commandJson["type"];
    strcpy(inputType, jsonInputType);

    if (!strcmp(inputType, "buttons") || !strcmp(inputType, "keypress"))
    {
      update_drive_rotate();
    }
    else if (!strcmp(inputType, "joystick"))
    {
      motorValue = commandJson["values"]["motorValue"];
      servoValue = commandJson["values"]["servoValue"];
    }
    else publish_status_message("Unknown input type");
  });
}

void update_drive_rotate()
{
  const char* jsonDriveDir = commandJson["values"]["driveDir"];
  const char* jsonRotateDir = commandJson["values"]["rotateDir"];

  if (jsonDriveDir != NULL)
  {
    strcpy(driveDir, jsonDriveDir);
    driveSpeed = commandJson["values"]["driveSpeed"];
  }
  if (jsonRotateDir != NULL)
  {
    strcpy(rotateDir, jsonRotateDir);
    rotateSpeed = commandJson["values"]["rotateSpeed"];
  }
}

void publish_action_values()
{
  actionJson.clear();
  timeClient.update();

  actionJson["servoAngle"] = servo.read();
  actionJson["motorValue"] = digitalRead(MOTOR_DPIN);
  actionJson["motorSpeed"] = analogRead(MOTOR_SPIN);
  actionJson["time"] = timeClient.getEpochTime();

  serializeJson(actionJson, actionString);

  client.publish("hampus/arduino-action", actionString);
}

void alloc_status_string(const char message[])
{
  statusJson.clear(); 
  timeClient.update();

  statusJson["status"] = message;
  statusJson["time"] = timeClient.getEpochTime();

  serializeJson(statusJson, statusString);
}

void publish_status_message(const char message[])
{
  alloc_status_string(message);

  client.publish("hampus/arduino-status", statusString);
}

void loop()
{
  client.loop();

  if (!strcmp(inputType, "buttons") || !strcmp(inputType, "keypress"))
  {
    buttons_keypress_handler();
  }
  if (!strcmp(inputType, "joystick"))
  {
    joystick_handler();
  }
}

void joystick_handler()
{
  motor_rotate(MOTOR_DPIN, MOTOR_SPIN, (int) motorValue);

  servo_negative_procent_rotate((int) servoValue);
}

void buttons_keypress_handler()
{
  if (driveDir != NULL && driveSpeed >= 0)
  {
    drive_action_parse(driveDir, driveSpeed);
  }
  if (rotateDir != NULL && rotateSpeed >= 0)
  {
    servo_rotate_action_parse(rotateDir, rotateSpeed);
  }  
}

void drive_action_parse(const char driveDir[], int driveSpeed)
{
  if (!strcmp(driveDir, "north"))
  {
    motor_rotate(MOTOR_DPIN, MOTOR_SPIN, driveSpeed);
  }
  else if (!strcmp(driveDir, "south"))
  {
    motor_rotate(MOTOR_DPIN, MOTOR_SPIN, -driveSpeed);
  }
  else if (!strcmp(driveDir, "none"))
  {
    motor_rotate(MOTOR_DPIN, MOTOR_SPIN, 0);
  }
}

void servo_rotate_action_parse(const char rotateDir[], int rotateSpeed)
{
  if (!strcmp(rotateDir, "west"))
  {
    servo_rotate_step_action(rotateSpeed, true);
  }
  else if (!strcmp(rotateDir, "east"))
  {
    servo_rotate_step_action(rotateSpeed, false);
  }
}

void servo_rotate_step_action(int speedProcent, bool directBool)
{
  float readDecimal = ANGLE_TO_DECIMAL(servo.read());

  int readProcent = readDecimal * 100;

  int startProcent = directBool ? (100 - readProcent) : readProcent;

  int stepDelay = servo_rotate_step_delay(speedProcent);

  int index = startProcent + ROTATE_INCR;

  if (index >= 100) return;

  int rotateProcent = directBool ? (100 - index) : index;

  servo_procent_rotate(rotateProcent);

  delay(stepDelay);
}

float servo_rotate_step_delay(int speedProcent)
{
  if (speedProcent < 0 || speedProcent > 100) return 0;

  float decimal = (float) (100 - speedProcent) / 100;

  float incrDecimal = (float) ROTATE_INCR / 100;

  float totalTime = (float) ROTATE_TIME * decimal;

  return totalTime * incrDecimal;
}

bool servo_procent_rotate(int procent)
{
  if (procent < 0 || procent > 100) return false;

  float decimal = (float) procent / 100;

  int servoAngle = DECIMAL_TO_ANGLE(decimal);

  return servo_angle_rotate(servoAngle);
}

bool servo_negative_procent_rotate(int negativeProcent)
{
  if(negativeProcent < -100 || negativeProcent > 100) return false;

  float decimal = ((float) (negativeProcent + 100) / 200);

  int servoAngle = DECIMAL_TO_ANGLE(decimal);

  return servo_angle_rotate(servoAngle);
}

bool servo_angle_rotate(int servoAngle)
{
  if(servoAngle < 0 || servoAngle > 100) return false;

  servo.write(servoAngle);

  return true;
}

bool motor_rotate(uint8_t motorDPin, uint8_t motorSPin, int procent)
{
  int motorDir = ((procent > 0 ) ? HIGH : LOW);
  int speedProcent = abs(procent);

  if (speedProcent < 0 || speedProcent > 100) return false;

  if (speedProcent == 0)
  {
    analogWrite(motorSPin, 0);
    return true;
  }

  float speedDecimal = (float) speedProcent / 100;

  int motorSpeed = (speedDecimal * (MAX_MSPEED - MIN_MSPEED) + MIN_MSPEED);

  digitalWrite(motorDPin, motorDir);

  analogWrite(motorSPin, motorSpeed);

  return true;
}
