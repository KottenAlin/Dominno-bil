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

#define ANGLE_TO_PROCENT(ANGLE) ((float) ((ANGLE) - (MIN_SANGLE)) / (float) ((MAX_SANGLE) - (MIN_SANGLE)) * 100)
#define PROCENT_TO_ANGLE(PROCENT) (((float) (PROCENT) / 100) * (float) ((MAX_SANGLE) - (MIN_SANGLE)) + (MIN_SANGLE))

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
DynamicJsonDocument messageJson(300);
DynamicJsonDocument statusJson(300);

char rotateDir[64];
int rotateSpeed = 0;
int driveSpeed = 0;

char messageString[256];
char statusString[256];

char buffer[1024];

void setup()
{
  Serial.begin(BAUD_VALUE);

  if(!wifi_connect(WIFI_SSID, WIFI_PASSWORD))
  {
    Serial.println("ERROR: WIFI connect");
    exit(1);
  }
  else Serial.println("SUCCESS: WIFI connect");

  timeClient.begin();
  
  if(!motor_servo_setup())
  {
    Serial.println("ERROR: Motor and servo setup");
    exit(2);
  }
  else Serial.println("SUCCESS: Motor and servo setup");
  
  if(!mqtt_setup())
  {
    Serial.println("ERROR: MQTT setup");
    exit(3);
  }
  else Serial.println("SUCCESS: MQTT setup");
}

/*
 * RETURN
 * bool result | Successful setup
 */
bool motor_servo_setup(void)
{
  pinMode(MOTOR_DPIN, OUTPUT);
  pinMode(MOTOR_SPIN, OUTPUT);

  if(servo.attach(SERVO_PIN) == 0) return false;

  return true;
}

/*
 *
 */
bool mqtt_setup(void)
{
  sprintf(buffer, "Connecting to broker: (%s:%d)", client.getMqttServerIp(), client.getMqttServerPort());
  Serial.println(buffer);

  client.enableDebuggingMessages();
  client.enableHTTPWebUpdater();
  client.enableOTA();

  char* message = message_json_create("offline");
  client.enableLastWillMessage("hampus/arduino-message", message);

  // return client.isConnected(); -- I don't know why this is failing?
  return true;
}

/*
 * PARAMS
 * - const char ssid[]     | The WIFI SSID
 * - const char password[] | The WIFI password
 *
 * RETURN
 * bool result | It successfully connected to the WIFI
 */
bool wifi_connect(const char ssid[], const char password[])
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

/*
 * 
 */
void onConnectionEstablished(void)
{
  message_publish("Connected to broker");

  client.subscribe("hampus/command", [](const String & payload)
  {
    DeserializationError error = deserializeJson(commandJson, payload);

    if (error)
    {
      sprintf(buffer, "Json failed: (%s)", error.f_str());
      Serial.println(buffer);

      message_publish("Error parsing JSON");
    }
    else 
    {
      command_json_parse();

      // =========================
      Serial.println(rotateDir);
      Serial.println(rotateSpeed);
      Serial.println(driveSpeed);
      // =========================

      status_publish();
    }
  });
}

/*
 *
 */
void command_json_parse(void)
{
  // rotate dir
  const char* jsonRotateDir = commandJson["rotateDir"];

  if(jsonRotateDir != NULL) strcpy(rotateDir, jsonRotateDir);

  // drive speed
  int jsonDriveSpeed = commandJson["driveSpeed"] | -1;

  if(jsonDriveSpeed != -1) driveSpeed = jsonDriveSpeed;

  // rotate speed
  int jsonRotateSpeed = commandJson["rotateSpeed"] | -1;

  if(jsonRotateSpeed != -1) rotateSpeed = jsonRotateSpeed;
}

/*
 * Publish the values of
 * - angle | The angle of the servo
 * - speed | The speed of the motor
 * - time  | Current Epoch Time
 */
void status_publish(void)
{
  statusJson.clear();
  timeClient.update();

  statusJson["angle"] = servo.read();
  statusJson["speed"] = analogRead(MOTOR_SPIN);
  statusJson["time"] = timeClient.getEpochTime();

  serializeJson(statusJson, statusString);

  client.publish("hampus/arduino-status", statusString);
}

/*
 * PARAMS
 * - const char message[] | The message to publish
 *
 * RETURN
 * - char* string | A pointer to the message string
 */
char* message_json_create(const char message[])
{
  messageJson.clear(); 
  timeClient.update();

  messageJson["message"] = message;
  messageJson["time"] = timeClient.getEpochTime();

  serializeJson(messageJson, messageString);

  return messageString;
}

/*
 * PARAMS
 * - const char message[] | The message to publish
 */
void message_publish(const char message[])
{
  char* messageString = message_json_create(message);

  client.publish("hampus/arduino-message", messageString);
}

void loop()
{
  // Refresh the values sent from the website
  client.loop();

  if(driveSpeed >= 0)
  {
    motor_rotate(MOTOR_DPIN, MOTOR_SPIN, driveSpeed);
  }
  if(rotateDir != NULL && rotateSpeed >= 0)
  {
    // Maybe change rotateDir to rotateBool and just call servo_rotate
    servo_rotate_parse(rotateDir, rotateSpeed);
  }
}

/*
 * PARAMS
 * - const char dir[] |
 * - int speed        |
 */
void servo_rotate_parse(const char dir[], int speed)
{
  if(!strcmp(dir, "west"))
  {
    servo_rotate(speed, true);
  }
  else if(!strcmp(dir, "east"))
  {
    servo_rotate(speed, false);
  }
}
/*
 * PARAMS
 * - int speed |
 * - int dir   |
 */
void servo_rotate(int speed, bool dir)
{
  // The procent to start rotating from
  int start = ANGLE_TO_PROCENT(servo.read());

  // The procent to rotate to
  int procent = dir ? (start - ROTATE_INCR) : (start + ROTATE_INCR);

  // Rotate to the corresponding angle
  servo.write(PROCENT_TO_ANGLE(procent));

  // Wait for some time depending on the rotate speed
  delay(servo_delay(speed));
}
/*
 * PARAMS
 * - int procent |
 */
float servo_delay(int procent)
{
  if(procent < 0 || procent > 100) return 0;
  
  float decimal = (float) (100 - procent) / 100;

  float incrDecimal = (float) ROTATE_INCR / 100;

  float totalTime = (float) ROTATE_TIME * decimal;

  return totalTime * incrDecimal;
}

#define PROCENT_TO_VALUE(PROCENT) ((float) PROCENT / 100 * (MAX_MSPEED - MIN_MSPEED) + MIN_MSPEED)

/*
 * PARAMS
 * - dpin    |
 * - spin    |
 * - procent |
 */
void motor_rotate(uint8_t dpin, uint8_t spin, int procent)
{
  int value = (procent == 0) ? 0 : PROCENT_TO_VALUE(procent);

  digitalWrite(dpin, HIGH);

  analogWrite(spin, value);
}
