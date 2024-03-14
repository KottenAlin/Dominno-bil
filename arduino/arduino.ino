#include "arduino.h"

WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "pool.ntp.org");

Servo servo;

EspMQTTClient client(
  WIFI_SSID,     // WIFI SSID
  WIFI_PASSWORD, // WIFI Password
  MQTT_ADDRESS,  // MQTT Broker server ip
  "",            // Can be omitted if not needed
  "",            // Can be omitted if not needed
  "domino-car",  // Client name that uniquely identify your device
  MQTT_PORT      // MQTT Broker server port
);

DynamicJsonDocument commandJson(300);
DynamicJsonDocument messageJson(300);
DynamicJsonDocument statusJson(300);

int turnProcent = 50;  // The procent rotation of the drive (0 - 100)
int driveSpeed   = 100; // The speed which to drive forward   (0 - 100)
int dominoSpace  = 100; // The space between domino bricks    (0 - 100)

char messageJsonString[256];
char statusJsonString[256];

char buffer[1024];

void setup()
{
  Serial.begin(BAUD_VALUE);

  if(wifi_connect(WIFI_SSID, WIFI_PASSWORD) != 0)
  {
    Serial.println("ERROR: WIFI connect");
    exit(1);
  }
  else Serial.println("SUCCESS: WIFI connect");

  timeClient.begin();
  
  if(motor_servo_setup() != 0)
  {
    Serial.println("ERROR: Motor and servo setup");
    exit(2);
  }
  else Serial.println("SUCCESS: Motor and servo setup");
  
  if(mqtt_setup() != 0)
  {
    Serial.println("ERROR: MQTT setup");
    exit(3);
  }
  else Serial.println("SUCCESS: MQTT setup");
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
 * RETURN (int status)
 * - 0 | Success!
 */
int mqtt_setup(void)
{
  sprintf(buffer, "Connecting to broker: (%s:%d)", client.getMqttServerIp(), client.getMqttServerPort());
  Serial.println(buffer);

  client.enableDebuggingMessages();
  client.enableHTTPWebUpdater();
  client.enableOTA();

  char* message = message_json_create("offline");
  client.enableLastWillMessage("hampus/arduino-message", message);

  // return client.isConnected(); -- I don't know why this is failing?
  return 0;
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

  for (int index = 0; index <= CONNECT_TRIES; index += 1)
  {
    if (WiFi.status() == WL_CONNECTED) return 0;

    delay(WIFI_DELAY);
  }
  return 1;
}

/*
 * When connection to the broker is established,
 * subscribe to the hampus/command topic
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
      Serial.println(turnProcent);
      Serial.println(driveSpeed);
      Serial.println(dominoSpace);
      // =========================

      status_publish();
    }
  });
}

/*
 * Parse the command json string, to update the driving values
 *
 * If the json object does not contain a specific value (ex driveSpeed),
 * do not update the value, continue with the existing value
 */
void command_json_parse(void)
{
  // rotate amount
  int jsonTurnProcent = commandJson["turnProcent"] | -1;

  if(jsonTurnProcent != -1) turnProcent = jsonTurnProcent;

  // drive speed
  int jsonDriveSpeed = commandJson["driveSpeed"] | -1;

  if(jsonDriveSpeed != -1) driveSpeed = jsonDriveSpeed;

  // domino space
  int jsonDominoSpace = commandJson["dominoSpace"] | -1;

  if(jsonDominoSpace != -1) dominoSpace = jsonDominoSpace;
}

#define SERVO_ANGLE_PROCENT(ANGLE) ((float) ((ANGLE) - (SERVO_ANGLE_MIN)) / (float) ((SERVO_ANGLE_MAX) - (SERVO_ANGLE_MIN)) * 100)
#define MOTOR_SPEED_PROCENT(SPEED) ((float) ((SPEED) - (MOTOR_SPEED_MIN)) / (float) ((MOTOR_SPEED_MAX) - (MOTOR_SPEED_MIN)) * 100)

/*
 * Publish the values of
 * - angle | The angle of the servo
 * - speed | The speed of the motor
 * - time  | The current epoch time
 */
void status_publish(void)
{
  statusJson.clear();
  timeClient.update();

  statusJson["angle"] = SERVO_ANGLE_PROCENT(servo.read());
  statusJson["speed"] = MOTOR_SPEED_PROCENT(analogRead(DRIVE_SPIN));
  statusJson["time"] = timeClient.getEpochTime();

  serializeJson(statusJson, statusJsonString);

  client.publish("hampus/arduino-status", statusJsonString);
}

/*
 * Create the message json object, 
 * containing the inputted message and the current time
 *
 * PARAMS
 * - const char* message | The message to publish
 *
 * RETURN
 * - char* string | A pointer to the message string
 */
char* message_json_create(const char* message)
{
  messageJson.clear(); 
  timeClient.update();

  messageJson["message"] = message;
  messageJson["time"] = timeClient.getEpochTime();

  serializeJson(messageJson, messageJsonString);

  return messageJsonString;
}

/*
 * Publish the inputted message to the hampus/arduino-message topic,
 * after first creating the message json object from the message string
 *
 * PARAMS
 * - const char* message | The message to publish
 */
void message_publish(const char* message)
{
  char* messageJsonString = message_json_create(message);

  client.publish("hampus/arduino-message", messageJsonString);
}

/*
 * The loop is running every tick
 */
void loop()
{
  // Refresh the values sent from the website
  if(client.isConnected()) client.loop();

  if(driveSpeed >= 0)
  {
    // 1. Place a domino brick on the floor
    domino_place();

    // 2. Drive forward leaving the domino standing
    wheels_drive();
  }
  if(turnProcent >= 0)
  {
    wheels_turn();
  }
}

// SPEED is in procent
#define DOMINO_SPEED_DELAY(SPEED) (100 / (SPEED) * DOMINO_DELAY)

/*
 * Place down a domino brick on the floor
 */
void domino_place(void)
{
  // 1. Close the door and rotate the feeder
  domino_motor_rotate(DOMINO_PLACE_SPEED);

  delay(DOMINO_SPEED_DELAY(DOMINO_PLACE_SPEED));

  // 2. Stop the rotation and let domino stay still
  domino_motor_rotate(0);

  delay(DOMINO_WAIT_DELAY);

  // 3. Open the door and return the feeder
  domino_motor_rotate(-DOMINO_RETURN_SPEED);

  delay(DOMINO_SPEED_DELAY(DOMINO_RETURN_SPEED));

  // 4. Stop the rotation when driving
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
  int driveDelay = DRIVE_PROCENT_DELAY(dominoSpace);

  delay((100 / (float) driveSpeed) * driveDelay);

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
