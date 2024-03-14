#ifndef ARDUINO_H
#define ARDUINO_H

#include "EspMQTTClient.h"
#include <Servo.h>
#include <VL53L0X.h>
#include "ArduinoJson.h"
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const unsigned long BAUD_VALUE = 9600;
const unsigned int LOOP_DELAY  = 1000;

// WIFI constants
#define WIFI_SSID     "ABBgym_2.4"
#define WIFI_PASSWORD "mittwifiarsabra"

const unsigned int CONNECT_TRIES = 32;
const unsigned int WIFI_DELAY    = 1000;

// MQTT constants
#define MQTT_ADDRESS "10.22.4.26"
#define MQTT_PORT    1883

// Servo constants
const int SERVO_PIN = D7;

const unsigned int SERVO_ANGLE_MAX = 180;
const unsigned int SERVO_ANGLE_MIN = 0;

// Motor constants
const unsigned int MOTOR_SPEED_MAX = 400;
const unsigned int MOTOR_SPEED_MIN = 120;

// Drive constants
const uint8_t DRIVE_DPIN = D3;
const uint8_t DRIVE_SPIN = D1;

const unsigned int DRIVE_DELAY_MAX = 900;
const unsigned int DRIVE_DELAY_MIN = 800;

// Domino constants
const uint8_t DOMINO_DPIN = D4;
const uint8_t DOMINO_SPIN = D2;

const unsigned int DOMINO_DELAY        = 1000;
const unsigned int DOMINO_WAIT_DELAY   = 1000;

const unsigned int DOMINO_PLACE_SPEED  = 50;
const unsigned int DOMINO_RETURN_SPEED = 100;

#endif // ARDUINO_H
