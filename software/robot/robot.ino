/*
   Board (install ESP32 from espressif using Board Manager): ESP32 Dev Module
   Requires the ESP32Servo library.
*/

#define ROBOT_NAME "Kiveris"

// Uncomment your variant.
#define PINS_KMS_2024 1 // Kaunas Makerspace Antweight Workshop October 2024
// #define PINS_KMS_2023 1 // Kaunas Makerspace Antweight Workshop September 2023
// #define PINS_NTA_2023 1 // No Trolls Allowed Antweight Workshop Summer 2023

// Uncomment this if the red motor wire is on the left when looking from the back
// #define KMS_2023_MOTOR_PINS_INVERT 1

#ifdef PINS_KMS_2024
    #define PIN_MOTOR_IN1 27
    #define PIN_MOTOR_IN2 13
    #define PIN_MOTOR_IN3 26
    #define PIN_MOTOR_IN4 25
    #define PIN_SERVO_1_SIG 32
    #define PIN_SERVO_2_SIG 23
#elif PINS_KMS_2023
  #ifndef KMS_2023_MOTOR_PINS_INVERT
    // MR & ML connected correctly (plus on PCB is red wire)
    #define PIN_MOTOR_IN1 26
    #define PIN_MOTOR_IN2 25
    #define PIN_MOTOR_IN3 13
    #define PIN_MOTOR_IN4 27
  #else
    // MR & ML connected incorrectly (plus on PCB is black wire)
    #define PIN_MOTOR_IN1 25
    #define PIN_MOTOR_IN2 26
    #define PIN_MOTOR_IN3 27
    #define PIN_MOTOR_IN4 13
  #endif
  #define PIN_SERVO_1_SIG 32
#elif PINS_NTA_2023
  #define PIN_MOTOR_IN1 19
  #define PIN_MOTOR_IN2 18
  #define PIN_MOTOR_IN3 17
  #define PIN_MOTOR_IN4 16
#endif

// PWM settings
#define MOTOR_PWM_FREQ     5000 // 1 - 50000 Hz
#define MOTOR_PWM_BIT_RES  8 // pwm bit resolution

// BLE settings
#define BLE_SERVICE_UUID        "99b96fd7-dd0e-49cd-b255-f7b692c3de5e"
#define BLE_CHARACTERISTIC_UUID "4fce1dff-9151-498f-aa72-581f3f9241f3"

#ifdef PIN_SERVO_1_SIG || PIN_SERVO_2_SIG
#include <ESP32Servo.h>
#endif

#include "esp32-hal-ledc.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> 

// speed variables
int spd_left = 0;
int spd_right = 0;

// servo variables
int servo1_deg = 0;
int servo2_deg = 0;
int servo1_deg_last = 0;
int servo2_deg_last = 0;

#ifdef PIN_SERVO_1_SIG || PIN_SERVO_2_SIG
Servo servo1;
Servo servo2;
#endif

// BLE
BLEServer *pServer = NULL;


class MyBLEServerCallbacks: public BLEServerCallbacks {
     void onConnect(BLEServer* pServer) {
         Serial.println("Connection established");
     };

     void onDisconnect(BLEServer* pServer) {
         Serial.println("Disconnected!");
         pServer->startAdvertising();
     }
 };

class MyBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      sscanf(value.c_str(), "ML%d MR%d SL%d SR%d", &spd_left, &spd_right, &servo2_deg, &servo1_deg);
      
      Serial.print("New speed values: L=");
      Serial.print(spd_left);
      Serial.print(" R=");
      Serial.print(spd_right);
      Serial.print(" SL=");
      Serial.print(servo2_deg);
      Serial.print(" SR=");
      Serial.print(servo1_deg);
      Serial.println();
    }
  }
};

void setup_pwm() {
  ledcAttach(PIN_MOTOR_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOTOR_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOTOR_IN3, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOTOR_IN4, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
}

void setup_servo() {
#ifdef PIN_SERVO_1_SIG
  servo1.setPeriodHertz(50);// Standard 50Hz servo
  servo1.attach(PIN_SERVO_1_SIG, 500, 2400);
#endif
#ifdef PIN_SERVO_2_SIG
  servo2.setPeriodHertz(50);// Standard 50Hz servo
  servo2.attach(PIN_SERVO_2_SIG, 500, 2400);
#endif
}

void setup_ble() {
   BLEDevice::init(ROBOT_NAME);
   pServer = BLEDevice::createServer();
   BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
   BLECharacteristic *pCharacteristic = pService->createCharacteristic(BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
   
   pServer->setCallbacks(new MyBLEServerCallbacks());
   pCharacteristic->setCallbacks(new MyBLECallbacks());
   pCharacteristic->setValue("ML0 MR0 SL0 SR0");
   pService->start();

   BLEAdvertising *pAdvertising = pServer->getAdvertising();
   pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
   // pAdvertising->setScanResponse(false);
   // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
   pAdvertising->start();
}

void set_speed_left(int spd) {
  int pin1 = PIN_MOTOR_IN3;
  int pin2 = PIN_MOTOR_IN4;
  if (spd < 0) {
    pin1 = PIN_MOTOR_IN4;
    pin2 = PIN_MOTOR_IN3;
    spd *= -1;
  }
  ledcWrite(pin1, 255);
  ledcWrite(pin2, 255 - spd);
}

void set_speed_right(int spd) {
  int pin1 = PIN_MOTOR_IN1;
  int pin2 = PIN_MOTOR_IN2;
  if (spd < 0) {
    pin1 = PIN_MOTOR_IN2;
    pin2 = PIN_MOTOR_IN1;
    spd *= -1;
  }
  ledcWrite(pin1, 255);
  ledcWrite(pin2, 255 - spd);
}

void setup() {
  Serial.begin(115200);
  Serial.println("PWM setup...");
  setup_pwm();
  Serial.println("Servo setup...");
  setup_servo();
  Serial.println("BLE setup...");
  setup_ble();
  Serial.println("Ready!");
}

void loop() {
  set_speed_left(constrain(spd_left, -255, 255));
  set_speed_right(constrain(spd_right, -255, 255));

#ifdef PIN_SERVO_1_SIG
  if (servo1_deg_last != servo1_deg) {
    servo1.write(constrain(servo1_deg, 0, 180));
    servo1_deg_last = servo1_deg;
  }
#endif

#ifdef PIN_SERVO_2_SIG
  if (servo2_deg_last != servo2_deg) {
    servo2.write(constrain(servo2_deg, 0, 180));
    servo2_deg_last = servo2_deg;
  }
#endif
}
