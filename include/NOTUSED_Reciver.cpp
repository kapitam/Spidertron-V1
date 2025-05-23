#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Define servo pins
//#define SERVO_PINFR1 0
//#define SERVO_PINFR2 21
//#define SERVO_PINFR3 4
//#define SERVO_PINCR1 1
//#define SERVO_PINCR2 16
//#define SERVO_PINCR3 17
//#define SERVO_PINBR1 5
//#define SERVO_PINBR2 18
//#define SERVO_PINBR3 19
//#define SERVO_PINFL1 10
//#define SERVO_PINFL2 22
//#define SERVO_PINFL3 13
//#define SERVO_PINCL1 12
//#define SERVO_PINCL2 14
//#define SERVO_PINCL3 27
//#define SERVO_PINBL1 25
//#define SERVO_PINBL2 33
//#define SERVO_PINBL3 32

//Assign Servo on PWM adafruit
#define SERVO_PINFR1 0
#define SERVO_PINFR2 1
#define SERVO_PINFR3 2
#define SERVO_PINCR1 4
#define SERVO_PINCR2 5
#define SERVO_PINCR3 6
#define SERVO_PINBR1 8
#define SERVO_PINBR2 9
#define SERVO_PINBR3 10
#define SERVO_PINFL1 0
#define SERVO_PINFL2 1
#define SERVO_PINFL3 2
#define SERVO_PINCL1 4
#define SERVO_PINCL2 5
#define SERVO_PINCL3 6
#define SERVO_PINBL1 8
#define SERVO_PINBL2 9
#define SERVO_PINBL3 10

const int minPulse = 450;
const int maxPulse = 2500; 

typedef struct test_struct {
  char side;
  int leg;
  double A1;
  double A2;
  double A3;
} test_struct;

test_struct target;

double translator(double angle) {
    double temp = minPulse + (angle / 180.0) * (maxPulse - minPulse);
    return temp * 4096 / (1000000 / 50); // Convert to PWM ticks
}

void UpdatePosition(char side, int leg, double A1, double A2, double A3) {
    
    A1 = translator(180 - A1);
    A2 = translator(A2);
    A3 = translator(180 - A3);

    if (side == 'R') {
        if (leg == 1) {
            //SERVOFR1.write(180 - A1);
            //SERVOFR2.write(A2);
            //SERVOFR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINFR1, 0, A1);
            pwm1.setPWM(SERVO_PINFR2, 0, A3);
            pwm1.setPWM(SERVO_PINFR3, 0, A1);
        } else if (leg == 2) {
            //SERVOCR1.write(180 - A1);
            //SERVOCR2.write(A2);
            //SERVOCR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINCR1, 0, A1);
            pwm1.setPWM(SERVO_PINCR2, 0, A2);
            pwm1.setPWM(SERVO_PINCR3, 0, A3);
        } else if (leg == 3) {
            //SERVOBR1.write(180 - A1);
            //SERVOBR2.write(A2);
            //SERVOBR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINBR1, 0, A1);
            pwm1.setPWM(SERVO_PINBR2, 0, A2);
            pwm1.setPWM(SERVO_PINBR3, 0, A3);
        }
    } else if (side == 'L') {
        if (leg == 1) {
            //SERVOFL1.write(180 - A1);
            //SERVOFL2.write(A2);
            //SERVOFL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINFL1, 0, A1);
            pwm2.setPWM(SERVO_PINFL2, 0, A2);
            pwm2.setPWM(SERVO_PINFL3, 0, A3);
        } else if (leg == 2) {
            //SERVOCL1.write(180 - A1);
            //SERVOCL2.write(A2);
            //SERVOCL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINCL1, 0, A1);
            pwm2.setPWM(SERVO_PINCL2, 0, A2);
            pwm2.setPWM(SERVO_PINCL3, 0, A3);
        } else if (leg == 3) {
            //SERVOBL1.write(180 - A1);
            //SERVOBL2.write(A2);
            //SERVOBL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINBL1, 0, A1);
            pwm2.setPWM(SERVO_PINBL2, 0, A2);
            pwm2.setPWM(SERVO_PINBL3, 0, A3);
        }
    } else if (side == 'M') {
        if (leg == 1) {
            //SERVOFR1.write(180 - A1);
            //SERVOFR2.write(A2);
            //SERVOFR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINFR1, 0, A1);
            pwm1.setPWM(SERVO_PINFR2, 0, A3);
            pwm1.setPWM(SERVO_PINFR3, 0, A1);
        } else if (leg == 2) {
            //SERVOCR1.write(180 - A1);
            //SERVOCR2.write(A2);
            //SERVOCR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINCR1, 0, A1);
            pwm1.setPWM(SERVO_PINCR2, 0, A2);
            pwm1.setPWM(SERVO_PINCR3, 0, A3);
        } else if (leg == 3) {
            //SERVOBR1.write(180 - A1);
            //SERVOBR2.write(A2);
            //SERVOBR3.write(180 - A3);
            pwm1.setPWM(SERVO_PINBR1, 0, A1);
            pwm1.setPWM(SERVO_PINBR2, 0, A2);
            pwm1.setPWM(SERVO_PINBR3, 0, A3);
        } else if (leg == 4) {
            //SERVOFL1.write(180 - A1);
            //SERVOFL2.write(A2);
            //SERVOFL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINFL1, 0, A1);
            pwm2.setPWM(SERVO_PINFL2, 0, A2);
            pwm2.setPWM(SERVO_PINFL3, 0, A3);
        } else if (leg == 5) {
            //SERVOCL1.write(180 - A1);
            //SERVOCL2.write(A2);
            //SERVOCL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINCL1, 0, A1);
            pwm2.setPWM(SERVO_PINCL2, 0, A2);
            pwm2.setPWM(SERVO_PINCL3, 0, A3);
        } else if (leg == 6) {
            //SERVOBL1.write(180 - A1);
            //SERVOBL2.write(A2);
            //SERVOBL3.write(180 - A3);
            pwm2.setPWM(SERVO_PINBL1, 0, A1);
            pwm2.setPWM(SERVO_PINBL2, 0, A2);
            pwm2.setPWM(SERVO_PINBL3, 0, A3);
        } 
    }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.print("Received Data from: ");
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);
  
    Serial.print("Bytes received: ");
    Serial.println(len);
  
    memcpy(&target, incomingData, sizeof(target));

    Serial.printf("Side: %c, Leg: %d, A1: %.2f, A2: %.2f, A3: %.2f\n",

        target.side, target.leg, target.A1, target.A2, target.A3);

    UpdatePosition(target.side, target.leg, target.A1, target.A2, target.A3);
}

void setup(){
    Serial.begin(115200);

    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
    WiFi.mode(WIFI_STA);
    WiFi.channel(1);

    pwm1.begin();
    pwm1.setPWMFreq(50);
    pwm2.begin();
    pwm2.setPWMFreq(50);

    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
       return;
    }

  //SERVOFR1.attach(SERVO_PINFR1);
  //SERVOFR2.attach(SERVO_PINFR2);
  //SERVOFR3.attach(SERVO_PINFR3);

  //SERVOCR1.attach(SERVO_PINCR1);
  //SERVOCR2.attach(SERVO_PINCR2);
  //SERVOCR3.attach(SERVO_PINCR3);

  //SERVOBR1.attach(SERVO_PINBR1);
  //SERVOBR2.attach(SERVO_PINBR2);
  //SERVOBR3.attach(SERVO_PINBR3);

  //SERVOFL1.attach(SERVO_PINFL1);
  //SERVOFL2.attach(SERVO_PINFL2);
  //SERVOFL3.attach(SERVO_PINFL3);

  //SERVOCL1.attach(SERVO_PINCL1);
  //SERVOCL2.attach(SERVO_PINCL2);
  //SERVOCL3.attach(SERVO_PINCL3);

  //SERVOBL1.attach(SERVO_PINBL1);
  //SERVOBL2.attach(SERVO_PINBL2);
  //SERVOBL3.attach(SERVO_PINBL3);


  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver ready.");
}

void loop(){

}