/*
  This code runs vixen through the arduino mega 2560 with a PCA9895 driver board attached via I2C

  Adapted from code by GSRVAhiker17, zparticle, victor_pv, Si_champion, Neil Tapp, and Richard Sloan.
*/
#include <Wire.h>     // I2C driver
#include <Adafruit_PWMServoDriver.h> // PCA9685 driver library
#include <Servo.h>    // Servos
#include <avr/wdt.h>  // Raw AVR watchdog timer
#include <FastLED.h>  // WS2812B driver

// Testing mode (treat servos as simple PWM)
const bool TESTINGMODE = false;

// Vixen header (identifies a light sequence)
const int HEADER_LEN = 2;
const char seqHeader[] = {'~', '!'};

// Timeout waiting for serial input before going to random mode (in milliseconds).
const int TIME_OUT = 1000;

// Servo channels mapped to PCA9685 pins
const int PUMPKIN1_MOUTH = 15;
const int PUMPKIN2_MOUTH = 13;
const int PUMPKIN3_MOUTH = 12;
const int PUMPKIN1_FANGS = 14;
const int TRUMPET = 9;
const int TRUMPET_ARM = 11;
const int SKULL = 10;

// Channels mapped to Mega 2560 pin numbers
const int PUMPKIN1_EYES_R  = 2;
const int PUMPKIN1_EYES_G = 3;
const int PUMPKIN1_EYES_B = 4;

const int PUMPKIN2_EYES_R  = 5;
const int PUMPKIN2_EYES_G = 6;
const int PUMPKIN2_EYES_B = 7;

const int PUMPKIN3_EYES_R  = 8;
const int PUMPKIN3_EYES_G = 9;
const int PUMPKIN3_EYES_B = 10;

const int BLACKLIGHT = 11;
const int LED_STRIP = 12;
const int LIGHTS1 = 28;
const int LIGHTS2 = 30;
const int LIGHTS3 = 32;
const int LIGHTS4 = 34;
const int CHAN11 = 29;
const int CHAN12 = 31;

// Servo channels directly controlled by Arduino
const int NUM_ARD_SERVOS = 0;
const int ardServoChannels[] = {};

// Servo channels controlled via PCA9685
const int NUM_PCA_SERVOS = 7;
const int PCAServoChannels[] = {
  PUMPKIN1_MOUTH, PUMPKIN2_MOUTH, PUMPKIN3_MOUTH, PUMPKIN1_FANGS,
  TRUMPET, TRUMPET_ARM, SKULL
};

// PWM Channels
const int NUM_PWM = 1;
const int PWMChannels[] = {BLACKLIGHT};

// Inverted PWM Channels
const int NUM_INV_PWM = 9;
const int invPWMChannels[] = {
  PUMPKIN1_EYES_R, PUMPKIN1_EYES_G, PUMPKIN1_EYES_B,
  PUMPKIN2_EYES_R, PUMPKIN2_EYES_G, PUMPKIN2_EYES_B,
  PUMPKIN3_EYES_R, PUMPKIN3_EYES_G, PUMPKIN3_EYES_B,
};

// Digital channels
const int NUM_DIGITAL = 2;
const int digitalChannels[] = {LIGHTS1, LIGHTS2};

// UART channels
const int NUM_UART = 1;

// LED channels
const int NUM_LED = 53;

// Total channels
const int NUM_ACTIVE_CHANNELS = 179;

// Servos
const int SERVO_DELAY = 15; // delay after servo is activated (allow it to move)
Servo servos[NUM_ARD_SERVOS];

// Neutral servo position in degrees and pulse length
const int ardServoNeutral[] = {};
const int PCAServoNeutral[] = {350, 320, 310, 347, 190, 300, 325};

// Min servo opening in degrees and pulse lengths
const int ardServoMin[] = {};
const int PCAServoMin[] = {260, 320, 310, 225, 190, 300, 325};

// Max servo opening in degrees
const int ardServoMax[] = {};
const int PCAServoMax[] = {350, 400, 410, 347, 280, 480, 395};

// Servo direction
const int CLOCKWISE = 1;
const int COUNTERCLOCKWISE = -1;
const int ardServoDirection[] = {};
const int PCAServoDirection[] = {COUNTERCLOCKWISE, CLOCKWISE, CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE, CLOCKWISE, CLOCKWISE};

// Serial
const long COM_SPEED = 115200;
const long OUT_COM_SPEED = 4800;
int incomingByte;

// PCA9895 driver
const int SERVO_FREQ = 60;      // PWM frequency for servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// LED strip
CRGB leds[NUM_LED];

// PA driven skull
const int PA_SKULL = 45;
const int PA_MIC = 49;
const int micReadDelay = 25;
const int servoMoveDelay = 50;
Servo paSkullServo;
const int paSkullMin = 0;
const int paSkullMax = 40;
int micVal = 0;
unsigned long micReadTimer = 0;
unsigned long servoMoveTimer = 0;

// Misc
int i = 0;                              // Loop counter
int j = 0;                              // Loop counter
volatile unsigned long  timer_a = 0;    // Timer


//setup the pins/ inputs & outputs
void setup()
{
  // enable the watchdog timer with a time of 1 second. If the board freezes, it will reset itself after 1 second.
  //wdt_enable(WDTO_1S);

  // specifically for the UNO
  sei();

  // initialize I2C interface for PCA9895
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(400000); // This is probably max I2C speed

  // Turn on output UART
  Serial1.begin(OUT_COM_SPEED);

  // initalize channels and set to starting state
  for (i = 0; i < NUM_ARD_SERVOS; i++) {
    if (TESTINGMODE) {
      pinMode(ardServoChannels[i], OUTPUT);
      digitalWrite(ardServoChannels[i], LOW);
    } else {
      servos[i].attach(ardServoChannels[i]);
      servos[i].write(ardServoNeutral[i]);
    }
  }

  for (i = 0; i < NUM_PCA_SERVOS; i++) {
    if (TESTINGMODE) {
      pwm.setPin(PCAServoChannels[i], 0);
    } else {
      pwm.setPin(PCAServoChannels[i], PCAServoNeutral[i]);
    }
  }

  for (i = 0; i < NUM_PWM; i++) {
    pinMode(PWMChannels[i], OUTPUT);
    digitalWrite(PWMChannels[i], LOW);
  }

  for (i = 0; i < NUM_INV_PWM; i++) {
    pinMode(invPWMChannels[i], OUTPUT);
    if (TESTINGMODE) {
      digitalWrite(invPWMChannels[i], LOW);
    } else {
      digitalWrite(invPWMChannels[i], HIGH);
    }
  }

  for (i = 0; i < NUM_DIGITAL; i++) {
    pinMode(digitalChannels[i], OUTPUT);
    digitalWrite(digitalChannels[i], LOW);
  }

  for (i = 0; i < NUM_UART; i++) {
    Serial1.write(0);
  }

  FastLED.addLeds<WS2812B, LED_STRIP, GRB>(leds, NUM_LED);
  for (i = 0; i < NUM_LED; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

  // Initialize PA skull
  pinMode(PA_MIC, INPUT);
  paSkullServo.attach(PA_SKULL);
  paSkullServo.write(paSkullMin);

  test_sequence(); // brief test
  Serial.begin(COM_SPEED);   // set up input serial
}

void loop()
{
  if (Serial.available() > 0) {
    //wdt_reset(); // resets the watchdog (prevents board lockup)
    timer_a = millis ();  // Mark the time when a message was received

    // read the header to verify this is in fact a light sequence
    for (int i = 0; i < HEADER_LEN; i++) {
      while (!Serial.available());
      if (seqHeader[i] != Serial.read()) {
        return;
      }
    }

    // Arduino servos
    for (i = 0; i < NUM_ARD_SERVOS; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      if (TESTINGMODE) {
        analogWrite(ardServoChannels[i], incomingByte);
      } else {
        int angle = map(incomingByte, 0, 255, ardServoMin[i], ardServoMax[i]);
        angle *= ardServoDirection[i];
        servos[i].write(ardServoNeutral[i] + angle);
        delay(SERVO_DELAY);
      }
    }

    // PCA9895 servos
    for (i = 0; i < NUM_PCA_SERVOS; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      if (TESTINGMODE) {
        int pulselen = map(incomingByte, 0, 255, 0, 4095);
        pwm.setPin(PCAServoChannels[i], pulselen);
      } else {
        if (PCAServoDirection[i] == COUNTERCLOCKWISE) {
          incomingByte = 255 - incomingByte;
        }
        int pulselen = map(incomingByte, 0, 255, PCAServoMin[i], PCAServoMax[i]);
        pwm.setPin(PCAServoChannels[i], pulselen);
      }
    }

    // PWM
    for (i = 0; i < NUM_PWM; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      analogWrite(PWMChannels[i], incomingByte);
    }

    // Inverted PWM
    for (i = 0; i < NUM_INV_PWM; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      if (TESTINGMODE) {
        analogWrite(invPWMChannels[i], incomingByte);
      } else {
        analogWrite(invPWMChannels[i], 255 - incomingByte);
      }
    }

    // Digital
    for (i = 0; i < NUM_DIGITAL; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      if (incomingByte <= 127) {
        digitalWrite(digitalChannels[i], LOW);
      } else {
        digitalWrite(digitalChannels[i], HIGH);
      }
    }

    // UART
    for (i = 0; i < NUM_UART; i++) {
      while (!Serial.available());
      incomingByte = Serial.read();
      Serial1.write(incomingByte);
    }

    // LED strip
    for (i = 0; i < NUM_LED; i++) {
      while (!Serial.available());
      leds[i].r = Serial.read();
      while (!Serial.available());
      leds[i].g = Serial.read();
      while (!Serial.available());
      leds[i].b = Serial.read();
    }
    FastLED.show();



  } else {
    // Random mode starts if no serial input has been received in TIME_OUT milliseconds
    //wdt_reset(); // resets the watchdog (prevents board lockup)
    unsigned long diff = millis() - timer_a;
    if (diff >= TIME_OUT) {
      timer_a = millis ();

      if (TESTINGMODE) {
        for (i = 0; i < NUM_ARD_SERVOS; i++) {
          random_light(ardServoChannels[i]);
        }
        for (i = 0; i < NUM_PCA_SERVOS; i++) {
          random_light_pca(PCAServoChannels[i]);
        }
      }

      for (i = 0; i < NUM_PWM; i++) {
        random_light(PWMChannels[i]);
      }
      for (i = 0; i < NUM_INV_PWM; i++) {
        random_light(invPWMChannels[i]);
      }
      for (i = 0; i < NUM_DIGITAL; i++) {
        random_light(digitalChannels[i]);
      }
      random_strip();
    }
  }
  
  // PA skull
  unsigned long now = millis();
  if (((now - micReadTimer) > micReadDelay) &&
      ((now - servoMoveTimer) > servoMoveDelay)) {
    micVal = digitalRead(PA_MIC);
    if (micVal) {
      paSkullServo.write(paSkullMax);
      servoMoveTimer = millis();
    } else {
      paSkullServo.write(paSkullMin);
    }
  }
}

// Test the setup briefly
void test_sequence() {
  if (TESTINGMODE) {
    for (i = 0; i < NUM_ARD_SERVOS; i++) {
      test_channel(ardServoChannels[i]);
    }
    for (i = 0; i < NUM_PCA_SERVOS; i++) {
      test_channel_pca(PCAServoChannels[i]);
    }
  }

  for (i = 0; i < NUM_PWM; i++) {
    test_channel(PWMChannels[i]);
  }
  for (i = 0; i < NUM_INV_PWM; i++) {
    test_channel(invPWMChannels[i]);
  }
  for (i = 0; i < NUM_DIGITAL; i++) {
    test_channel(digitalChannels[i]);
  }

  for (i = 0; i < NUM_LED; i++) {
    leds[i] = CRGB(255, 255, 255);
    FastLED.show();
  }
}

// Activate channel for time and then deactivate
void test_channel(int channel) {
  wdt_reset(); // resets the watchdog
  digitalWrite(channel, HIGH);
  delay (500);
  digitalWrite(channel, LOW);
}

// Activate PCA driver channel for time and then deactivate
void test_channel_pca(int channel) {
  wdt_reset(); // resets the watchdog
  pwm.setPin(channel, 4095);
  delay (500);
  pwm.setPin(channel, 0);
}

// Random light
void random_light(int channel) {
  int random_a = random(0, 2);
  if (random_a == 0) {
    digitalWrite(channel, LOW);
  } else {
    digitalWrite(channel, HIGH);
  }
}

// Random light on PCA driver
void random_light_pca(int channel) {
  int random_a = random(0, 2);
  if (random_a == 0) {
    pwm.setPin(channel, 0);
  } else {
    pwm.setPin(channel, 4095);
  }
}

// Random sequence on LED strip
void random_strip() {
  for (i = 0; i < NUM_LED; i++) {
    int random_r = random(0, 25);
    int random_g = random(0, 25);
    int random_b = random(0, 25);
    leds[i] = CRGB(random_r, random_g, random_b);
  }
  FastLED.show();
}
