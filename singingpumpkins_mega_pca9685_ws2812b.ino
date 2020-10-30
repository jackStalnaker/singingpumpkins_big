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
const char SEQ_HEADER[] = {'~', '!'};

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
const int ARD_SERVO_CHANNELS[] = {};

// Servo channels controlled via PCA9685
const int NUM_PCA_SERVOS = 7;
const int PCA_SERVO_CHANNELS[] = {
  PUMPKIN1_MOUTH, PUMPKIN2_MOUTH, PUMPKIN3_MOUTH, PUMPKIN1_FANGS,
  TRUMPET, TRUMPET_ARM, SKULL
};

// PWM Channels
const int NUM_PWM = 1;
const int PWM_CHANNELS[] = {BLACKLIGHT};

// Inverted PWM Channels
const int NUM_INV_PWM = 9;
const int INV_PWM_CHANNELS[] = {
  PUMPKIN1_EYES_R, PUMPKIN1_EYES_G, PUMPKIN1_EYES_B,
  PUMPKIN2_EYES_R, PUMPKIN2_EYES_G, PUMPKIN2_EYES_B,
  PUMPKIN3_EYES_R, PUMPKIN3_EYES_G, PUMPKIN3_EYES_B,
};

// Digital channels
const int NUM_DIGITAL = 2;
const int DIGITAL_CHANNELS[] = {LIGHTS1, LIGHTS2};

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
const int ARD_SERVO_NEUTRAL[] = {};
const int PCA_SERVO_NEUTRAL[] = {350, 320, 310, 347, 190, 300, 325};

// Min servo opening in degrees and pulse lengths
const int ARD_SERVO_MIN[] = {};
const int PCA_SERVO_MIN[] = {260, 320, 310, 225, 190, 300, 325};

// Max servo opening in degrees
const int ARD_SERVO_MAX[] = {};
const int PCA_SERVO_MAX[] = {350, 400, 410, 347, 280, 480, 395};

// Servo direction
const int CLOCKWISE = 1;
const int COUNTERCLOCKWISE = -1;
const int ARD_SERVO_DIRECTION[] = {};
const int PCA_SERVO_DIRECTION[] = {COUNTERCLOCKWISE, CLOCKWISE, CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE, CLOCKWISE, CLOCKWISE};

// Serial
const long COM_SPEED = 115200;
const long OUT_COM_SPEED = 4800;
int incoming_byte;

// PCA9895 driver
const int SERVO_FREQ = 60;      // PWM frequency for servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// LED strip
CRGB leds[NUM_LED];

// PA driven skull
const int PA_SKULL = 45;
const int PA_MIC = 49;
const int PA_MIC_READ_DELAY = 25;
const int PA_SERVO_MOVE_DELAY = 50;
const int PA_SKULL_MIN = 0;
const int PA_SKULL_MAX = 30;
Servo pa_skull_servo;
int mic_val = 0;
unsigned long mic_read_timer = 0;
unsigned long servo_move_timer = 0;

// Misc
int i = 0;                                  // Loop counter
int j = 0;                                  // Loop counter
volatile unsigned long last_msg_time = 0;   // Timer


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
      pinMode(ARD_SERVO_CHANNELS[i], OUTPUT);
      digitalWrite(ARD_SERVO_CHANNELS[i], LOW);
    } else {
      servos[i].attach(ARD_SERVO_CHANNELS[i]);
      servos[i].write(ARD_SERVO_NEUTRAL[i]);
    }
  }

  for (i = 0; i < NUM_PCA_SERVOS; i++) {
    if (TESTINGMODE) {
      pwm.setPin(PCA_SERVO_CHANNELS[i], 0);
    } else {
      pwm.setPin(PCA_SERVO_CHANNELS[i], PCA_SERVO_NEUTRAL[i]);
    }
  }

  for (i = 0; i < NUM_PWM; i++) {
    pinMode(PWM_CHANNELS[i], OUTPUT);
    digitalWrite(PWM_CHANNELS[i], LOW);
  }

  for (i = 0; i < NUM_INV_PWM; i++) {
    pinMode(INV_PWM_CHANNELS[i], OUTPUT);
    if (TESTINGMODE) {
      digitalWrite(INV_PWM_CHANNELS[i], LOW);
    } else {
      digitalWrite(INV_PWM_CHANNELS[i], HIGH);
    }
  }

  for (i = 0; i < NUM_DIGITAL; i++) {
    pinMode(DIGITAL_CHANNELS[i], OUTPUT);
    digitalWrite(DIGITAL_CHANNELS[i], LOW);
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
  pa_skull_servo.attach(PA_SKULL);
  pa_skull_servo.write(PA_SKULL_MIN);

  test_sequence(); // brief test
  Serial.begin(COM_SPEED);   // set up input serial
}

void loop()
{
  if (Serial.available() > 0) {
    //wdt_reset(); // resets the watchdog (prevents board lockup)
    last_msg_time = millis ();  // Mark the time when a message was received

    // read the header to verify this is in fact a light sequence
    for (int i = 0; i < HEADER_LEN; i++) {
      while (!Serial.available());
      if (SEQ_HEADER[i] != Serial.read()) {
        return;
      }
    }

    // Arduino servos
    for (i = 0; i < NUM_ARD_SERVOS; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      if (TESTINGMODE) {
        analogWrite(ARD_SERVO_CHANNELS[i], incoming_byte);
      } else {
        int angle = map(incoming_byte, 0, 255, ARD_SERVO_MIN[i], ARD_SERVO_MAX[i]);
        angle *= ARD_SERVO_DIRECTION[i];
        servos[i].write(ARD_SERVO_NEUTRAL[i] + angle);
        delay(SERVO_DELAY);
      }
    }

    // PCA9895 servos
    for (i = 0; i < NUM_PCA_SERVOS; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      if (TESTINGMODE) {
        int pulselen = map(incoming_byte, 0, 255, 0, 4095);
        pwm.setPin(PCA_SERVO_CHANNELS[i], pulselen);
      } else {
        if (PCA_SERVO_DIRECTION[i] == COUNTERCLOCKWISE) {
          incoming_byte = 255 - incoming_byte;
        }
        int pulselen = map(incoming_byte, 0, 255, PCA_SERVO_MIN[i], PCA_SERVO_MAX[i]);
        pwm.setPin(PCA_SERVO_CHANNELS[i], pulselen);
      }
    }

    // PWM
    for (i = 0; i < NUM_PWM; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      analogWrite(PWM_CHANNELS[i], incoming_byte);
    }

    // Inverted PWM
    for (i = 0; i < NUM_INV_PWM; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      if (TESTINGMODE) {
        analogWrite(INV_PWM_CHANNELS[i], incoming_byte);
      } else {
        analogWrite(INV_PWM_CHANNELS[i], 255 - incoming_byte);
      }
    }

    // Digital
    for (i = 0; i < NUM_DIGITAL; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      if (incoming_byte <= 127) {
        digitalWrite(DIGITAL_CHANNELS[i], LOW);
      } else {
        digitalWrite(DIGITAL_CHANNELS[i], HIGH);
      }
    }

    // UART
    for (i = 0; i < NUM_UART; i++) {
      while (!Serial.available());
      incoming_byte = Serial.read();
      Serial1.write(incoming_byte);
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
    unsigned long time_without_msg = millis() - last_msg_time;
    if (time_without_msg >= TIME_OUT) {
      last_msg_time = millis ();

      if (TESTINGMODE) {
        for (i = 0; i < NUM_ARD_SERVOS; i++) {
          random_light(ARD_SERVO_CHANNELS[i]);
        }
        for (i = 0; i < NUM_PCA_SERVOS; i++) {
          random_light_pca(PCA_SERVO_CHANNELS[i]);
        }
      }

      for (i = 0; i < NUM_PWM; i++) {
        random_light(PWM_CHANNELS[i]);
      }
      for (i = 0; i < NUM_INV_PWM; i++) {
        random_light(INV_PWM_CHANNELS[i]);
      }
      for (i = 0; i < NUM_DIGITAL; i++) {
        random_light(DIGITAL_CHANNELS[i]);
      }
      random_strip();
    }
  }
  
  // PA skull
  unsigned long now = millis();
  if (((now - mic_read_timer) > PA_MIC_READ_DELAY) &&
      ((now - servo_move_timer) > PA_SERVO_MOVE_DELAY)) {
    mic_val = digitalRead(PA_MIC);
    if (mic_val) {
      pa_skull_servo.write(PA_SKULL_MAX);
      servo_move_timer = millis();
    } else {
      pa_skull_servo.write(PA_SKULL_MIN);
    }
  }
}

// Test the setup briefly
void test_sequence() {
  if (TESTINGMODE) {
    for (i = 0; i < NUM_ARD_SERVOS; i++) {
      test_channel(ARD_SERVO_CHANNELS[i]);
    }
    for (i = 0; i < NUM_PCA_SERVOS; i++) {
      test_channel_pca(PCA_SERVO_CHANNELS[i]);
    }
  }

  for (i = 0; i < NUM_PWM; i++) {
    test_channel(PWM_CHANNELS[i]);
  }
  for (i = 0; i < NUM_INV_PWM; i++) {
    test_channel(INV_PWM_CHANNELS[i]);
  }
  for (i = 0; i < NUM_DIGITAL; i++) {
    test_channel(DIGITAL_CHANNELS[i]);
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
