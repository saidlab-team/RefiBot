//=============================================================================
//    Copyright (C) 2022-2023 Wageningen University - All Rights Reserved
//            Author: Gonzalo Mier & Jurrian Doornbos
//=============================================================================

#ifndef AWESOME_LIB_H
#define AWESOME_LIB_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>


#define SERVOMIN  88  // This is the 'minimum' pulse length count (out of 4096) (org=150)
#define SERVOMAX  320 // This is the 'maximum' pulse length count (out of 4096) (org=600)
#define USMIN  600    // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 (org=600)
#define USMAX  2400   // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600(org=2400)
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates (org=50)

#define DEFAULT_WHEEL_STABLE 340 // Value of stable point if none found
#define WHEEL_FRONT 120
#define WHEEL_BACK -120
int left_wheel_front_dir {1}; // Dir of the wheel when going to front. Change to -1 if opposite dir
int right_wheel_front_dir {1}; // Dir of the wheel when going to front. Change to -1 if opposite dir
int left_wheel_stable_point {DEFAULT_WHEEL_STABLE};
int right_wheel_stable_point {DEFAULT_WHEEL_STABLE};
volatile int left_wheel_count = 0; // Count for the odometry
volatile int right_wheel_count = 0; // Count for the odometry
volatile int left_wheel_dir = 0; // Direction for the odometry
volatile int right_wheel_dir = 0; // Direction for the odometry


#define BLACK_LINE_THRESHOLD 700 // Value of black line treshold
#define NUM_FOLLOW_LINE             4  // number of sensors used
#define NUM_SAMPLES_PER_FOLLOW_LINE  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             4  // emitter is controlled by digital pin 4

#define PROX_SENSOR_LEFT 3    // Pin for the odometry sensor in the left wheel
#define PROX_SENSOR_RIGHT 2    // Pin for the odometry sensor in the right wheel

#define echoPin 6     // attach pin D6 Arduino to pin Echo of HC-SR04
#define trigPin 5     //attach pin D5 Arduino to pin Trig of HC-SR04

const int buzzer = 7; //buzzer to arduino pin 7




bool call_calibrate_follow_line_cameras {false};
bool call_calibrate_motors = {false};

bool proximity_sensor_left_connected {false};
bool proximity_sensor_right_connected {false};


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// defines variables
//long duration; // variable for the duration of sound wave travel
//int distance; // variable for the distance measurement
uint8_t L_servo = 0; // our servo # counter
uint8_t R_servo = 1; // our servo # counter

QTRSensors qtra;
unsigned int sensorValues[NUM_FOLLOW_LINE];

void count_prox_sensor_left(void) {
  left_wheel_count += left_wheel_dir;
}

void count_prox_sensor_right(void) {
  right_wheel_count += right_wheel_dir;
}

void setMotorsStablePoints(int left_wheel, int right_wheel) {
  left_wheel_stable_point = left_wheel;
  right_wheel_stable_point = right_wheel;
}



// time in seconds
void wait(float time) {
  delay(time * 1000);
}

void wait_for_serial_input() {
  while(Serial.available() == 0) {}
}



// time in seconds
void buzzer_sound(float time) {
  tone(buzzer, 100); // Send 100Hz sound signal...
  wait(time);        // ...for some time
  noTone(buzzer);     // Stop sound...
}

void buzzer_number(int no_beeps) {
  for (int i = 0; i < no_beeps; i++) {
    buzzer_sound(0.1);
    wait(0.1);
  }
}




float* read_rgb_sensor() {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED
  static float color[3];
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  return color;
}


float get_sonar_distance() {
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  return duration * 0.034 / 2;       // Speed of sound wave divided by 2 (go and back)
}

void stop_robot(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point);
}

void go_front(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir * WHEEL_FRONT);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir * WHEEL_FRONT);
  left_wheel_dir = 1;
  right_wheel_dir = 1;
}

void go_back(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir * WHEEL_BACK);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir * WHEEL_BACK);
  left_wheel_dir = -1;
  right_wheel_dir = -1;
}

void go_right(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir * WHEEL_FRONT);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir * WHEEL_BACK);
  left_wheel_dir = 1;
  right_wheel_dir = -1;
}

void go_left(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir * WHEEL_BACK);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir * WHEEL_FRONT);
  left_wheel_dir = -1;
  right_wheel_dir = 1;
}


uint16_t read_line_black_position(void) {
  return qtra.readLineBlack(sensorValues);
}

//some function to give array of true false values and black line position
bool* black_line_array(void) {
  static bool line_array[NUM_FOLLOW_LINE];

  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    line_array[i] = sensorValues[i] > BLACK_LINE_THRESHOLD;
  }
  return line_array;
}

void print_black_line_array_to_string(bool* arr) {
  for (int i = 0; i < NUM_FOLLOW_LINE; ++i) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
}

// Only for follo line follower
int array_count(bool* arr) {
  int sum = 0;
  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    sum = sum + arr[i];
  }
  return sum;
}


void follow_line(int error) {
  int Kp = 3;

  int adjust = Kp * error;

  float L_speed =  left_wheel_front_dir * (WHEEL_FRONT - adjust);
  float R_speed =  right_wheel_front_dir * (WHEEL_FRONT - adjust);

  pwm.setPWM(L_servo, 0, left_wheel_stable_point + L_speed);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + R_speed);
  delay(50);
}


int read_prox_sensor_left(void) {
  return digitalRead(PROX_SENSOR_LEFT);
}

int read_prox_sensor_right(void) {
  return digitalRead(PROX_SENSOR_RIGHT);
}

void overwrite_ir_calibration(void) {
  // robot calibration is quite irreliable, so we overwrite it with some default values
  // only works after calibration is executed
  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    qtra.calibrationOn.minimum[i] = 600;
    qtra.calibrationOn.maximum[i] = 900;
  }
}

void overwrite_ir_calibration(int* min, int* max) {
  // robot calibration is quite irreliable, so we overwrite it with some default values
  // only works after calibration is executed
  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    qtra.calibrationOn.minimum[i] = min[i];
    qtra.calibrationOn.maximum[i] = max[i];
  }
}


void calibrate_motors(void) {
  delay(500);
  left_wheel_stable_point = left_wheel_stable_point / 1.5;
  right_wheel_stable_point = right_wheel_stable_point / 1.5;
  left_wheel_dir = 1;
  right_wheel_dir = 1;
  bool stop_calibrating_left {false};
  bool stop_calibrating_right {false};
  const int number_zeros_read_stop_calib {10};
  int zeros_left {0};
  int zeros_right {0};

  for (int i = 0; i < 10000; i++) {
    stop_robot();


    if ( left_wheel_count > 0) {
      proximity_sensor_left_connected = true;
      zeros_left = 0;
    } else if ( left_wheel_count == 0) {
      zeros_left++;
      if (number_zeros_read_stop_calib < zeros_left) {
        stop_calibrating_left = true;
      }
    }

    if ( right_wheel_count > 0) {
      proximity_sensor_right_connected = true;
      zeros_right = 0;
    } else if ( right_wheel_count == 0) {
      zeros_right++;
      if (number_zeros_read_stop_calib < zeros_right) {
        stop_calibrating_right = true;
      }
    }

    if (!stop_calibrating_left) {
      left_wheel_stable_point += abs(left_wheel_count);
      if (left_wheel_stable_point > DEFAULT_WHEEL_STABLE * 1.50) {
        stop_calibrating_left = true;
        left_wheel_stable_point = DEFAULT_WHEEL_STABLE;
        Serial.println("Stable left motor point not found");
      }
    }
    left_wheel_count = 0;

    if (!stop_calibrating_right) {
      right_wheel_stable_point += abs(right_wheel_count);
      if (right_wheel_stable_point > DEFAULT_WHEEL_STABLE * 1.50) {
        stop_calibrating_right = true;
        right_wheel_stable_point = DEFAULT_WHEEL_STABLE;
        Serial.println("Stable right motor point not found");
      }
    }
    right_wheel_count = 0;

    if (stop_calibrating_right && stop_calibrating_left) {
      break;
    }

    delay(100);
  }
  if ( proximity_sensor_left_connected) {
    Serial.println("Found left proximity sensor");
    Serial.print("The stable point for the left wheel is: ");
    Serial.println( left_wheel_stable_point);
  } else {
    Serial.println("Left proximity sensor is not connected!!!");
    left_wheel_stable_point = DEFAULT_WHEEL_STABLE;
  }

  if ( proximity_sensor_right_connected) {
    Serial.println("Found right proximity sensor");
    Serial.print("The stable point for the right wheel is: ");
    Serial.println(right_wheel_stable_point);
  } else {
    Serial.println("Right proximity sensor is not connected!!!");
    right_wheel_stable_point = DEFAULT_WHEEL_STABLE;
  }

  left_wheel_dir = 0;
  right_wheel_dir = 0;
}

void calibrate_follow_line_cameras(bool true_calibration = true) {
  if (true_calibration) {
    Serial.println("Starting follow line calibration. Get ready");
    delay(500);
  }
  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  if (true_calibration) {
    for (int i = 0; i < 400; i++) {  // make the calibration take about 10 seconds
      qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
    // print the calibration minimum values measured when emitters were on
    for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
      Serial.print(qtra.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
      Serial.print(qtra.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(500);
  }
  else {
    qtra.calibrate();
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(500);
}




void default_config_setup() {
  Serial.begin(9600);
  Serial.println("Starting Serial Communication.");

  // Proximity sensor
  pinMode(PROX_SENSOR_LEFT, INPUT);
  pinMode(PROX_SENSOR_RIGHT, INPUT);

  delay(500);
  Serial.println("Starting Odometry");
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_LEFT), count_prox_sensor_left, RISING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_RIGHT), count_prox_sensor_right, RISING);



  // Config buzzer
  pinMode(buzzer, OUTPUT);

  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No color sensor found ... check your connections");
  }

  //Start PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  Serial.println("Starting PWM PCA9685.");

  // Initialize sonar pins
  Serial.println("Initialising HC-SR04 trig and echo pins.");
  pinMode(trigPin, OUTPUT);    // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);     // Sets the echoPin as an INPUT

  // Initialize follow line
  Serial.println("Initialising Follow line pins.");
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {
    0, 1, 2, 3
  }, NUM_FOLLOW_LINE);
  qtra.setEmitterPin(EMITTER_PIN);

  calibrate_follow_line_cameras(call_calibrate_follow_line_cameras);

  if (call_calibrate_motors) {
    calibrate_motors();
  }


}


#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

int melodyb[] = {

  // Never Gonna Give You Up - Rick Astley
  // Score available at https://musescore.com/chlorondria_5/never-gonna-give-you-up_alto-sax
  // Arranged by Chlorondria

  NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //40
  NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
  NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
  NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,
  NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,

  NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //45

  NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
  NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,

  NOTE_E5, 4, NOTE_D5, 2, REST, 4

};

int melodya[] = {

  // Take on me, by A-ha
  // Score available at https://musescore.com/user/27103612/scores/4834399
  // Arranged by Edward Truong

  NOTE_FS5,8, NOTE_FS5,8,NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8,
  REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
  NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8,
  REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,
  NOTE_FS5,8, NOTE_FS5,8,NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8,

  REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
  NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8,
  REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,
  NOTE_FS5,8, NOTE_FS5,8,NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8,
  REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,

  NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8,
  REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,

};


int melodyc[] = {

  // Game of Thrones
  // Score available at https://musescore.com/user/8407786/scores/2156716

  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, //1
  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16,
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,-4, NOTE_C4,-4,//5

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, //6
  NOTE_D4,-1, //7 and 8
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, //11 and 12

  //repeats from 5
  NOTE_G4,-4, NOTE_C4,-4,//5

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, //6
  NOTE_D4,-1, //7 and 8
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, //11 and 12
  NOTE_G4,-4, NOTE_C4,-4,
  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4,  NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16,

  NOTE_D4,-2,//15
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_D4,-8, NOTE_DS4,-8, NOTE_D4,-8, NOTE_AS3,-8,
  NOTE_C4,-1,
  NOTE_C5,-2,
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2,
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_F4,-4,
  NOTE_G4,-1,

  NOTE_C5,-2,//28
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2,
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_D4,-4,
};

void memes(int a=0) {

  int melody[256];

  if (a==0){

    byte i;
    for (i=0; i<(sizeof(melodya)/sizeof(int)); i++){
      if (i<=sizeof(melodya)){
        melody[i] = melodya[i];
      }
      else {
        melody[i] = NULL;
      }
    }
  }

  else if (a==1){

    byte i;
    for (i=0; i<(sizeof(melodyb)/sizeof(int)); i++){
      if (i<=sizeof(melodyb)){
        melody[i] = melodyb[i];
      }
      else{
        melody[i] = NULL;
      }
    }
  }

  else {

    byte i;
    for (i=0; i<(sizeof(melodyc)/sizeof(int)); i++){
      if (i<=sizeof(melodyc)){
        melody[i] = melodyc[i];
      }
      else {
        melody[i] = NULL;
      }
    }
  }


  // change this to make the song slower or faster
  int tempo = 114;


  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!

  // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
  // there are two values per note (pitch and duration), so for each note there are four bytes
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;

  // this calculates the duration of a whole note in ms
  int wholenote = (60000 * 4) / tempo;

  int divider = 0, noteDuration = 0;



  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
  }

}


#endif  // AWESOME_LIB_H
