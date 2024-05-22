#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// LED and button tracking variables
const int col[8]              = {6, A3, 13, 11, A0, 10, 5, 4};
const int row[8]              = {9, 7, 2, 8, 12, 3, A2, A1};
int button                    = A7;
uint8_t bitmap[8]             = {0,0,0,0,0,0,0,0};

/*
------------------------------------
BITMAP DEFINITIONS
------------------------------------
*/

/* Yes, this is long and stupid. The alternative is putting each
of these into hex (not doing that because I want it to be clear
what each of these bitmaps is actually drawing) or turning each
of these into their own functions, which takes even more lines.
Normally I'd put this into another file to not clog my main one,
but I want to make this a single code file for ease of use. */
// Byte array with major graphic screens
uint8_t graphicOptions[13][8] = {{0b01111110, // 0 Target hit
                                  0b11100111,
                                  0b11000011,
                                  0b11000011,
                                  0b11000011,
                                  0b11000011,
                                  0b11100111,
                                  0b01111110},
                                 {0b11111111, // 1 Upper right
                                  0b00000011,
                                  0b00000101,
                                  0b00001001,
                                  0b00010001,
                                  0b00100001,
                                  0b01000001,
                                  0b10000001},
                                 {0b11111111, // 2 Upper left
                                  0b11000000,
                                  0b10100000,
                                  0b10010000,
                                  0b10001000,
                                  0b10000100,
                                  0b10000010,
                                  0b10000001},
                                 {0b10000001, // 3 Lower right
                                  0b01000001,
                                  0b00100001,
                                  0b00010001,
                                  0b00001001,
                                  0b00000101,
                                  0b00000011,
                                  0b11111111},
                                 {0b10000001, // 4 Lower left
                                  0b10000010,
                                  0b10000100,
                                  0b10001000,
                                  0b10010000,
                                  0b10100000,
                                  0b11000000,
                                  0b11111111},
                                 {0b00011000, // 5 Up
                                  0b00111100,
                                  0b01011010,
                                  0b10011001,
                                  0b00011000,
                                  0b00011000,
                                  0b00011000,
                                  0b00011000},
                                 {0b00011000, // 6 Down
                                  0b00011000,
                                  0b00011000,
                                  0b00011000,
                                  0b10011001,
                                  0b01011010,
                                  0b00111100,
                                  0b00011000},
                                 {0b00001000, // 7 Right
                                  0b00000100,
                                  0b00000010,
                                  0b11111111,
                                  0b11111111,
                                  0b00000010,
                                  0b00000100,
                                  0b00001000},
                                 {0b00010000, // 8 Left
                                  0b00100000,
                                  0b01000000,
                                  0b11111111,
                                  0b11111111,
                                  0b01000000,
                                  0b00100000,
                                  0b00010000},
                                 {0b00011000, // 9 Target hover
                                  0b00011000,
                                  0b00011000,
                                  0b11111111,
                                  0b11111111,
                                  0b00011000,
                                  0b00011000,
                                  0b00011000},
                                 {0b00000000, // 10 Blank
                                  0b00000000,
                                  0b00000000,
                                  0b00000000,
                                  0b00000000,
                                  0b00000000,
                                  0b00000000,
                                  0b00000000},
                                 {0b10111101, // 10 Target
                                  0b01000010,
                                  0b10100101,
                                  0b10011001,
                                  0b10011001,
                                  0b10100101,
                                  0b01000010,
                                  0b10111101},
                                 {0b00000000, // 12 Smiley face
                                  0b00100100,
                                  0b00100100,
                                  0b00000000,
                                  0b01000010,
                                  0b01100110,
                                  0b00111100,
                                  0b00000000}};

// Using hex here because number bitmaps aren't rocket science
uint8_t numbers[10][8]        = { {0x6,0x9,0x9,0x9,0x9,0x9,0x9,0x6},  // 0
                                  {0x2,0x6,0x2,0x2,0x2,0x2,0x2,0x2},  // 1
                                  {0x6,0x9,0x9,0x1,0x2,0x4,0x8,0xF},  // 2
                                  {0x6,0x9,0x1,0x6,0x1,0x1,0x9,0x6},  // 3
                                  {0x9,0x9,0x9,0xF,0x1,0x1,0x1,0x1},  // 4
                                  {0xF,0x8,0x8,0xE,0x1,0x1,0x9,0x6},  // 5
                                  {0x6,0x9,0x8,0xE,0x9,0x9,0x9,0x6},  // 6
                                  {0xF,0x9,0x1,0x1,0x1,0x1,0x1,0x1},  // 7
                                  {0x6,0x9,0x9,0x6,0x9,0x9,0x9,0x6},  // 8
                                  {0x6,0x9,0x9,0x7,0x1,0x1,0x1,0x1}}; // 9

// Time-keeping variables
unsigned long gameStartTime;            // Time since we started current round of the game
unsigned long lastUpdateTime;           // Time since last major input update
unsigned long targetHitTime;            // Time we last hit a target - used for tracking hit graphic uptime
unsigned long gameDuration    = 60000;  // One-minute game duration
int ledInterval               = 1000;   // One-second initial blink interval

// Scoring and target tracking variables
bool targetHit;                         // Tracks if the target hit graphic is up
bool scoreDisplayed           = false;  // Tracks if the score display is currently up
int score                     = 0;      // Self-explanatory
int gameStatus                = 0;      // 0 getting ready to start, 1 playing game, 2 showing score

//max and min values that I'd WANT to use: 160, -160
#define YAW_MAX 149
#define YAW_MIN -150
float ghostYaw;
//max and min: 0, -65
#define PITCH_MAX 0
#define PITCH_MIN -65
float ghostPitch;

/*
------------------------------------
GYRO SETUP
------------------------------------
*/

MPU6050 mpu; // gyro

#define INTERRUPT_PIN 2 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // says whether MPU interrupt pin has gone high

// This is only a function so it can be passed into another function
void dmpDataReady() {
  mpuInterrupt = true;
}

// Handle all of the gyro setup
void setupGyro(){
  // join I2C
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-36);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(2925);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // make sure main loop() function knows it's okay to use this
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // 1: initial memory load failed, 2: DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup() {
  setupGyro();

  /*
  ----------------------------------
  LED board setup
  ----------------------------------
  */
  Serial.begin(9600);
  for (int i = 0; i < 8; i++){
    pinMode(col[i], OUTPUT);
    pinMode(row[i], OUTPUT);

    digitalWrite(col[i], LOW);
  }

  // Do time initializations
  lastUpdateTime  = 0;
  targetHitTime   = -2000;

  // Spawn in our ghost target
  ghostYaw = (float) random(YAW_MIN, YAW_MAX);
  ghostPitch = (float) random(PITCH_MIN, PITCH_MAX);

  memcpy(bitmap, graphicOptions[11], sizeof bitmap); // Draw target
}

// Handles screen refresh off of the current bitmap values
void refreshScreen(){
  for (int currRow = 0; currRow < 8; currRow++){
    for (int currCol = 0; currCol < 8; currCol++){
      // bitshift to get the correct column's bit
      uint8_t bit = bitmap[currRow] & (1 << (7-currCol));
      digitalWrite(col[currCol], bit);
    }

    // Blink the current row
    digitalWrite(row[currRow], LOW);
    delay(1);
    digitalWrite(row[currRow], HIGH);
  }
}

// Draw the current score to the bitmap
void drawScore(){
  memcpy(bitmap, graphicOptions[10], sizeof bitmap);  // Blank out the screen

  if (score > 9){
    for (int row = 0; row < 8; row++){
      int tens = floor(score / 10);
      bitmap[row] |= (numbers[tens][row] << 4);
    }
    for (int row = 0; row < 8; row++){
      bitmap[row] |= (numbers[score % 10][row]);
    }
  }
  // draw it in the middle if we've only got one digit
  else{
    for (int row = 0; row < 8; row++){
      bitmap[row] |= (numbers[score][row] << 2);
    }
  }
  scoreDisplayed = true;
}

// Check if the current gyro coords pull is from within the thresholds of a hit
bool isInRange(float currYaw, float currPitch){
  int range = 8;
  int yawDistance;    // 0: on target, -1/1: within range in either direction, -2/2: out of range
  int pitchDistance;  // 0: on target, -1/1: within range in either direction, -2/2: out of range

  // Set our tracker distance variables for yaw and pitch
  float yawDiff = ghostYaw - currYaw;
  if (yawDiff <= range && yawDiff >= 0)               yawDistance = -1;
  else if (yawDiff >= (0-range) && yawDiff <= 0)      yawDistance = 1;
  else if (yawDiff < range)                           yawDistance = -2;
  else                                                yawDistance = 2;

  float pitchDiff = ghostPitch - currPitch;
  if (pitchDiff <= range && pitchDiff >= 0)           pitchDistance = -1;
  else if (pitchDiff >= (0-range) && pitchDiff <= 0)  pitchDistance = 1;
  else if (pitchDiff < range)                         pitchDistance = -2;
  else                                                pitchDistance = 2;

  // If we've looked at the target, draw the target register graphic
  if ((yawDistance == -1 || yawDistance == 1) && (pitchDistance == -1 || pitchDistance == 1)){
    memcpy(bitmap, graphicOptions[9], sizeof bitmap);
    return true;
  }
  else {  // Otherwise update the arrows to what they need to be
    int bitmapChoice = 0;
    if (yawDistance == -2 && pitchDistance == -2)     bitmapChoice = 2; // We're lower right, turn upper left
    else if (yawDistance == -2 && pitchDistance == 2) bitmapChoice = 4; // We're upper right, turn lower left
    else if (yawDistance == 2 && pitchDistance == -2) bitmapChoice = 1; // We're lower left, turn upper right
    else if (yawDistance == 2 && pitchDistance == 2)  bitmapChoice = 3; // We're upper left, turn lower right
    else if (yawDistance == -2)   bitmapChoice = 8; // We're directly right, turn left
    else if (yawDistance == 2)    bitmapChoice = 7; // We're directly left, turn right
    else if (pitchDistance == -2) bitmapChoice = 5; // We're directly below, look up
    else if (pitchDistance == 2)  bitmapChoice = 6; // We're directly above, look down

    memcpy(bitmap, graphicOptions[bitmapChoice], sizeof bitmap);
    return false;
  }
}

// Handle trigger pulls
void buttonHandler(bool onTarget){
  if (onTarget){
    memcpy(bitmap, graphicOptions[0], sizeof bitmap);
    targetHit = true;
    targetHitTime = millis();
    score++;
  }
  lastUpdateTime = millis();
}

// Handle one loop's worth of base game logic
void updateGame(){
  // If we hit a target and it's been a second since graphic was shown, generate new ghost
  if (targetHit && (millis() - targetHitTime > 500)){
    targetHit = false;
    ghostYaw = (float) random(YAW_MIN, YAW_MAX);
    ghostPitch = (float) random(PITCH_MIN, PITCH_MAX);
  }

  // read a packet from FIFO
  if (!targetHit && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // If we're not still showing the graphic for a hit target, check if we're looking at target
    bool onTarget = isInRange(ypr[0] * 180/M_PI, ypr[1] * 180/M_PI);

    // If the user's pressing the button, check for hit
    if (analogRead(button) > 500 && (millis() - lastUpdateTime >= 50)){
      buttonHandler(onTarget);
    }
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // Handle main game loop
  switch (gameStatus){
    case 0: // Game hasn't started, watch for start button press
      if (analogRead(button) > 500 && millis() - lastUpdateTime > 2000){
        gameStartTime = millis();
        gameStatus = 1;
      }
      break;
    case 1: // Game is active, check for time complete or else handle game logic
      if (millis() - gameStartTime >= gameDuration){
        gameStatus = 2;
      }
      else{
        updateGame();
      }
      break;
    case 2: // Game over, show score and watch for reset button after short delay
      if (!scoreDisplayed){
        drawScore();
        lastUpdateTime = millis();
        delay(10);
      }
      else if (scoreDisplayed && analogRead(button) > 500 && millis() - lastUpdateTime > 2000){
        memcpy(bitmap, graphicOptions[11], sizeof bitmap); // Draw target
        gameStatus = 0;
        lastUpdateTime = millis();
        scoreDisplayed = false;
      }
      break;
  }
  
  refreshScreen();
}