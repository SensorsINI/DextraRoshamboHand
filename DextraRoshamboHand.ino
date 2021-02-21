// This is firmware for the Dextra robot hand roshambo demos from Sensors group NPP project
// copyright T. Delbruck, 2016-2019
// change notes
// May 2019: Add extra servo output for hand with opposed thumb

#define VERSION  "Version dated 7.10,2020"
#define BAUDRATE 115200 // serial port baud rate, host must set same speed
#undef USELEDSTRIP // define to send commands to LED strip from Adafruit
#undef ECHO // define to echo back cmd chars (for measuring roundtrip latency)
#define OPPOSEDTHUMB // to use with hand that has opposed thumb (new type from aliexpress without spring compliant actuation)

const int MAX_SERVO_ACTIVATION_TIME_MS = 400; // limits time servos are powered to any one position, set zero to disable timeout
const long int NO_COMMAND_TURN_OFF_TIME_MS = 6000; // time to relax (turn off servos) if no command has been recieved for a long time
const long int BUT_TAP_TIME_MS = 300; // if button is only tapped for less than this time, then show the whole RSB sequence, otherwise, if button is held down, go to rock while held and then paper when released.

#ifdef USELEDSTRIP
#include <Adafruit_WS2801.h> // for LED strip, if error compiling, install Adafruit library WS2801
#endif
#include "FastServo.h" // for fast servo pulse rate (300Hz)
//change COM ports accordingly

// roshambo robot hand control
// in linux echo “x” >> /dev/ttyUBS0
// where x="1" for paper, "2" for scissors, "3" for rock, "4" for slowly wiggling, "5" to relax (turn off servos), "6" to twitch, i.e. during game throws
// serial port characters and states, e.g. for ROCK send '3'
const int PAPER = 1, SCISSORS = 2, ROCK = 3, SLOWLY_WIGGLE = 4, TURN_OFF = 5, TWITCH = 6;
int state = SLOWLY_WIGGLE; // last command received
long lastCmdTimeMs;

FastServo servo0, servo1;
#ifdef OPPOSEDTHUMB
FastServo servoThumb;
#endif

// servo0 = pin 9 = scissors servo
// servo1 = pin 10 = papers servo
// servo0 scissors
// servo1 + servo0 paper
// servo0+1 twitch rock

const int SERVO_0_PIN = 9, SERVO_1_PIN = 10, BUT_PIN = 2, LED_PIN = 7;
#ifdef OPPOSEDTHUMB
const int SERVO_THUMB_PIN = 11;
#endif


// LED strip light from http://www.play-zone.ch/en/12mm-diffused-digital-rgb-led-pixels-50er-strang-ws2801.html
const int LED_STRIP_DATA_PIN = 4, LED_STRIP_CLOCK_PIN = 3, NUM_LEDS = 50;

// https://learn.adafruit.com/12mm-led-pixels/code Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
// first third of LEDS are rock, 2nd 3rd are paper, last third are scissors lamps
#ifdef USELEDSTRIP
Adafruit_WS2801 ledStrip;
#endif

// Servos
// rest and extended positions are set by reading pot depending on button press
// on robot hand from ebay, fingers are wired up so that thumb and last 2 fingers are on one servo output and first two fingers are on the other.
// because the servos have different polarity of rotation, see the rock() scissors() and paper() functions to see the exact action
const unsigned int rest0Pos = 1800, ext0Pos = 1000, rest1Pos = 1800, ext1Pos = 1000;// servo pos in us, 2000 finger curled, 1000 fully out
#ifdef OPPOSEDTHUMB
const unsigned int restThumb = 1800, extThumb = 1000;// servo pos in us, 2000 finger curled, 1000 fully out
#endif

// idle waving action. When in background mode, the servo slowly wave around neutral position
const unsigned int idle0dc = (ext0Pos + rest0Pos) / 2, idle1dc = (rest1Pos + ext1Pos) / 2;
const int idleAcAmplitude = 700; // amplitude of osciallations
const int idle0PeriodMs = 1000, idle1PeriodMs = idle0PeriodMs * 1.11111; // discomensurate periods
const int idleReturnTauMs = 3000; // time constant of IIR lowpass filter that filters sinusoidal generators for return to idle position
const int twitchDelayMs = 50, twitchAmplitude = 50;

// cutoff frequency is high enough to let through the AC but prevent sudden jerky returns to osciallation

float idle0LpPos, idle1LpPos;
long lastUpdateTimeMs;

// serial port
int cmd = 0;
bool lastBut = false;
bool attached = false;
int lastAdcCheckTimeMs = 0;
long int lastHeartbeatTime = 0;
long int heartbeatHalfPeriodMs = 500;

int lastS0, lastS1; // last settings for servo 0 and 1, to check for new settings and timeout of powering servo
#ifdef OPPOSEDTHUMB
int lastThumb;
#endif
long lastNewServoSettingTimeMs; // last time that a unique new set of servo positions was set
long lastButtonChangeTimeMs; // last time that button state changed

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUT_PIN, INPUT_PULLUP);
  pinMode(A1, INPUT);
  analogReference(EXTERNAL); // tied to 3V3 so 1023=full scale
  Serial.begin(BAUDRATE);
  attach();
  Serial.println("Dextra RoShamBo ready to serve");
  Serial.println(VERSION);
#ifdef USELEDSTRIP
  Serial.println("Commands for Adafruit LED strip are enabled");
#else
  Serial.println("Adafruit LED strip is disabled");
#endif
#ifdef ECHO
  Serial.println("Command characters will echo");
#else
  Serial.println("Command characters will not echo");
#endif

  Serial.print("Enter ");
  Serial.print(ROCK);
  Serial.print(" for rock, ");
  Serial.print(SCISSORS);
  Serial.print(" for scissors, ");
  Serial.print(PAPER);
  Serial.println(" for paper, ");
  Serial.print(SLOWLY_WIGGLE);
  Serial.println(" for slowly-wiggling state, ");
  Serial.print(TURN_OFF);
  Serial.println(" to turn off servos and LEDs,");
  Serial.print(TWITCH);
  Serial.println(" to twitch the servos");
  //  Serial.println("Turn pot and hit black button to set rest position");

#ifdef USELEDSTRIP
  ledStrip = Adafruit_WS2801(NUM_LEDS, LED_STRIP_DATA_PIN, LED_STRIP_CLOCK_PIN);
  ledStrip.begin(); // LED strip initialization
  ledStrip.show(); // Update LED contents, to start they are all 'off'
#endif
  toggleLed();
  roshamboSequence();

  idle0LpPos = idle0dc; // initialize oscillator
  idle1LpPos = idle1dc; // initialize oscillator
  lastUpdateTimeMs = millis();
  lastCmdTimeMs = lastUpdateTimeMs;
  lastButtonChangeTimeMs=lastUpdateTimeMs;

}


void loop() {
  boolean butPressed = !digitalRead(BUT_PIN); // button pressed state, TRUE if pressed now
  //int adc=analogRead(A7);
  if (butPressed && !lastBut) { // if button just pressed but wasn't last cycle
    toggleLed();
    lastButtonChangeTimeMs=millis();
    state=ROCK;
    idle0LpPos = idle0dc; // initialize oscillator
    idle1LpPos = idle1dc; // initialize oscillator
    lastUpdateTimeMs = millis();
    lastButtonChangeTimeMs=lastUpdateTimeMs;
    lastCmdTimeMs = lastUpdateTimeMs;
  } else if (!butPressed && lastBut && millis()-lastButtonChangeTimeMs<BUT_TAP_TIME_MS) { // button released but quickly
    lastButtonChangeTimeMs=millis();
    roshamboSequence();
  }else if (!butPressed && lastBut){
    toggleLed();
    lastButtonChangeTimeMs=millis();
    state=PAPER; // if released slowly, then now just freeze paper
  }
  lastBut = butPressed;

  if (Serial.available())  {
    toggleLed();
    digitalWrite(LED_PIN, 1);
    heartbeatHalfPeriodMs = 300;
    cmd = Serial.read() - '0'; // convert the character '1'-'9' to decimal 1-9
#ifdef ECHO
    Serial.write(cmd + '0');
#endif
    lastCmdTimeMs = millis();
    switch (cmd) {
      case PAPER:
        state = PAPER;
        break;
      case SCISSORS:
        state = SCISSORS;
        break;
      case ROCK:
        state = ROCK;
        break;
      case SLOWLY_WIGGLE:
        state = SLOWLY_WIGGLE;
        break;
      case TURN_OFF:
        state = TURN_OFF;
        break;
      case TWITCH:
        state = TWITCH;
        break;
      default:
        state = TURN_OFF;
        break;
    }
  }

  if (millis() - lastCmdTimeMs > NO_COMMAND_TURN_OFF_TIME_MS) {
    state = TURN_OFF;
  }

  switch (state) {
    case PAPER:
      paper();
      break;
    case SCISSORS:
      scissors();
      break;
    case ROCK:
      rock();
      break;
    case SLOWLY_WIGGLE:
      slowlyWiggle();
      break;
    case TURN_OFF:
      turnoff();
      break;
    case TWITCH:
      twitch();
      state = TURN_OFF;
      break;
    default:
      break;

  }
  if (millis() - lastHeartbeatTime > heartbeatHalfPeriodMs) {
    lastHeartbeatTime = millis();
    toggleLed();
  }
  digitalWrite(LED_PIN, 0);
}

void updateOscillators() {
  long t = millis();
  int dt = t - lastUpdateTimeMs;
  if (dt > 0) {
    //    Serial.print(" t=");
    //    Serial.print(t);
    //        Serial.print(" dt=");
    //        Serial.print(dt);
    idle0LpPos = updateOscillator(t, dt, idle0LpPos, idle0dc, idleAcAmplitude, idle0PeriodMs);
    idle1LpPos = updateOscillator(t, dt, idle1LpPos, idle1dc, idleAcAmplitude, idle1PeriodMs);
    lastUpdateTimeMs = t;
    //        Serial.print(" idle0LpPos=");
    //        Serial.print(idle0LpPos);
    //        Serial.print(" idle1LpPos=");
    //        Serial.println(idle1LpPos);
  }
}

float updateOscillator(long t, int dt, float lastPos, int dc, int ac, int per) {
  if (dt > 0) {
    float pos = dc + ac * sin((2 * PI * (t % per)) / per);
    //    Serial.print(" pos=");
    //    Serial.println(pos);
    float m = (float)dt / idleReturnTauMs;
    if (m > 1) m = 1;
    //    Serial.print(" m=");
    //    Serial.print(m, 6);
    float newPos = ((1 - m) * lastPos) + (m * pos);
    //    Serial.print(" lastPos=");
    //    Serial.print(lastPos);
    //    Serial.print(" pos=");
    //    Serial.print(pos);
    //    Serial.print(" newPos=");
    //    Serial.println(newPos);
    //
    //    Serial.println();
    return newPos;
  }
  return lastPos;
}

void attach() {
  servo0.attach(SERVO_0_PIN);  // attaches the servo on pin 9 to the servo object
  servo1.attach(SERVO_1_PIN);  // attaches the servo on pin 10 to the servo object
#ifdef OPPOSEDTHUMB
  servoThumb.attach(SERVO_THUMB_PIN);
#endif
  attached = true;
}

void detach() {
  servo0.detach();
  servo1.detach();
#ifdef OPPOSEDTHUMB
  servoThumb.detach();
#endif
  attached = false;
}

void turnoff() { // turn off servos
  state = TURN_OFF;
  if (attached) {
    detach();
#ifdef USELEDSTRIP
    clearLedStrip();
#endif
  }
  else {
#ifdef USELEDSTRIP
    clearLedStrip();
#endif
  }
  heartbeatHalfPeriodMs = 1500;
}

void checkAttached() {
  if (!attached) {
    attach();
  }
}

// wiggle servos slowly
void slowlyWiggle() {
  if (millis() - lastCmdTimeMs > NO_COMMAND_TURN_OFF_TIME_MS) {
    state = TURN_OFF;
  } else {
    state = SLOWLY_WIGGLE;
    heartbeatHalfPeriodMs = 750;


    //  servo0.writeMicroseconds(rest0Pos);
    //  servo1.writeMicroseconds(rest1Pos);
    updateOscillators();
    checkAttached();
    setServoValues((int)idle0LpPos, (int)idle1LpPos, false);
#ifdef OPPOSEDTHUMB
    setThumbServo((int)idle1LpPos);
#endif
#ifdef USELEDSTRIP
    clearLedStrip();
#endif
  }
}

void toggleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void comePlay() {
#ifdef USELEDSTRIP
  clearLedStrip();
#endif
  for (int loop_idx = 0; loop_idx < 1500; loop_idx++)
    if (Serial.available()) {
      break;
    } else {
      delay(1);
    }
#ifdef USELEDSTRIP
  for (int i = 0; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, 0xFFFFFF);
  }
  ledStrip.show();
#endif
  for (int loop_idx = 0; loop_idx < 1500; loop_idx++)
    if (Serial.available()) {
      break;
    } else {
      delay(1);
    }
}

#ifdef USELEDSTRIP
void clearLedStrip() {
  for (int i = 0; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, 0);
  }
  ledStrip.show();
}
#endif

void setServoValues(int s0, int s1, bool resetLowPass) {
  if (s0 != lastS0 || s1 != lastS1) {
    servo0.writeMicroseconds(s0);
    servo1.writeMicroseconds(s1);
    lastNewServoSettingTimeMs = millis();

    if (resetLowPass) {
      idle0LpPos = s0;
      idle1LpPos = s1;
    }
  } else if (millis() - lastNewServoSettingTimeMs > MAX_SERVO_ACTIVATION_TIME_MS) {
    // timeout, detach servos
    turnoff();
  }
}

#ifdef OPPOSEDTHUMB
void setThumbServo(int value){
 if (value != lastThumb) {
    servoThumb.writeMicroseconds(value);
    lastNewServoSettingTimeMs = millis();
  } else if (millis() - lastNewServoSettingTimeMs > MAX_SERVO_ACTIVATION_TIME_MS) {
    // timeout, detach servos
    turnoff();
  }
}
#endif

void rock() {
  checkAttached();
  setServoValues(ext0Pos, ext1Pos, true);
#ifdef OPPOSEDTHUMB
 setThumbServo(restThumb);
#endif
#ifdef USELEDSTRIP
  clearLedStrip();
  for (int i = 2 * NUM_LEDS / 3; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, 0x0000FF);
  }
  ledStrip.show();
#endif
}

void scissors() {
  checkAttached();
  setServoValues(ext0Pos, rest1Pos, true);
#ifdef OPPOSEDTHUMB
 setThumbServo(restThumb);
#endif
#ifdef USELEDSTRIP
  clearLedStrip();
  for (int i = NUM_LEDS / 3; i < 2 * NUM_LEDS / 3; i++) {
    ledStrip.setPixelColor(i, 0x00FF00);
  }
  ledStrip.show();
#endif
}

void paper() {
  checkAttached();
  setServoValues(rest0Pos, rest1Pos, true);
#ifdef OPPOSEDTHUMB
 setThumbServo(extThumb);
#endif
#ifdef USELEDSTRIP
  clearLedStrip();
  for (int i = 0; i < NUM_LEDS / 3; i++) {
    ledStrip.setPixelColor(i, 0xFF0000);
  }
  ledStrip.show();
#endif
}

void roshamboSequence() {
  int d = 1000;
  rock();
  delay(d);
  scissors();
  delay(d);
  paper();
  delay(d);
  d = 100;
  for (int i = 0; i < 10; i++) {
    rock();
    delay(d);
    scissors();
    delay(d);
    paper();
    delay(d);
  }
  paper();

}

void twitch() {
  checkAttached();
  setServoValues(idle0dc - twitchAmplitude, idle1dc - twitchAmplitude, true);
#ifdef USELEDSTRIP
  clearLedStrip();
  for (int i = 0; i < NUM_LEDS ; i++) {
    ledStrip.setPixelColor(i, 0xFFFFFF);
  }
  ledStrip.show();
#endif
  delay(twitchDelayMs);
  setServoValues(idle0dc + twitchAmplitude, idle1dc + twitchAmplitude, true);
#ifdef USELEDSTRIP
  clearLedStrip();
  for (int i = 0; i < NUM_LEDS ; i++) {
    ledStrip.setPixelColor(i, 0);
  }
  ledStrip.show();
#endif
  delay(twitchDelayMs);

}
