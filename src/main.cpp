/****
 * PTMSCPrototype V0.77, February 2022
 * 
 * Drive the pinto abalone exhibition prototype simulating a scuba diver doing
 * abalone outplanting using a "flying camera" mechanism simiar in principle 
 * to the cameras on four cables the NFL uses to "fly" over the field 
 * (technically called a "cable-driven parallel robot"). The prototype is 
 * intended to work out the kinks in the mechanism and its control before 
 * building the real thing. Here the software is controlled using a 
 * linux-like command interface delivered over the Arduino Serial stream or 
 * via a joystick-and-buttons console specially made for this.
 * 
 * The command interface is described in using the help command.
 * 
 * Once the mechnism is calibrated, moving the diver around is pretty 
 * straightforward. Push the joystick forward to "swim"" forward, forward and 
 * left to swim forward but curving to the left, forward and right to swim 
 * forward but curving to the right. Release the joystick to stop. Moving the 
 * joystick back doesn't do anything because the diver can't swim backwards. 
 * If the Down button is pressed while the diver is swimming, she'll descend.
 * Pressing the Up button causes her to rise. Pressing the joystick left or 
 * right (but not forward) causes her to turn in place. Pressing Up or Down 
 * with no motion doesn't do anything. (But maybe it should just make her rise 
 * or fall vertically.)
 * 
 * Before the mechanism is calibrated, the joystick can be used to initiate 
 * calibration by holding the Place button down and pressing the joystick 
 * forward. Releasing the joystick stops the process.
 * 
 * The "hard part" of driving the mechanism is in the FlyingPlatform library.
 * 
 * In the exhibit, our Serial is connected to the media player. For debugging 
 * purposes it echos what we send it. It also provides a means to relay 
 * commands typed on the keyboard to us, and can issue commands to us, if need 
 * be. Importantly, we can send it commands to cause it to play different 
 * video clips appropriate to the state of the visitor's interaction. this is 
 * done by sending "!play <clip_name>\n" to Serial. Here <clip_name> is 
 * the name of the clip to be played, as defined in the media player's 
 * mediadef.h file.
 * 
 * Copyright (C) 2020-2022 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/

#include <Arduino.h>
#include "FlyingPlatform.h"
#include "UserInput.h"
#include "Console.h"
#include "Storyboard.h"

#define DEBUG                       // Uncomment to turn on debugging messages

// For the stepper motors
#define PIN_0D        (4)           // Motor driver 0 (x = 0, y = 0 winch) direction pin
#define PIN_0P        (5)           //   pulse pin 0
#define PIN_1D        (6)           // Motor driver 1 (x = SIZE_X, y = 0 winch) direction pin
#define PIN_1P        (7)           //   pulse pin 1
#define PIN_2D        (9)           // Motor driver 2 (x = 0, y = SIZE_Y winch) direction pin
#define PIN_2P        (10)          //   pulse pin 2
#define PIN_3D        (11)          // Motor driver 3 (x = SIZE-X, y = SIZE_Y winch) direction pin
#define PIN_3P        (12)          //   pulse pin 3
#define PIN_EN        (13)          // Enable pin - common to all motor drivers (active LOW)
#define MAX_SPEED     (800)         // In steps per second

#define CAL_CLK_PIN   (2)           // The pin from which the calReader clock signal comes
#define CAL_DATA_PIN  (3)           // The pin from which the calReader data signal comes
#define DVR_CLK_PIN   (39)          // Pin to which the diver's clock signal goes
#define DVR_DATA_PIN  (38)          // Pin to which the diver's data goes

// Physical size of flying space (mm) measured from floor and between hoist points
#define SIZE_X        (945)
#define SIZE_Y        (550)
#define SIZE_Z        (777)

// Safety margins (mm) for left, right, front, back, down and up
#define MARGIN_L      (30)
#define MARGIN_R      (30)
#define MARGIN_F      (25)
#define MARGIN_B      (25)
#define MARGIN_D      (310)           // Big 'cause the diver hangs down from the suspension point
#define MARGIN_U      (80)            // The cables get pretty tight around this height

// Home location (mm)
#define HOME_X        (445)
#define HOME_Y        (290)
#define HOME_Z        (565)

// Rest location relative to home (mm)
#define REST_DELTA_X  (0)
#define REST_DELTA_Y  (0)
#define REST_DELTA_Z  (-150)

// Conversion between mm and steps. (The FlyingPlatform deals exclusively in steps.)
#define STEPS_PER_REV (800.0)       // What it sounds like
#define MM_PER_REV    (32.0)        // Millimeters of wire travel per revolution
#define STEPS_PER_MM  (STEPS_PER_REV / MM_PER_REV)
#define mmToSteps(mm)       ((long)((mm) * STEPS_PER_MM))
#define stepsToMm(steps)    ((long)((steps) / STEPS_PER_MM))

// Pins to which the console is attached
#define SW_LEFT         (22)
#define SW_FORWARD      (24)
#define SW_RIGHT        (26)
#define SW_BACK         (28)
#define SW_DOWN         (30)
#define SW_UP           (32)
#define SW_PLACE        (34)
#define LED_PLACE       (36)

// Storyboard related definitions
#define TIMEOUT_MS      (30000)       // millis() of no activity before exhibit auto-reset

// Misc compile-time definitions
#define BANNER          F("PTMSC Prototype v 0.77 March 2022")

// Global variables
FlyingPlatform diver {
  PIN_0D, PIN_0P, PIN_1D, PIN_1P, 
  PIN_2D, PIN_2P, PIN_3D, PIN_3P, 
  PIN_EN, CAL_CLK_PIN, CAL_DATA_PIN,
  DVR_CLK_PIN, DVR_DATA_PIN,
  mmToSteps(SIZE_X), mmToSteps(SIZE_Y), mmToSteps(SIZE_Z)};
UserInput ui {Serial};
Console console {SW_LEFT, SW_FORWARD, SW_RIGHT, SW_BACK, SW_DOWN, SW_UP, SW_PLACE, LED_PLACE};
Storyboard* sb = Storyboard::getInstance();
unsigned long lastTouchMillis;                // millis at when the last console evrnt happened
bool backing = false;   // Temp: Need a way to stop media player: joystick back, then press Down

/**
 * 
 * Interpret FlyingPlatform return codes to Serial
 * 
 **/
void interpretRc(int8_t rc) {
  switch (rc)   {
    case fp_ok:
      Serial.println(F("Ok."));
      break;
    case fp_oob:
      Serial.println(F("Result would be out of bounds."));
      break;
    case fp_ncp:
      Serial.println(F("Can't move. Home position not set."));
      break;
    case fp_dis:
      Serial.println(F("Can't move. stepper motors disabled."));
      break;
    case fp_mov:
      Serial.println(F("Can't do this while moving."));
      break;
    case fp_nom:
      Serial.println(F("Can't do this unless moving."));
  }
}

/**
 * 
 * ui command handlers
 * 
 **/

// Unknown
void onUnknown() {
  Serial.println(F("Unknown or unimplemented command."));
}

// enable [0 | 1]
void onEnable() {
  bool en = ui.getWord(1).toInt() != 0;
  bool response = en ? diver.enableOutputs() : diver.disableOutputs();
  if (response == fp_ok) {
    Serial.print(F("Motor drivers "));
    Serial.println(en ? F("enabled.") : F("disabled."));
  } else {
    Serial.println(F("Can't disable motors while moving."));
  }
}

// help and h
void onHelp() {
  Serial.println(F(
    "Commands:\n"
    "  enable <1 | 0>           Enable or disable motor drivers.\n"
    "  help                     Display this text.\n"
    "  h                        Same as help.\n"
    "  home                     Move to the home position.\n"
    "  rest                     Move to the resting position (e.g., before shutdown)\n"
    "  move <mmX> <mmY> <mmZ>   Move by the amounts specified\n"
    "  m <mmX> <mmY> <mmZ>      Same as move.\n"
    "  stop                     Stop all motion.\n"
    "  s                        Same as stop\n"
    "  setHome                  Assume the current position is home.\n"
    "  calibrate                Move to the sensor-defined home position.\n"
    "  status                   Print movement status information\n"
    "  where                    Print (z, y, z) location of diver."));
}

// m and move <x> <y> <z>
void onMove() {
  int8_t rc = diver.moveBy(fp_Point3D {
    mmToSteps(ui.getWord(1).toInt()), 
    mmToSteps(ui.getWord(2).toInt()), 
    mmToSteps(ui.getWord(3).toInt())});
  interpretRc(rc);
}

// sethome
void onSethome(){
  interpretRc(diver.setCurrentPosition(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
  Serial.print(F("Home set to ("));
  Serial.print(HOME_X);
  Serial.print(F(", "));
  Serial.print(HOME_Y);
  Serial.print(F(", "));
  Serial.print(HOME_Z);
  Serial.println(F(")"));
}

// calibrate
void onCalibrate() {
  interpretRc(diver.calibrate(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
  Serial.print(F("Calibrating to ("));
  Serial.print(HOME_X);
  Serial.print(F(", "));
  Serial.print(HOME_Y);
  Serial.print(F(", "));
  Serial.print(HOME_Z);
  Serial.println(F(")"));
}

// home
void onHome(){
  interpretRc(diver.moveTo(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
}

// rest
void onRest(){
  interpretRc(diver.moveTo(fp_Point3D {mmToSteps(HOME_X + REST_DELTA_X), mmToSteps(HOME_Y + REST_DELTA_Y), mmToSteps(HOME_Z + REST_DELTA_Z)}));
}

// s and stop
void onStop() {
  diver.stop();
  Serial.println(F("Stopping."));
}

// status
void onStatus() {
  diver.status();
}

// w and where
void onWhere() {
  fp_Point3D loc = diver.where();
  Serial.print(F("Diver is located at: ("));
  Serial.print(stepsToMm(loc.x));
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.y));
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.z));
  Serial.print(F(")  which is ("));
  Serial.print((stepsToMm(loc.x)) - HOME_X);
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.y) - HOME_Y);
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.z) - HOME_Z);
  Serial.println(F(") from home"));
}

/**
 * 
 * Console event handlers. The console consists of a joystick and three 
 * pushbutton switches. The joystick events are 
 * 
 **/

// toForward
void onToForward() {
  lastTouchMillis = millis();
  if (console.placeLedIsOn() && !diver.isCalibrated() && diver.isEnabled()) {
    #ifdef DEBUG
    Serial.print(F("Console: calibrate. "));
    interpretRc(diver.calibrate(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
    #else
    diver.calibrate(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)});
    #endif
    return;
  }
  #ifdef DEBUG
  Serial.print(F("console: Forward. "));
  interpretRc(diver.go());
  #else
  diver.go();
  #endif
}

// Temp: toBack Need a way to stop media player: joystick back, then press Down
void onToBack() {
  backing = true;
}

// toStop
void onToStop() {
  lastTouchMillis = millis();
  backing = false;  //Temp
  diver.stop();
  #ifdef DEBUG
  Serial.println(F("console: Stop"));
  #endif
}

// toLeft
void onToLeft() {
  lastTouchMillis = millis();
  #ifdef DEBUG
  Serial.print(F("console: Left. "));
  interpretRc(diver.turn(fp_left));
  #else
  diver.turn(fp_left);
  #endif
}

// toNeutral
void onToNeutral() {
  lastTouchMillis = millis();
  #ifdef DEBUG
  Serial.print(F("console: Neutral. "));
  interpretRc(diver.turn(fp_straight));
  #else
  diver.turn(fp_straight);
  #endif
}

// toRight
void onToRight() {
  lastTouchMillis = millis();
  #ifdef DEBUG
  Serial.print(F("console: Right. "));
  interpretRc(diver.turn(fp_right));
  #else
  diver.turn(fp_right);
  #endif
}

// downPressed
void onDownPressed() {
  lastTouchMillis = millis();
  // Temp: Need a way to stop media player: joystick back, then press Down
  if (backing) {
    Serial.println(F("\n!stop")); // Issue stop command to media player
    return;
  }
  #ifdef DEBUG
  Serial.print(F("console: Falling. "));
  interpretRc(diver.turn(fp_falling));
  #else
  diver.turn(fp_falling);
  #endif
}

// downReleased or upReleased
void onUpOrDownReleased() {
  lastTouchMillis = millis();
  #ifdef DEBUG
  Serial.print("console: Level. ");
  interpretRc(diver.turn(fp_level));
  #else
  diver.turn(fp_level);
  #endif
}

// upPressed
void onUpPressed() {
  lastTouchMillis = millis();
  #ifdef DEBUG
  Serial.print(F("console: Rising. "));
  interpretRc(diver.turn(fp_rising));
  #else
  diver.turn(fp_rising);
  #endif
}

// placePressed
void onPlacePressed() {
  lastTouchMillis = millis();
  console.setPlaceLED(true);
  // Temp: Command to media player, just to try it out
  if (diver.isCalibrated() && diver.isEnabled()) {
    diver.stop();
    Serial.println(F("\n!play miss1"));
  }
}

// placeReleased
void onPlaceReleased() {
  lastTouchMillis = millis();
  console.setPlaceLED(false);
}

/**
 *
 * Storyboard trigger handlers
 * 
 **/
// asynch trigger handler
bool onAsynchTrigger(sb_stateid_t s, sb_trigid_t t) {
 return millis() - lastTouchMillis > TIMEOUT_MS;
}

// videoEnds trigger handler
bool onVideoEnds(sb_stateid_t s, sb_trigid_t t) {
  return true;    // Stub
}

/**
 * 
 * Arduino setup() function. Called once at initialization
 * 
 **/
void setup() {
  Serial.begin(9600);
  Serial.println(BANNER);

  diver.begin();
  diver.setMaxSpeed(MAX_SPEED);
  diver.setSafetyMargins(
    mmToSteps(MARGIN_L), mmToSteps(MARGIN_R),
    mmToSteps(MARGIN_F), mmToSteps(MARGIN_B),
    mmToSteps(MARGIN_D), mmToSteps(MARGIN_U));

  // Attach the command handlers for the ui
  ui.attachDefaultCmdHandler(onUnknown);
  bool succeeded = 
    ui.attachCmdHandler("enable", onEnable) &&
    ui.attachCmdHandler("help", onHelp) &&
    ui.attachCmdHandler("h", onHelp) &&
    ui.attachCmdHandler("move", onMove) &&
    ui.attachCmdHandler("m", onMove) &&
    ui.attachCmdHandler("sethome", onSethome) &&
    ui.attachCmdHandler("calibrate", onCalibrate) &&
    ui.attachCmdHandler("home", onHome) &&
    ui.attachCmdHandler("rest", onRest) &&
    ui.attachCmdHandler("stop", onStop) &&
    ui.attachCmdHandler("s", onStop) &&
    ui.attachCmdHandler("status", onStatus) &&
    ui.attachCmdHandler("where", onWhere);
  if (!succeeded) {
    Serial.println(F("Too many command handlers."));
  }

  // Initialize control pad
  console.begin();
  console.attachHandler(clEvent::evForward, onToForward);
  console.attachHandler(clEvent::evBack, onToBack);
  console.attachHandler(clEvent::evStop, onToStop);
  console.attachHandler(clEvent::evLeft, onToLeft);
  console.attachHandler(clEvent::evNeutral, onToNeutral);
  console.attachHandler(clEvent::evRight, onToRight);
  console.attachHandler(clEvent::evUpPressed, onUpPressed);
  console.attachHandler(clEvent::evUpReleased, onUpOrDownReleased);
  console.attachHandler(clEvent::evDownReleased, onUpOrDownReleased);
  console.attachHandler(clEvent::evDownPressed, onDownPressed);
  console.attachHandler(clEvent::evPlacePressed, onPlacePressed);
  console.attachHandler(clEvent::evPlaceReleased, onPlaceReleased);

  Serial.println(F("Type \"h\" for list of commands."));
}

/**
 * 
 * Arduino loop() function. Called repeatedly as soon as it returns.
 * 
 **/
void loop() {
  diver.run();    // Let the diver do its thing
  ui.run();       // Let the UI do its thing
  console.run();  // Let the console do its
}