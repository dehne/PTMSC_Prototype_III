/****
 * PTMSCPrototype V0.80, June 2022
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

#define DEBUG                         // Uncomment to turn on debugging messages
//#define VERBOSE                       // Uncomment for even more debugging messages
#define NO_STORY                      // Uncomment to disable storyboard

// For the stepper motors
#define PIN_0D        (4)             // Motor driver 0 (x = 0, y = 0 winch) direction pin
#define PIN_0P        (5)             //   pulse pin 0
#define PIN_1D        (6)             // Motor driver 1 (x = SIZE_X, y = 0 winch) direction pin
#define PIN_1P        (7)             //   pulse pin 1
#define PIN_2D        (9)             // Motor driver 2 (x = 0, y = SIZE_Y winch) direction pin
#define PIN_2P        (10)            //   pulse pin 2
#define PIN_3D        (11)            // Motor driver 3 (x = SIZE-X, y = SIZE_Y winch) direction pin
#define PIN_3P        (12)            //   pulse pin 3
#define PIN_EN        (13)            // Enable pin - common to all motor drivers (active LOW)
#define MAX_SPEED     (800)           // In steps per second

// For communicating with the calReader and diver Arduino Nanos
#define CAL_CLK_PIN   (2)             // The pin from which the calReader clock signal comes
#define CAL_DATA_PIN  (3)             // The pin from which the calReader data signal comes
#define DVR_CLK_PIN   (39)            // Pin to which the diver's clock signal goes
#define DVR_DATA_PIN  (38)            // Pin to which the diver's data goes

// For the cohort count LEDs
#define COHORT1_PIN   (23)            // Pin to which LED indicating 1 cohort remaining is attached
#define COHORT2_PIN   (25)            // Pin to which LED indicating 2 cohorts remaining is attached
#define COHORT3_PIN   (27)            // Pin to which LED indicating 3 cohorts remaining is attached

// Debugging LED
#ifdef DEBUG
#define DEBUG_LED     (A5)            // We have a green LED attached to this pin for debugging
#endif

// Physical size of flying space (mm) measured from floor and between hoist points
#define SIZE_X        (950)
#define SIZE_Y        (557)
#define SIZE_Z        (777)
#define DIVER_HANG    (330)             // How much (mm) the diver hangs below the suspension point

// Safety margins (mm) for left, right, front, back, down and up
#define MARGIN_L      (110)             // Diver's flippers about this far from lift point
#define MARGIN_R      (110)
#define MARGIN_F      (110)
#define MARGIN_B      (110)
#define MARGIN_D      (DIVER_HANG + 15) // Diver can come within 15mm of bottom
#define MARGIN_U      (130)             // Don't want to go above home height by much at all

// Home location (mm) as measured directly after calibration
#define HOME_X        (471)
#define HOME_Y        (280)
#define HOME_Z        (643)

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
#define TIMEOUT_MS      (60000UL)               // millis() of no activity before exhibit auto-reset
#define NEAR_MM         (30)                    // How close the diver needs to be to be "near" a site (mm)
#define NEAR_RULER      (3 * NEAR_MM * NEAR_MM) // To be "near" deltaX**2 + deltaY**2 + deltaZ**2 must be less than this
#define AWAY_MM         (60)                    // How far the diver needs to be to have moved "away" from a site (mm)
#define AWAY_RULER      (3 * AWAY_MM * AWAY_MM) // To be "away" deltaX**2 + deltaY**2 + deltaZ**2 must be more than this
#define INIT_COHORTS    (3)                     // The number of abalone cohorts the has to outplant

// Misc compile-time definitions
#define BANNER          F("PTMSC Prototype v 0.80 July 2022")

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

fp_Point3D boatLoc = {HOME_X, HOME_Y, HOME_Z - DIVER_HANG};
uint8_t nCohorts = INIT_COHORTS;              // The number of cohorts the diver is currrently carrying
unsigned long lastTouchMillis = 0;            // millis at when the last console evrnt happened
bool videoHasEnded = false;                   // From media player: The last requested video clip finished
bool controlsAreEnabled = false;              // Whether the visitor can make the diver swim

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
 * Set nCohorts. Set the value and deal with the UI
 * 
 */
void setNCohorts(uint8_t n) {
  n = constrain(n, 0, INIT_COHORTS);
  digitalWrite(COHORT1_PIN, n >= 1 ? HIGH : LOW);
  digitalWrite(COHORT2_PIN, n >= 2 ? HIGH : LOW);
  digitalWrite(COHORT3_PIN, n >= 3 ? HIGH : LOW);
  nCohorts = n;
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
    "  toSite <n>               Move to outplacement site <n>\n"
    "  stop                     Stop all motion.\n"
    "  s                        Same as stop\n"
    "  setHome                  Assume the current position is home.\n"
    "  calibrate                Move to the sensor-defined home position.\n"
    "  status                   Print movement status information\n"
    "  where                    Print (z, y, z) location of diver.\n\n"
    #ifdef FP_DEBUG_GEO
    "  doTest                   Debugging: Do a test of the current method of\n"
    "                           batch stepping.\n\n"
    #endif
    "Additional 'pseudo commands' as info from the media player\n"
    "  !videoEnd                Indicates the requested video clip ended"));
}

// m and move <x> <y> <z>
void onMove() {
  int8_t rc = diver.moveBy(fp_Point3D {
    mmToSteps(ui.getWord(1).toInt()), 
    mmToSteps(ui.getWord(2).toInt()), 
    mmToSteps(ui.getWord(3).toInt())});
  interpretRc(rc);
}

// toSite <n>
void onToSite() {
  int siteIx = ui.getWord(1).toInt();
  if (siteIx < 0 || siteIx > SB_SITE_COUNT - 1) {
    Serial.print(F("Invalid site number: "));
    Serial.print(siteIx);
    Serial.print(F(". Must be a number from 0 to "));
    Serial.println(SB_SITE_COUNT - 1);
  }
  fp_Point3D pt = sb_site[siteIx].loc;
  pt.x = mmToSteps(pt.x);
  pt.y = mmToSteps(pt.y);
  pt.z = mmToSteps(pt.z + DIVER_HANG);            // Position is diver_HANG steps directly above the selected site
  interpretRc(diver.moveTo(pt));
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

#ifdef FP_DEBUG_GEO
// doTest
void onDoTest() {
  fp_Point3D src = {mmToSteps(SIZE_X - MARGIN_R), mmToSteps(SIZE_Y / 2), mmToSteps(SIZE_Z - MARGIN_U)};
  fp_Point3D tgt = {mmToSteps(MARGIN_L), mmToSteps(SIZE_Y / 2), mmToSteps(SIZE_Z - MARGIN_U)};
  fp_Point3D nextPoint;
  fp_Point3D batch;
  fp_CableBundle batchCb;
  Serial.print(F("t, x, y, z\n"));
  float dt = 0.03;
  float t = -dt;
  while (t < 1.0) {
    t = min(t + dt, 1.0);
    Serial.print(t);
    Serial.print(F(", "));
    nextPoint.x = src.x + t * (tgt.x - src.x);
    nextPoint.y = src.y + t * (tgt.y - src.y);
    nextPoint.z = src.z + t * (tgt.z - src.z);
    batchCb = diver.p3DToCb(nextPoint);
    batch = diver.cbToP3D(batchCb);
    Serial.print(batch.x);
    Serial.print(F(", "));
    Serial.print(batch.y);
    Serial.print(F(", "));
    Serial.println(batch.z);
  }
}
#endif

// !vidEnd pseudo command -- infor from media player that the requested clip has played
void onPseudoCommandVidEnd() {
  videoHasEnded = true;
}

/**
 * 
 * Console event handlers. The console consists of a joystick and three 
 * pushbutton switches. The joystick events are 
 * 
 **/

// toForward
void onToForward() {
  if (console.placeLedIsOn() && !diver.isCalibrated() && diver.isEnabled()) {
    #ifdef DEBUG
    Serial.print(F("Console: calibrate. "));
    interpretRc(diver.calibrate(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
    #else
    diver.calibrate(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)});
    #endif
    return;
  }
  lastTouchMillis = millis();
  if (!controlsAreEnabled) {
    return;
  }
  #ifdef VERBOSE
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
  backing = false;  //Temp -- see onToBack()
  diver.stop();
  #ifdef VERBOSE
  Serial.println(F("console: Stop"));
  #endif
}

// toLeft
void onToLeft() {
  lastTouchMillis = millis();
  if (!controlsAreEnabled) {
    return;
  }
  #ifdef VERBOSE
  Serial.print(F("console: Left. "));
  interpretRc(diver.turn(fp_left));
  #else
  diver.turn(fp_left);
  #endif
}

// toNeutral
void onToNeutral() {
  lastTouchMillis = millis();
  #ifdef VERBOSE
  Serial.print(F("console: Neutral. "));
  interpretRc(diver.turn(fp_straight));
  #else
  diver.turn(fp_straight);
  #endif
}

// toRight
void onToRight() {
  lastTouchMillis = millis();
  if (!controlsAreEnabled) {
    return;
  }
  #ifdef VERBOSE
  Serial.print(F("console: Right. "));
  interpretRc(diver.turn(fp_right));
  #else
  diver.turn(fp_right);
  #endif
}

// downPressed
void onDownPressed() {
  // Temp: Need a way to stop media player: joystick back, then press Down
  if (backing) {
    Serial.println(F("\n!stop")); // Issue stop command to media player
    return;
  }
  if (!controlsAreEnabled) {
    return;
  }
  #ifdef VERBOSE
  Serial.print(F("console: Falling. "));
  interpretRc(diver.turn(fp_falling));
  #else
  diver.turn(fp_falling);
  #endif
}

// downReleased or upReleased
void onUpOrDownReleased() {
  #ifdef VERBOSE
  Serial.print("console: Level. ");
  interpretRc(diver.turn(fp_level));
  #else
  diver.turn(fp_level);
  #endif
}

// upPressed
void onUpPressed() {
  //Temp: Need a way to have media player toggle fullscreen mode: joystick back then press Up
  if(backing) {
    Serial.println(F("\n!toggleFS"));
    return;
  }
  if (!controlsAreEnabled) {
    return;
  }
  #ifdef VERBOSE
  Serial.print(F("console: Rising. "));
  interpretRc(diver.turn(fp_rising));
  #else
  diver.turn(fp_rising);
  #endif
}

// placePressed
void onPlacePressed() {
  console.setPlaceLED(true);
}

// placeReleased
void onPlaceReleased() {
  console.setPlaceLED(false);
}

/**
 *
 * Storyboard trigger handlers
 * 
 **/
// always trigger handler
bool onAlwaysTrigger(sb_stateid_t s, sb_trigid_t t) {
  return true;
}

// asynchTimer trigger handler
bool onAsynchTimerTrigger(sb_stateid_t s, sb_trigid_t t) {
 return millis() - lastTouchMillis > TIMEOUT_MS;
}

// videoEnds trigger handler
bool onVideoEndsTrigger(sb_stateid_t s, sb_trigid_t t) {
  if (videoHasEnded) {
    videoHasEnded = false;
    return true;
  }
  return false;
}

// touchJoystick trigger handler
bool onTouchJoystickTrigger(sb_stateid_t s, sb_trigid_t t) {
  #ifdef DEBUG
  digitalWrite(DEBUG_LED, digitalRead(DEBUG_LED) == LOW ? HIGH : LOW);
  #endif
  return lastTouchMillis != 0;
}

// Convenience function for nearSitex triggers gathering the distance goop in one place
bool isNearSite(uint8_t siteIx) {
  fp_Point3D diverLoc = diver.where();
  diverLoc.x = stepsToMm(diverLoc.x);
  diverLoc.y = stepsToMm(diverLoc.y);
  diverLoc.z = stepsToMm(diverLoc.z) - DIVER_HANG;
  fp_Point3D delta;
  if (((delta.x = abs(sb_site[siteIx].loc.x - diverLoc.x)) > NEAR_MM) ||
      ((delta.y = abs(sb_site[siteIx].loc.y - diverLoc.y)) > NEAR_MM) ||
      ((delta.z = abs(sb_site[siteIx].loc.z - diverLoc.z)) > NEAR_MM) ||
      (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) > NEAR_RULER) {
    return false;
  }
  return true;
}

// nearOpenSiteCohorts<n> trigger handler for all sites
bool onNearOpenSiteCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  uint16_t siteIx = (uint8_t)t - (uint8_t)nearOpenSite1Cohorts;
  return isNearSite(siteIx) && !sb_site[siteIx].isFull && nCohorts > 0;
}

// nearFullSiteCohorts<n> trigger handler for all sites
bool onNearFullSiteCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  uint16_t siteIx = (uint8_t)t - (uint8_t)nearFullSite1Cohorts;
  return isNearSite(siteIx) && sb_site[siteIx].isFull && nCohorts > 0;
}

// nearSiteNoCohorts<n> trigger handler for all sites
bool onNearSiteNoCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  uint16_t siteIx = (uint8_t)t - (uint8_t)nearSite1NoCohorts;
  return isNearSite(siteIx) && nCohorts == 0;
}

// awayFromSite<n> trigger handler for all sites
bool onAwayFromSiteTrigger(sb_stateid_t s, sb_trigid_t t) {
  fp_Point3D diverLoc = diver.where();
  diverLoc.x = stepsToMm(diverLoc.x);
  diverLoc.y = stepsToMm(diverLoc.y);
  diverLoc.z = stepsToMm(diverLoc.z) - DIVER_HANG;
  uint16_t siteIx = (uint8_t)t - (uint8_t)awayFromSite1;
  fp_Point3D delta = {abs(sb_site[siteIx].loc.x - diverLoc.x), abs(sb_site[siteIx].loc.y - diverLoc.y), abs(sb_site[siteIx].loc.z - diverLoc.z)};
  if (delta.x > AWAY_MM || delta.y > AWAY_MM || delta.z > AWAY_MM || 3 * (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) > AWAY_RULER) {
        #ifdef VERBOSE
        Serial.print(F("onAwayFromSiteTrigger() siteIx: "));
        Serial.print(siteIx);
        Serial.print(F(", delta: "));
        Serial.print(delta.x);
        Serial.print(F(" "));
        Serial.print(delta.y);
        Serial.print(F(" "));
        Serial.print(delta.z);
        Serial.print(F(", diverLoc: "));
        Serial.print(diverLoc.x);
        Serial.print(F(" "));
        Serial.print(diverLoc.y);
        Serial.print(F(" "));
        Serial.print(diverLoc.z);
        Serial.print(F(", sb_site[siteIx]: "));
        Serial.print(sb_site[siteIx].loc.x);
        Serial.print(F(" "));
        Serial.print(sb_site[siteIx].loc.y);
        Serial.print(F(" "));
        Serial.println(sb_site[siteIx].loc.z);
        #endif
        return  true;
  }
  return false;
}

// videoEndsCohorts trigger handler
bool onVideoEndsCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  if (videoHasEnded && nCohorts > 0) {
    videoHasEnded = false;
    return true;
  }
  return false;
}

// videoEndsNoCohorts trigger handler
bool onvideoEndsNoCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  if (videoHasEnded && nCohorts == 0) {
    videoHasEnded = false;
    return true;
  }
  return false;
}

// Convenience function for near boat trigger handlers
bool isNearBoat() {
  fp_Point3D diverLoc = diver.where();
  fp_Point3D delta;
  diverLoc.x = stepsToMm(diverLoc.x);
  diverLoc.y = stepsToMm(diverLoc.y);
  diverLoc.z = stepsToMm(diverLoc.z) - DIVER_HANG;
  if (((delta.x = abs(boatLoc.x - diverLoc.x)) > NEAR_MM) ||
      ((delta.y = abs(boatLoc.y - diverLoc.y)) > NEAR_MM) ||
      ((delta.z = abs(boatLoc.z - diverLoc.z)) > NEAR_MM) ||
      3 * (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) > NEAR_RULER) {
    return false;
  }
  return true;
}

// nearBoatCohorts trigger handler
bool onNearBoatCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  return isNearBoat() && nCohorts > 0;
}

// nearBoatNoCohorts trigger handler
bool onNearBoatNoCohortsTrigger(sb_stateid_t s, sb_trigid_t t) {
  return isNearBoat() && nCohorts == 0;
}

// awayFromBoat
bool onAwayFromBoatTrigger(sb_stateid_t s, sb_trigid_t t) {
  fp_Point3D diverLoc = diver.where();
  diverLoc.x = stepsToMm(diverLoc.x);
  diverLoc.y = stepsToMm(diverLoc.y);
  diverLoc.z = stepsToMm(diverLoc.z) - DIVER_HANG;
  fp_Point3D delta = {abs(boatLoc.x - diverLoc.x), abs(boatLoc.y - diverLoc.y), abs(boatLoc.z - diverLoc.z)};
  if (delta.x > AWAY_MM || delta.y > AWAY_MM || delta.z > AWAY_MM || 3 * (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) > AWAY_RULER) {
    return true;
  }
  return false;
}

// pressPlaceButton trigger handler
bool onPressPlaceButtonTrigger(sb_stateid_t s, sb_trigid_t t) {
  return console.placeLedIsOn();
}

// calibrated trigger handler
bool onCalibratedTrigger(sb_stateid_t s, sb_trigid_t t) {
  return diver.isCalibrated();
}

// convenience function for score trigger handlers 0 ==> meh, 1 ==> good, 2 ==> super
uint8_t getScore() {
  uint8_t answer = sb_site[0].isFull ? 3 : 0;
  answer += sb_site[1].isFull ? 3 : 0;
  answer += sb_site[2].isFull ? 1 : 0;
  answer += sb_site[3].isFull ? 4 : 0;
  answer += sb_site[4].isFull ? 4 : 0;
  answer = answer > 10 ? 2 : answer > 9 ? 1 : 0;
  return answer; 
}

// superScore trigger handler
bool onSuperScoreTrigger(sb_stateid_t s, sb_trigid_t t) {
  return getScore() == 2;
}

// goodScore trigger handler
bool onGoodScoreTrigger(sb_stateid_t s, sb_trigid_t t) {
  return getScore() == 1;
}

// mehScore trigger handler
bool onMehScoreTrigger(sb_stateid_t s, sb_trigid_t t) {
  return getScore() == 0;
}

/**
 * 
 * Storyboard action handlers
 * 
 */
// null action handler
void onNullAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  return;
}

// setLoop action handler
void onSetLoopAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  Serial.print(F("!setLoop "));
  Serial.println(clipId);
}

// playClip action handler
void onPlayClipAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  Serial.print(F("!playClip "));
  Serial.println(clipId);
}

// maybePlayClip action handler
void onMaybePlayClipAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  uint8_t siteIx = stateId - (uint8_t)resultSite1;
  #ifdef DEBUG
  if (siteIx > SB_SITE_COUNT) {
    Serial.print(F("Ignored maybePlayClipAction with invalid site index: "));
    Serial.print(siteIx);
    Serial.print(F(" from state "));
    Serial.println(stateId);
  } else 
  #endif
  if (sb_site[siteIx].isFull) {
    onPlayClipAction(stateId, actionId, clipId);  // Play clip if the corresponding site is full
  } else {
    videoHasEnded = true;                         // Otherwise just pretend we played it
  }
}

// prepareNew action handler
void onPrepareNewAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  onHome();                                     // Back to the boat
  setNCohorts(INIT_COHORTS);                    // Get stuff to outplant
  for (uint8_t siteIx = 0; siteIx < SB_SITE_COUNT; siteIx++) {
    sb_site[siteIx].isFull = false;             // Empty the sites of putplants
  }
  controlsAreEnabled = true;                    // Let it rip!
  lastTouchMillis = 0;
}

// disableControls action handler
void onDisableControlsAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  controlsAreEnabled = false;
}

// deposit<n> action handler for all sites
void onDepositAction(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  uint8_t siteIx = (uint8_t)stateId - (uint8_t)fillSite1; // Determint which site we're depositing in
  sb_site[siteIx].isFull = true;                  // Make the deposit
  setNCohorts(nCohorts - 1);
  #ifdef DEBUG
  Serial.print(F("Depositing at site "));
  Serial.print(siteIx + 1);
  Serial.print(F(" cohorts after deposit: "));
  Serial.println(nCohorts);
  #endif
}

// doSurvivalSequence 
void onDoSurvivalSequence(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
  Serial.println(F("Stub: doSurvivalSequence."));
}

/** 
 * 
 * Arduino setup() function. Called once at initialization
 * 
 **/
void setup() {
  Serial.begin(9600);
  Serial.println(BANNER);

  #ifdef DEBUG
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  #endif

  // Initialize the cohort UI LEDs
  pinMode(COHORT1_PIN, OUTPUT);
  digitalWrite(COHORT1_PIN, LOW);
  pinMode(COHORT2_PIN, OUTPUT);
  digitalWrite(COHORT2_PIN, LOW);
  pinMode(COHORT3_PIN, OUTPUT);
  digitalWrite(COHORT3_PIN, LOW);

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
    ui.attachCmdHandler("toSite", onToSite) &&
    ui.attachCmdHandler("sethome", onSethome) &&
    ui.attachCmdHandler("calibrate", onCalibrate) &&
    ui.attachCmdHandler("home", onHome) &&
    ui.attachCmdHandler("rest", onRest) &&
    ui.attachCmdHandler("stop", onStop) &&
    ui.attachCmdHandler("s", onStop) &&
    ui.attachCmdHandler("status", onStatus) &&
    ui.attachCmdHandler("where", onWhere) &&
    #ifdef FP_DEBUG_GEO
    ui.attachCmdHandler("doTest", onDoTest) &&
    #endif
    ui.attachCmdHandler("!videoEnds", onPseudoCommandVidEnd);
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

  // Initialize the Storyboard trigger handlers
  sb->attachTriggerHandler(always, onAlwaysTrigger);
  sb->attachTriggerHandler(asynchTimer, onAsynchTimerTrigger);
  sb->attachTriggerHandler(videoEnds, onVideoEndsTrigger);
  sb->attachTriggerHandler(touchJoystick, onTouchJoystickTrigger);
  sb->attachTriggerHandler(nearOpenSite1Cohorts, onNearOpenSiteCohortsTrigger);
  sb->attachTriggerHandler(nearOpenSite2Cohorts, onNearOpenSiteCohortsTrigger);
  sb->attachTriggerHandler(nearOpenSite3Cohorts, onNearOpenSiteCohortsTrigger);
  sb->attachTriggerHandler(nearOpenSite4Cohorts, onNearOpenSiteCohortsTrigger);
  sb->attachTriggerHandler(nearOpenSite5Cohorts, onNearOpenSiteCohortsTrigger);
  sb->attachTriggerHandler(nearFullSite1Cohorts, onNearFullSiteCohortsTrigger);
  sb->attachTriggerHandler(nearFullSite2Cohorts, onNearFullSiteCohortsTrigger);
  sb->attachTriggerHandler(nearFullSite3Cohorts, onNearFullSiteCohortsTrigger);
  sb->attachTriggerHandler(nearFullSite4Cohorts, onNearFullSiteCohortsTrigger);
  sb->attachTriggerHandler(nearFullSite5Cohorts, onNearFullSiteCohortsTrigger);
  sb->attachTriggerHandler(nearSite1NoCohorts, onNearSiteNoCohortsTrigger);
  sb->attachTriggerHandler(nearSite2NoCohorts, onNearSiteNoCohortsTrigger);
  sb->attachTriggerHandler(nearSite3NoCohorts, onNearSiteNoCohortsTrigger);
  sb->attachTriggerHandler(nearSite4NoCohorts, onNearSiteNoCohortsTrigger);
  sb->attachTriggerHandler(nearSite5NoCohorts, onNearSiteNoCohortsTrigger);
  sb->attachTriggerHandler(awayFromSite1, onAwayFromSiteTrigger);
  sb->attachTriggerHandler(awayFromSite2, onAwayFromSiteTrigger);
  sb->attachTriggerHandler(awayFromSite3, onAwayFromSiteTrigger);
  sb->attachTriggerHandler(awayFromSite4, onAwayFromSiteTrigger);
  sb->attachTriggerHandler(awayFromSite5, onAwayFromSiteTrigger);
  sb->attachTriggerHandler(videoEndsCohorts, onVideoEndsCohortsTrigger);
  sb->attachTriggerHandler(videoEndsNoCohorts, onvideoEndsNoCohortsTrigger);
  sb->attachTriggerHandler(nearBoatCohorts, onNearBoatCohortsTrigger);
  sb->attachTriggerHandler(nearBoatNoCohorts, onNearBoatNoCohortsTrigger);
  sb->attachTriggerHandler(awayFromBoat, onAwayFromBoatTrigger);
  sb->attachTriggerHandler(pressPlaceButton, onPressPlaceButtonTrigger);
  sb->attachTriggerHandler(calibrated, onCalibratedTrigger);
  sb->attachTriggerHandler(superScore, onSuperScoreTrigger);
  sb->attachTriggerHandler(goodScore, onGoodScoreTrigger);
  sb->attachTriggerHandler(mehScore, onMehScoreTrigger);

  // Initialize the Storyboard action handlers
  sb->attachActionHandler(nullAction, onNullAction);
  sb->attachActionHandler(setLoop, onSetLoopAction);
  sb->attachActionHandler(playClip, onPlayClipAction);
  sb->attachActionHandler(prepareNew, onPrepareNewAction);
  sb->attachActionHandler(disableControls, onDisableControlsAction);
  sb->attachActionHandler(deposit1, onDepositAction);
  sb->attachActionHandler(deposit2, onDepositAction);
  sb->attachActionHandler(deposit3, onDepositAction);
  sb->attachActionHandler(deposit4, onDepositAction);
  sb->attachActionHandler(deposit5, onDepositAction);
  sb->attachActionHandler(maybePlayClip, onMaybePlayClipAction);

  #ifndef NO_STORY
  sb->begin();  //Start the state machine running
  #else
  controlsAreEnabled = true;
  #endif

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
  #ifndef NO_STORY
  sb->run();      // Let the storyboard do its thing
  #endif
}