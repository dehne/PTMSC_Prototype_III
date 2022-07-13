/****
 * 
 * This file is a portion of the package FlyingPlatform, a library that 
 * provides an Arduino sketch running on an Arduino Mega 2560 Rev3 with the 
 * ability to control four stepper motors (through stepper motor drivers) to 
 * operate a "flying camera" setup (technically, a parallel cable-driven 
 * robot) in which a platform is connected by cables to four stepper-motor-
 * controlled winches. By judiciously reeling in and letting out the cables, 
 * the platform can be moved around in three space.
 * 
 * See FlyingPlatform.h for details.
 *
 *****
 * 
 * FlyingPlatform V0.75, September 2021
 * Copyright (C) 2020-2021 D.L. Ehnebuske
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

#include "FlyingPlatform.h"
#include <util/atomic.h>


/**
 * 
 * Variables held in common with the ISRs. I don't like this, but I can find no 
 * clean way of sharing variables with ISRs.
 * 
 **/

// Timer ISR
volatile static bool nextReady;             // Lock for sharing nextXxxx variables. true => ISR owns false => run()
static bool nextShortening[4];              // Buffer for Whether the cable is shortening or lengthening in next batch
static long nextPendingSteps[4];            // Buffer for pendingSteps in next batch of steps
static unsigned long nextDsInterval[4];     // Buffer for interval between steps (μs) in next batch

volatile static bool timerISRHasWork;       // True if timer ISR still has steps to take. (Owned by ISR)
volatile static long cableSteps[4];         // The current length of the cables, in steps (Owned by ISR; use ATOMIC access)
static byte dirPin[4];                      // Direction digital output pins. High means lengthen cables (Owned by object)
static byte stepPin[4];                     // Step digital output pins. Normally LOW. Pulse HIGH to step. (Owned by object)
volatile static bool enabled;               // Whether motor drivers are enabled
volatile static bool calibrated;            // True if we know where the platform is

#ifdef FP_DEBUG_ISR
volatile static bool late = false;          // Set true if didn't dispatch step on time, reset after reporting
volatile static bool dump = false;          // Set true to have ISR fill in debugging vars (below). ISR resets when done
volatile static bool captured = false;      // Set true by ISR when debugging vars captured reset by run() after printing dump
volatile static byte tNFinished;            // nFinished at time of dump
volatile static unsigned long tCurMicros;   // curMicros and targetMicros at time of dump
volatile static unsigned long tTargetMicros[4];
#endif

// calReader ISR
static byte calClkPin;                      // The calReader clock pin (Owned by object)
static byte calDataPin;                     // The calReader data pin (Owned by object)
static bool doCal;                          // Set true to trigger calibration, false to stop one. ISR resets when calibration finishes

// diver ISR
static byte diverClkPin;                    // The diver's clock pin (Owned by object)
static byte diverDataPin;                   // The diver's data pin (Owned by object)

#ifdef FP_DEBUG_CAL_ISR
volatile static bool newCablePos;           // Set true by ISR on a cablePos change. Reset by loop() when printed
volatile static bool newBead;               // Set true by ISR when curBeads is updated. Reset by loop() when printed
volatile static byte curBeads;              // Set to the calReader when the state of calReader changes
volatile static cableState nsCablePos;      // ISR cablePos at state change
volatile static winchState nsWinchDir;      // ISR winchDir at state change
#endif

// Shared between Timer ISR and calReader ISR
enum winchState : byte {shortening, paused, lengthening};
volatile static winchState winchDir[4];     // If/how the winches are moving

/**
 * 
 * Timer/Counter3 interrupt service routine. This is the ISR that drives the 
 * steppers that move the diver around the exhibit. It is invoked whenever the 
 * count in Timer/Counter3 reaches the value specified in register OCR3A. 
 * Because Timer/Counter3 is set up (in begin()) to tick once every 4μs and 
 * OCR3A is a 16-bit register, the longest we can go between interrupts is 4 x 
 * 2^^16 = 262144μs. We adjust OCR3A as needed to cause interrupts at shorter 
 * intervals.
 * 
 * Basically, when an interrupt occurs, after checking to see if there's 
 * anything at all to do, we check to see if it's time to load a new batch of 
 * steps. If so, we copy the batch data from the nextXxxx variables to the
 * live ones and reset the nextReady lock to indicate we did so. In the 
 * process we also set the direction pin on each motor so that steps taken 
 * during the batch will be in the correct direction.
 * 
 * Next we dispatch any steps whose time has come. Dispatching a step consists 
 * of emiting a HIGH pulse on the appropriate GPIO pin. In the process we also 
 * figure out if there are any steps whose time will become due during the 
 * next ms, and, if so, what micros() for it will be when it's due.
 * 
 * Finally, if there is a step whose time will come due inside the next 
 * 262144μs, we adjust the value of OCR3A so that an interrupt will occur at 
 * the correct time. If there's isn't one, we set OCR3A to 0xFFFF to cause the 
 * next interrupt to happen in 262144μs.
 * 
 **/
ISR(TIMER3_COMPA_vect) {
    static byte nFinished = 4;                              // Number of motors finished with this batch
    static long pendingSteps[4] = {0, 0, 0, 0};             // Number of remaining steps by which to change each cable
    static unsigned long dsInterval[4];                     // Interval between each cable's steps (μs)
    static unsigned long targetMicros[4] =  {0, 0, 0, 0};   // When to dispatch next step for each cable
    static unsigned long lastMicros = 0;                    // At entry to interrupt: curMicros for last interrupt
    bool microsOflow;                                       // true if micros() wrapped around since last interrupt

    unsigned long curMicros = micros();                     // micros() at entry to ISR
    microsOflow = curMicros < lastMicros;                   // True if micros() overflowed since last interrupt
    lastMicros = curMicros;

    #ifdef FP_DEBUG_ISR
    digitalWrite(FP_ISR_I_PIN, HIGH);                       // Show we're in the ISR
    if (dump && !captured) {                                // If dump requested and it's possible to do it
        tCurMicros = curMicros;                             //  Do it
        tNFinished = nFinished;
        for (byte i = 0; i < 4; i++) {
            tTargetMicros[i] = targetMicros[i];
        }
        dump = false;                                       // And indicate we did
        captured = true;
    }
    #endif

    if (nFinished == 4) {                                   // Finished with batch
        if (!nextReady) {                                   // No work to do if no new batch ready
            timerISRHasWork = false;
            OCR3A = 0xFFFF;                                 // Interrupt in the longest possible interval
            #ifdef FP_DEBUG_ISR
            digitalWrite(FP_ISR_T_PIN, HIGH);               // Show we're in long interrupt
            digitalWrite(FP_ISR_W_PIN, LOW);                // With no pending work
            digitalWrite(FP_ISR_I_PIN, LOW);                // And not in the ISR
            #endif
            return;
        }
        timerISRHasWork = true;
        digitalWrite(FP_ISR_W_PIN, HIGH);                   // Show we have work to do
        // Copy batch data from nextXxx variables to ISR's variables; set rotation directions
        for (byte i = 0; i < 4; i++) {
            pendingSteps[i] = nextPendingSteps[i];
            if (nextShortening[i]) {
                digitalWrite(dirPin[i], fp_shorten(i));    // Set to shorten cable
                winchDir[i] = winchState::shortening;
            } else {
                digitalWrite(dirPin[i], fp_lengthen(i));   // Set to lengthen cable
                winchDir[i] = winchState::lengthening;
            }
            dsInterval[i] = nextDsInterval[i];
            targetMicros[i] = curMicros;
        }
        nextReady = false;                  // Hand nextXxx variables back to object
    }

    // Process any steps that are ready. Along the way, discover micros() at 
    // which we'll need to process the next step(s). FP_MAX_ISR_TIME needs to 
    // be big enough so that there is time to get through dispatching these 
    // steps and set OCR0A and exit before shortestT passes by or we'll miss a 
    // whole ms.
    nFinished = 0;
    unsigned long shortestT = 1000000;                  // Assume micros to next interrupt will be a long way off
    for (byte i = 0; i < 4; i++) {
        if (pendingSteps[i] > 0 && 
            ((!microsOflow && curMicros >= targetMicros[i] - FP_MAX_ISR_TIME) || (microsOflow && targetMicros[i] - FP_MAX_ISR_TIME >= curMicros))) {
            digitalWrite(stepPin[i], HIGH);             // Start step pulse
            cableSteps[i] += winchDir[i] == winchState::shortening ? -1 : 1;
            pendingSteps[i]--; 
            targetMicros[i] += dsInterval[i];           // Figure micros() at next step
            #ifdef FP_DEBUG_ISR
            if ((!microsOflow && targetMicros[i] < curMicros) || (microsOflow && curMicros > targetMicros[i])) {
                late = true;
            }
            #endif
            digitalWrite(stepPin[i], LOW);              // Complete step pulse
        }
        if (pendingSteps[i] == 0) {
            nFinished++;                                // Mark that we're done with stepper[i]
            winchDir[i] = winchState::paused;
        } else if (targetMicros[i] - curMicros < shortestT) {       // Otherwise see if we have a new shortest time to next interrupt
            shortestT = targetMicros[i] - curMicros;
        }
    }

    // Assuming we're not finished with a batch or, if we are, that there's 
    // not another batch ready to go, shortestT now holds the number of 
    // micros() past curMicros at which the next interrupt should occur. 
    // If on the other hand we're done with a batch, and there's a new one 
    // ready to go, we need an interrupt quickly get working on it. Either 
    // way, figure out what OCR3A should be to interrupt appropriately. 
    unsigned long microsToGo = (nFinished < 4 || !nextReady) ? (shortestT) : FP_MAX_ISR_TIME;
    if (microsToGo < 262144UL) {            // If not too long for the hardware
        OCR3A = (microsToGo >> 2) & 0xFFFF; // Interrupt in microsToGo μs
        #ifdef FP_DEBUG_ISR
        digitalWrite(FP_ISR_T_PIN, LOW);    // Show we're not in long interrupt
        #endif
    } else {
        OCR3A = 0xFFFF;                     // Interrupt in the longest possible interval
        #ifdef FP_DEBUG_ISR
        digitalWrite(FP_ISR_T_PIN, HIGH);   // Show we're in long interrupt
        #endif
    }
    #ifdef FP_DEBUG_ISR
    digitalWrite(FP_ISR_I_PIN, LOW);        // Show we're not in the ISR
    #endif
}

/****
 * 
 * Timer/Counter4 ISR Interrupts come every TICKS_PER_INT * 4μs
 * 
 * This ISR sends data  to the diver about what heading to adopt and what 
 * colors to display on her outplacement device. It can also be used to send a 
 * "calibrate yourself" command to cause her turn so she's facing directly to 
 * the right in the display and to make be heading 0.
 * 
 * Data is sent in 8-bit packets. These have the form:
 * 
 * 	ttdddddd
 * 
 * Where tt tells the type of packet and dddddd is the data for that packet. 
 * The types are:
 * 
 * 	tt = 00 Heading packet
 * 	tt = 01 Pixel 0 packet
 * 	tt = 10 Pixel 1 packet
 * 	tt = 11 Control packet
 * 
 * For Heading packets, dddddd is the heading to adopt. Its value, 0..39, is 
 * that of hHeading. For Pixel packets, dddddd is the rrggbb value, where rr, 
 * gg, and bb can be 0 (off) 1 .. 3 (max bright). Pixel 0 is the forward-
 * facing pixel, pixel 1 faces backward. For Control packets, dddddd = 0 
 * is an idle packet which can be used to establish sync without changing 
 * anything. dddddd = 1 means do a homing operation. Other values are 
 * reserved.
 * 
 * Transmission occurs in transmission groups of 1 to three packets per group. 
 * A group starts with at least FP_DIVER_MIN_SILENCE μs of 0 (LOW) on the 
 * clock line. Thereafter bits are sent at a nominal rate of FP_DIVER_BPS bits 
 * per second.
 * 
 * To have the ISR send data: 
 * 
 *  1. Wait until packetCount == 0, indicating there is no transmission 
 *     currently underway.
 *  2. Fill packets[], starting with packets[0], with the packets to be transmitted.
 *  3. Set packetCount to the number of packets to be sent.
 *  4. When it completes the transmission, the ISR will set packetCount to 0
 * 
 * Transmission is controlled by a state machine. Sending a bit occurs over 
 * the course of three interrupts. The first places the bit value on DATA_PIN. 
 * The second sets CLOCK_PIN to HIGH. The third sets CLOCK_PIN back to LOW.
 * 
 ****/

uint8_t packets[3] = {0};                               // The packets to send. packetCount == 0, owned by ISR else owned by loop()
uint8_t packetCount = 0;                                // Count of packets ISR is to send in this transmission group

ISR(TIMER4_COMPA_vect) {
    static uint8_t curPacket = 0;                       // The number of the packet to send
    static uint8_t curBit = 0;                          // The number of the current bit in packet to send. LSB == bit 0
    static unsigned long silentStart = micros();        // micros() when clock line went LOW at end of last transmission group
    enum xState_t : uint8_t {idle, silent, startGroup, startPacket, sendBit, clockHigh, clockLow};
    static xState_t state = xState_t::idle;             // The current state of the state machine

    while (true) {                                      // Doing a "break;" in the case statements loops. 
        switch (state) {                                // Doing a "return;" waits for the next interrupt
            case xState_t::idle:                        // Idling (Waiting for work)
                if (packetCount == 0) {                 //   If nothing to do, wait for the next interrupt
                    return;
                }
                #ifdef FP_DEBUG_DIVER
                digitalWrite(FP_LED_PIN, HIGH);          //   Show we're working
                #endif
                state = xState_t::silent;               //   Otherwise we need to transmit a new group. To get there we need a period of silence.
                break;                                  //   Get on with it

            case xState_t::silent:                      // Waiting for MIN_SILENCE μs to have passed since the last bit was sent
                if (micros() - silentStart < FP_DIVER_MIN_SILENCE) {
                    return;                             //   If we're not there yet, wait for the next interrupt.
                }
                state = startGroup;                     //   Okay, lets get on with the transmitting the group
                break;

            case xState_t::startGroup:                  // Starting a new transmission group
                curPacket = 0;                          //   Start with packet 0
                state = xState_t::startPacket;          //   Get on with it
                break;

            case xState_t::startPacket:                 // Starting to send packet curPack of the transmission group
                curBit = 0;                             //   Start with bit 0
                state = xState_t::sendBit;              //   Get on with it
                break;

            case xState_t::sendBit:                     // Sending bit curBit of packet curPacket
                digitalWrite(diverDataPin, (packets[curPacket] & (0x01 << curBit++)) != 0 ? HIGH : LOW);
                state = xState_t::clockHigh;            //   Set the bit value (and increment curBit). Next up: state clockHigh
                return;                                 //   Wait for the next interrupt

            case xState_t::clockHigh:                   // Let receiver know there's a bit ready
                digitalWrite(diverClkPin, HIGH);        //   Set CLOCK_PIN to HIGH
                state = xState_t::clockLow;             //   Next up: state clockLow
                return;                                 //   Wait for next interrupt

            case xState_t::clockLow:                    // Let receiver know the data might not be good any more and get ready for what's next
                digitalWrite(diverClkPin, LOW);         //   Set CLOCK_PIN to LOW
                if (curBit < 8) {                       //   If we're in the middle of a packet, next up: sending the next bit
                    state = xState_t::sendBit;
                    return;                             //   Wait for next interrupt
                }
                curPacket++;                            //    We're finished with that packet. Move to the next, if any
                if (curPacket < packetCount) {          //    If there are more packets in the transmission group, next up: start the next one
                    state = xState_t::startPacket;
                    return;                             //    Wait for next interrupt
                }
                packetCount = 0;                        //   We finished the transmission group. Mark the work as done.
                silentStart = micros();                 //   Remember when we finished for the silence before the next transmission group
                state = xState_t::idle;                 //   Next up: Idling until until there's more work
                #ifdef FP_DEBUG_DIVER
                digitalWrite(FP_LED_PIN, LOW);         //   Show we're idle
                #endif
                return;                                 //   Wait for next interrupt
        }
    }
}

/**
 * 
 * Interrupt service routine used to deal with calReader data. This ISR is 
 * attached to CAL_CLK_PIN. It reads calReader data, gathers it into packets 
 * and processes the packets as they become available. Normally, the ISR 
 * just keeps track of how long the cables are compared to their calibration 
 * points. But it can also do a calibration, i.e., drive the motors to move 
 * the cables to the calibration points.
 * 
 * To do a calibration, set doCal to true. Don't try to run the motors when 
 * doCal is true. To stop a calibration already in progress, simply set 
 * doCal to false.
 * 
 **/
void calClkISR() {
    static volatile int calData = 0;            // The data received from calReader
    static bool lastDoCal = false;              // doCal the last time through
    enum cableState : byte {tooShort, shortish, atCal, tooLong};
    static volatile cableState cablePos[4];     // Current position of the cables

    // Read the incoming bit; return if packet still incomplete
      calData = (calData << 1) | digitalRead(calDataPin);
    if ((calData & FP_CAL_SYNC_MASK) != FP_CAL_SYNC) {
        return;
      }
    #ifdef FP_DEBUG_CAL_ISR
    if ((calData & 0x000F) != curBeads) {
            curBeads = calData & 0x000F;
            newBead = true;
        }
    #endif

    // Update the state of the cable lengths
    for (byte cable = 0; cable < 4; cable++) {
        if (winchDir[cable] != paused) {                        // Only update if winching
            byte reading = (calData >> (3 - cable) & 1);
            if (reading == FP_CAL_B) {                          // Magnet present at Hall sensor
                if (winchDir[cable] == winchState::shortening) {
                    if (cablePos[cable] == cableState::tooLong) {
                        cablePos[cable] = cableState::atCal;    // magnet, shortening, tooLong ==> arrived at calibration point
                        #ifdef FP_DEBUG_CAL_ISR
                        newCablePos = true;
                        #endif
                    } else if (cablePos[cable] == cableState::atCal) {
                        cablePos[cable] = cableState::tooShort; // magnet, shortening, atCal ==> cable now too short
                        #ifdef FP_DEBUG_CAL_ISR
                        newCablePos = true;
                        #endif
                    }
                } else {
                    if (cablePos[cable] == cableState::tooShort) { 
                        cablePos[cable] = cableState::shortish;
                        #ifdef FP_DEBUG_CAL_ISR
                        newCablePos = true;
                        #endif
                    } else if (cablePos[cable] == cableState::atCal) {
                        cablePos[cable] = cableState::tooLong; // magnet, lengthening, atCal ==> cable now too long
                        #ifdef FP_DEBUG_CAL_ISR
                        newCablePos = true;
                        #endif
                    }
                }
            } else {                                        // No magnet present
                if (cablePos[cable] == cableState::atCal || cablePos[cable] == cableState::shortish) {
                    cablePos[cable] = winchDir[cable] == winchState::shortening ? cableState::tooShort : cableState::tooLong;
                                                            // no magent, shortish, shortening, ==> cable now too short
                                                            // no magnet, shortish, lengthening ==> cable now too long
                                                            // no magnet, at calibration point, shortening ==> cable now too short
                                                            // no magnet, at calibration point, lengthening ==> cable now too long
                    #ifdef FP_DEBUG_CAL_ISR
                    newCablePos = true;
                    #endif
                }
            }
        }
    }

    // Deal with initiating a calibration run
    if (doCal && !lastDoCal) {
        for (byte i = 0; i < 4; i++) {
            cablePos[i] = cableState::tooLong;
        }
    }
    lastDoCal = doCal;

    // Take a step in the calibration if we're doing one
    if (doCal) {
        byte doneCount = 0;
        for (byte cable = 0; cable < 4; cable++) {
            switch (cablePos[cable]) {
                case cableState::tooLong:
                digitalWrite(dirPin[cable], fp_shorten(cable));
                digitalWrite(stepPin[cable], HIGH);
                digitalWrite(stepPin[cable], LOW);
                winchDir[cable] = winchState::shortening;
                break;
                case cableState::tooShort:
                case cableState::shortish:
                digitalWrite(dirPin[cable], fp_lengthen(cable));
                digitalWrite(stepPin[cable], HIGH);
                digitalWrite(stepPin[cable], LOW);
                winchDir[cable] = winchState::lengthening;
                break;
                case atCal:
                doneCount++;
                winchDir[cable] = paused;
                break;
            }
        }
        if (doneCount == 4) {
            doCal = false;
        }
    }
    return;
}

FlyingPlatform::FlyingPlatform(
    byte pin0D, byte pin0P, 
    byte pin1D, byte pin1P, 
    byte pin2D, byte pin2P, 
    byte pin3D, byte pin3P,
    byte pinEn,
    byte pinCC, byte pinCD,
    byte pinDC, byte pinDD,
    long spaceWidth, long spaceDepth, long spaceHeight) {
        dirPin[0] = pin0D;
        dirPin[1] = pin1D;
        dirPin[2] = pin2D;
        dirPin[3] = pin3D;
        stepPin[0] = pin0P;
        stepPin[1] = pin1P;
        stepPin[2] = pin2P;
        stepPin[3] = pin3P;
        enablePin = pinEn;
        calClkPin = pinCC;
        calDataPin = pinCD;
        diverClkPin = pinDC;
        diverDataPin = pinDD;
        space = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
        marginsMin = fp_Point3D {0, 0, 0};
        marginsMax = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
        targetsMin = fp_Point3D {marginsMin.x + FP_TARGETS_BUFFER, marginsMin.y + FP_TARGETS_BUFFER, marginsMin.z + FP_TARGETS_BUFFER};
        targetsMax = fp_Point3D {marginsMax.x - FP_TARGETS_BUFFER, marginsMax.y - FP_TARGETS_BUFFER, marginsMax.z + FP_TARGETS_BUFFER};
    }

void FlyingPlatform::begin() {
    pinMode(enablePin, OUTPUT);
    for (byte i = 0; i < 4; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(stepPin[i], OUTPUT);
        cableSteps[i] = 0;          // Timer/Counter3 ISR not yet running; no ATOMIC stuff needed
        nextPendingSteps[i] = 0;
        nextDsInterval[i] = 0;
        nextShortening[i] = false;
    }
    pinMode(calClkPin, INPUT);
    pinMode(calDataPin, INPUT);
    pinMode(diverClkPin, OUTPUT);
    pinMode(diverDataPin, OUTPUT);
    calibrated = false;
    doCal = false;
    digitalWrite(enablePin, LOW);   // Enable (active LOW) motor drivers
    enabled = true;

    batchSteps = FP_BATCH_STEPS;
    t = 1.0;
    newMove = false;
    stopping = false;
    maxSpeed = FP_MAX_SPEED;
    hHeading = 0;
    hChange = fp_straight;
    diverHLight = fp_straight;      // Temproary
    diverhHeading = hHeading;
    vHeading = FP_LEVEL;
    vChange = fp_level;
    diverVLight = fp_level;         // Temporary
    turnMicros = 0;
    nextReady = false;
    #ifdef FP_DEBUG_ISR
    pinMode(FP_ISR_T_PIN, OUTPUT);
    pinMode(FP_ISR_I_PIN, OUTPUT);
    pinMode(FP_ISR_W_PIN, OUTPUT);
    digitalWrite(FP_ISR_T_PIN, LOW);
    digitalWrite(FP_ISR_I_PIN, LOW);
    digitalWrite(FP_ISR_W_PIN, LOW);
    #endif
    #ifdef FP_DEBUG_DIVER
    pinMode(FP_LED_PIN, OUTPUT);
    digitalWrite(FP_LED_PIN, LOW);
    #endif
    #ifdef FP_DEBUG_GEO
    fp_CableBundle longCable0 = p3DToCb(targetsMax);
    maxCableLength = longCable0.c[0];                   // No cable should be longer than this
    #endif

    // Set up our Timer/Counter3 ISR (This is the ISR that moves the diver around the exhibit)
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TIMSK3 = 0x00;                                  // Reset Timer 3
        TCCR3A = 0x00;
        TCCR3B = 0x00;
        TCCR3C = 0x00;
        OCR3A = 0xFFFF;                                 // Set count of (ticks - 1) between interrupts, i.e., 62500 * 4μs == 0.25s
        TIMSK3 = _BV(OCIE3A);                           // Enable OCR3A compare interrupts
        TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);    // WGM3[3:0] = 0x4 i.e., Set Clear Timer on Compare Match (CTC) Mode with OCR3A as TOP
                                                        // CS3[2:0] = 0x3 i.e., Set Timer Clock Source to Clock/64 i.e., 16MHz/64 = 250kHz
                                                        // COM3x[1:0] = 0 i.e., Set normal port operation, OC3A/OC3B/OC3C disconnected
                                                        // ICNC3 = 0      i.e., Disable Input Capture Noise Canceler
                                                        // ICES3 = 0      i.e., Disable Input Capture Edge Select
    }

    // Attach the calReader ISR (This is the one that keep track of whether the cables are longer or shorter than the calibration point.)
    attachInterrupt(digitalPinToInterrupt(calClkPin), calClkISR, RISING);

    // Set up our Timer/Counter4 ISR (This is the one that causes the diver to point in the direction of motion and lights her LEDs)
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TIMSK4 = 0x00;                                  // Reset Timer/Counter4
        TCCR4A = 0x00;
        TCCR4B = 0x00;
        TCCR4C = 0x00;
        OCR4A = FP_DIVER_TICKS_PER_INT;                 // Set count of (ticks - 1) between interrupts
        TIMSK4 = _BV(OCIE4A);                           // Enable OCR4A compare interrupts
        TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40);    // WGM4[3:0] = 0x4 i.e., Set Clear Timer on Compare Match (CTC) Mode with OCR4A as TOP
                                                        // CS4[2:0] = 0x3 i.e., Set Timer Clock Source to Clock/64 i.e., 16MHz/64 = 250kHz
                                                        // COM4x[1:0] = 0 i.e., Set normal port operation, OC4A/OC4B/OC4C disconnected
                                                        // ICNC4 = 0      i.e., Disable Input Capture Noise Canceler
                                                        // ICES4 = 0      i.e., Disable Input Capture Edge Select
  }
  packets[0] = FP_DIVER_IDLE_PACKET;                    // Send the diver an idle packet to establish sync
  packets[1] = FP_DIVER_TT_HEAD + 0;                    // And then a packet head her in direction 0 to ensure she's heading where we assume she is
  packetCount = 2;
}

bool FlyingPlatform::run() {
    #ifdef FP_DEBUG_RU
    static byte newMoveCount = 0;
    #endif
    #ifdef FP_DEBUG_ISR
        if (captured) {
            unsigned long ttm[4], tcm;
            byte tnf;
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                tnf = tNFinished;
                tcm = tCurMicros;
                for (int8_t i = 0; i < 4; i++) {
                    ttm[i] = tTargetMicros[i];
                }
            }
            Serial.print(F("ISR status dump. curMicros: "));
            Serial.print(tcm);
            Serial.print(F(", nFinished: "));
            Serial.print(tnf);
            Serial.print(F(", targetMicros[]: "));
            for (int8_t i = 0; i < 4; i++) {
                Serial.print(ttm[i]);
                Serial.print(i == 3 ? F("\n") : F(", "));
            }
            captured = false;
        }
        if (late) {
            Serial.print(F("Trap!: Late in dispatching a step."));
            late = false;
        }
    #endif
    
    if (!calibrated || !enabled) {
        return false;
    }
    unsigned long curMicros = micros();

    // Deal with turning
    if (curMicros - turnMicros >= FP_TURN_INTERVAL) {
        bool changed = false;
        if (hChange == fp_left) {
            hHeading = (hHeading + 1) % FP_N_HHEADINGS;
            changed = true;
        } else if (hChange == fp_right) {
            hHeading = (hHeading + FP_N_HHEADINGS - 1) % FP_N_HHEADINGS;
            changed = true;
        }
        switch (vChange) {
            case fp_falling:
                if (vHeading > 0) {
                    vHeading--;
                    changed = true;
                }
                break;
            case fp_level:
                if (vHeading > FP_LEVEL) {
                    vHeading--;
                    changed = true;
                } else if (vHeading < FP_LEVEL) {
                    vHeading++;
                    changed = true;
                }
                break;
            case fp_rising:
                if (vHeading < FP_N_VHEADINGS - 1) {
                    vHeading++;
                    changed = true;
                }
                break;
        }
        // If there's a new heading and we're going and we're not trying to stop
        if (changed && timerISRHasWork && !stopping) {
            moveTo(newTarget());
        }
        turnMicros = curMicros;
    }

    // Orient the diver so she always points in the direction in which she's swimming
    if (packetCount == 0) {
        uint8_t packetIx = 0;
        if (diverhHeading != hHeading) {
            packets[packetIx++] = FP_DIVER_TT_HEAD + hHeading;
            diverhHeading = hHeading;
        }
        // Temporary: Front light: red = turning left, green = right off = straight
        if (diverHLight != hChange) {
            packets[packetIx++] = FP_DIVER_TT_PIX0 + (hChange == fp_hTurns::fp_left ? 0b110000 : hChange == fp_hTurns::fp_right ? 0b001100 : 0);
            diverHLight = hChange;
        }
        // Temporary: Back light: blue = descending, yellow = rising, off = level
        if (diverVLight != vChange) {
            packets[packetIx++] = FP_DIVER_TT_PIX1 + (vChange == fp_vTurns::fp_falling ? 0b000011 : vChange == fp_vTurns::fp_rising ? 0b101000 : 0);
            diverVLight = vChange;
        }            
        packetCount = packetIx;
    }

    // If we're stopping, no further work until the queued ISR work finishes
    if (stopping && timerISRHasWork) {
        return true;
    }


    // If we're stopping (and the queued ISR work is finished), we're stopped.
    if (stopping) {
        #ifdef FP_DEBUG_GEO
        fp_CableBundle cs;
        for (byte i = 0; i < 4; i++) {      // ISR is idle; no need for atomic access
            cs.c[i] = (float)cableSteps[i];
        }
        fp_Point3D here = cbToP3D(cs);
        if (here.x < marginsMin.x || here.y < marginsMin.y || here.z < marginsMin.z ||
          here.x > marginsMax.x || here.y > marginsMax.y || here.z > marginsMax.z) {
            Serial.print(F("run() Stopped out of bounds: (<"));
            Serial.print(marginsMin.x);
            Serial.print(F("> "));
            Serial.print(here.x);
            Serial.print(F(" <"));
            Serial.print(marginsMax.x);
            Serial.print(F(">, <"));
            Serial.print(marginsMin.y);
            Serial.print(F("> "));
            Serial.print(here.y);
            Serial.print(F(" <"));
            Serial.print(marginsMax.y);
            Serial.print(F(">, <"));
            Serial.print(marginsMin.z);
            Serial.print(F("> "));
            Serial.print(here.z);
            Serial.print(F(" <"));
            Serial.print(marginsMax.z);
            Serial.print(F(">), cables: ("));
            for (byte i = 0; i < 4; i++) {
                Serial.print((long)(0.5 + cs.c[i]));
                Serial.print( i == 3 ? F(").\n") : F(", "));
            }
        }
        #endif
        stopping = false;
        t = 1.0;            // Move is done
        newMove = false;    // And we're not starting another
        return false;
    }

    // If all done with move and nothing new has been scheduled, no use doing more work.
    if (t == 1.0 && !newMove) {
        return false;
    }

    // If we need to calculate the next batch of steps
    if (!nextReady) {
        // If we're to go off in a new direction, set up for first batch of the move from wherever we are to target
        if (newMove) {            
            newMove = false;
            source = where();
            #ifdef FP_DEBUG_GEO
            if (source.x < marginsMin.x || source.y < marginsMin.y || source.z < marginsMin.z ||
                source.x > marginsMax.x || source.y > marginsMax.y || source.z > marginsMax.z) {
                Serial.print(F("run() starting new move out of bounds.\n  source: ("));
                Serial.print(source.x);
                Serial.print(F(", "));
                Serial.print(source.y);
                Serial.print(F(", "));
                Serial.print(source.z);
                Serial.print(F("), hSlope: "));
                Serial.print(hSlope[hHeading]);
                Serial.print(F(", vSlope: "));
                Serial.println(vSlope[vHeading]);
            }
            // Super temporary test
            if (source.z != target.z) {
                Serial.print(F("Unexpected change in z while starting new move. source: ("));
                Serial.print(source.x);
                Serial.print(F(", "));
                Serial.print(source.y);
                Serial.print(F(", "));
                Serial.print(source.z);
                Serial.print(F("), target: ("));
                Serial.print(target.x);
                Serial.print(F(", "));
                Serial.print(target.y);
                Serial.print(F(", "));
                Serial.print(target.z);
                Serial.println(F(")."));
            }
            #endif
            float moveLength = sqrt(
                (target.x - source.x) * (target.x - source.x) + 
                (target.y - source.y) * (target.y - source.y) + 
                (target.z - source.z) * (target.z - source.z));
            #ifdef FP_DEBUG_RU
            Serial.print(F("run() new move. source: ("));
            Serial.print(source.x);
            Serial.print(F(", "));
            Serial.print(source.y);
            Serial.print(F(", "));
            Serial.print(source.z);
            Serial.print(F("), target: ("));
            Serial.print(target.x);
            Serial.print(F(", "));
            Serial.print(target.y);
            Serial.print(F(", "));
            Serial.print(target.z);
            Serial.print(") moveLength: ");
            Serial.println(moveLength);
            newMoveCount++;
            #endif

            // Start at t = 0 and with the dt needed to have batches consisting of batchSteps steps
            t = 0.0;
            dt = 1.0 / (moveLength / batchSteps);   // A batch every batchSteps
            nextCableSteps = p3DToCb(source);       //   starting at source
            nextPoint = source;
        }

        // Calculate the next batch of steps and mark them as available to the ISR. For the last batch use
        // the cable lengths at target since we could otherwise end at a slightly different place due to
        // errors from limited precision.
        float nextT = min(1.0, t + dt);                         // t for the next batch of steps
        fp_CableBundle startCableSteps = nextCableSteps;        // Cable lengths and 
        if (nextT < 0.999) {                                    // If it's not the last batch in the move
            nextPoint.x = source.x + nextT * (target.x - source.x);
            nextPoint.y = source.y + nextT * (target.y - source.y);
            nextPoint.z = source.z + nextT * (target.z - source.z);
            nextCableSteps = p3DToCb(nextPoint);
        } else {                                                // Else it's the last batch; use cable lengths at target
            nextCableSteps = p3DToCb(target);
            nextPoint = cbToP3D(nextCableSteps);
        }
        t = nextT;
        #ifdef FP_DEBUG_GEO
        if (nextPoint.x < marginsMin.x || nextPoint.y < marginsMin.y || nextPoint.z < marginsMin.z ||
            nextPoint.x > marginsMax.x || nextPoint.y > marginsMax.y || nextPoint.z > marginsMax.z) {
            Serial.print(F("run() Heading out of bounds. t: "));
            Serial.print(t);
            Serial.print(F(", dt: "));
            Serial.print(dt);
            Serial.print(F(", nextPoint: ("));
            Serial.print(nextPoint.x);
            Serial.print(F(", "));
            Serial.print(nextPoint.y);
            Serial.print(F(", "));
            Serial.print(nextPoint.z);
            Serial.println(F(")"));
        }
        #endif
        long mostSteps = 0;
        for (byte i = 0; i < 4; i++) {
            nextPendingSteps[i] = nextCableSteps.c[i] - startCableSteps.c[i];
            if (nextPendingSteps[i] < 0) {
                nextPendingSteps[i] = -nextPendingSteps[i];
                nextShortening[i] = true;
            } else {
                nextShortening[i] = false;
            }
            if (nextPendingSteps[i] > mostSteps) {
                mostSteps = nextPendingSteps[i];
            }
        }
        long batchInterval = 1000000.0 * mostSteps / maxSpeed;  // How long (μs) it will take to do batch
        for (byte i = 0; i < 4; i++) {              // Set interval between steps
            nextDsInterval[i] = nextPendingSteps[i] == 0 ? batchInterval : batchInterval / nextPendingSteps[i];
        }
        #ifdef FP_DEBUG_RU
        Serial.print(F("run() New batch. t: "));
        Serial.print(t, 5);
        Serial.print(F(", pendingSteps: ("));
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextShortening[i] ? -nextPendingSteps[i] : nextPendingSteps[i]);
            Serial.print(i == 3 ? F("), dsInterval: (") : F(", "));
        }
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextDsInterval[i]);
            Serial.print(i == 3 ? F(").\n") : F(", "));
        }
        #endif
        #ifdef FP_DEBUG_RU
        if (t >= 1.0) {
            Serial.print(F("Last batch. moveCount: "));
            Serial.print(newMoveCount);
            Serial.print(F(", pendingSteps: ("));
            for (byte i = 0; i < 4; i++) {
                Serial.print(nextShortening[i] ? -nextPendingSteps[i] : nextPendingSteps[i]);
                Serial.print(i == 3 ? F("), dsInterval: (") : F(", "));
            }
            for (byte i = 0; i < 4; i++) {
                Serial.print(nextDsInterval[i]);
                Serial.print(i == 3 ? F(").\n") : F(", "));
            }
        }
        #endif
        nextReady = true;
        return true;
    }
    return false;
}

void FlyingPlatform::setMaxSpeed(float speed) {
    maxSpeed = speed;
}

fp_return_code FlyingPlatform::setSafetyMargins(
    long leftMargin, long rightMargin, 
    long frontMargin, long backMargin, 
    long bottomMargin, long topMargin) {
    if (leftMargin < 0 || rightMargin > space.x ||
        frontMargin < 0 || backMargin > space.y ||
        bottomMargin < 0 || topMargin > space.z) {
        return fp_oob;
    }
    marginsMin = fp_Point3D {leftMargin, frontMargin, bottomMargin};
    marginsMax = fp_Point3D {space.x - rightMargin, space.y - backMargin, space.z - topMargin};
    targetsMin = fp_Point3D {marginsMin.x + FP_TARGETS_BUFFER, marginsMin.y + FP_TARGETS_BUFFER, marginsMin.z + FP_TARGETS_BUFFER};
    targetsMax = fp_Point3D {marginsMax.x - FP_TARGETS_BUFFER, marginsMax.y - FP_TARGETS_BUFFER, marginsMax.z - FP_TARGETS_BUFFER};
    return fp_ok;
}

void FlyingPlatform::setBatchSize(long steps) {
    batchSteps = steps;
}

fp_return_code FlyingPlatform::setCurrentPosition (fp_Point3D tgt) {
    if (tgt.x < marginsMin.x || tgt.y < marginsMin.y || tgt.z < marginsMin.z ||
        tgt.x > marginsMax.x || tgt.y > marginsMax.y || tgt.z > marginsMax.z) {
            #ifdef FP_DEBUG_GEO
            Serial.print(F("flyingPlatform::setCurrentPosition out of bounds: ("));
            Serial.print(marginsMax.x);
            Serial.print(F(", "));
            Serial.print(marginsMax.y);
            Serial.print(F(", "));
            Serial.print(marginsMax.z);
            Serial.print(F(")\n"));
            #endif
            return fp_oob;  // Can't do it: Target is out of bounds
        }
    if (isRunning()) {
        return fp_mov;      // Can't do it: A move is underway
    }
    fp_CableBundle cs = p3DToCb(tgt);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {          // Techincally this shouldn't be needed since we're not running, but paranoia
        cableSteps[0] = (long)(0.5 + cs.c[0]);
        cableSteps[1] = (long)(0.5 + cs.c[1]);
        cableSteps[2] = (long)(0.5 + cs.c[2]);
        cableSteps[3] = (long)(0.5 + cs.c[3]);
    }
    source = target = tgt;
    hHeading = 0;
    vHeading = FP_LEVEL;

    calibrated = true;
    return fp_ok;;
}

fp_return_code FlyingPlatform::calibrate(fp_Point3D tgt) {
    if (!enabled) {
        return fp_dis;
    }
    fp_return_code rc = setCurrentPosition(tgt);
    if (rc != fp_ok) {
        return rc;
    }
    hHeading = 0;
    vHeading = FP_LEVEL;
    doCal = true;
    return fp_ok;
}

fp_return_code FlyingPlatform::enableOutputs() {
    digitalWrite(enablePin, LOW);   // Active LOW
    enabled = true;
    return fp_ok;
}

fp_return_code FlyingPlatform::disableOutputs() {
    if (isRunning()) {
        return fp_mov;  // Can't do it: A move is underway.
    }
    digitalWrite(enablePin, HIGH);  // Active LOW
    enabled = false;
    return fp_ok;
}

fp_return_code FlyingPlatform::moveTo(fp_Point3D tgt) {
    if (tgt.x < marginsMin.x || tgt.y < marginsMin.y || tgt.z < marginsMin.z ||
        tgt.x > marginsMax.x || tgt.y > marginsMax.y || tgt.z > marginsMax.z) {
        #ifdef FP_DEBUG_GEO
        Serial.print(F("moveTo() Out of bounds. tgt: ("));
        Serial.print(tgt.x);
        Serial.print(F(", "));
        Serial.print(tgt.y);
        Serial.print(F(", "));
        Serial.print(tgt.z);
        Serial.print(F("), Min: ("));
        Serial.print(marginsMin.x);
        Serial.print(F(", "));
        Serial.print(marginsMin.y);
        Serial.print(F(", "));
        Serial.print(marginsMin.z);
        Serial.print(F("), Max: ("));
        Serial.print(marginsMax.x);
        Serial.print(F(", "));
        Serial.print(marginsMax.y);
        Serial.print(F(", "));
        Serial.print(marginsMax.z);
        Serial.println(F(")."));
        #endif
        return fp_oob;      // Can't do it: Target is out of bounds
    }
    if (!calibrated) {
        return fp_ncp;          // Can't do it: No current position set
    }
    // Okay, go for it: Set things up to move to tgt
    target = tgt;
    newMove = true;                         // Indicate new move

    #ifdef FP_DEBUG_MT
    Serial.print(F("moveTo() from: ("));
    Serial.print(source.x);
    Serial.print(F(", "));
    Serial.print(source.y);
    Serial.print(F(", "));
    Serial.print(source.z);
    Serial.print(F("), to: ("));
    Serial.print(target.x);
    Serial.print(F(", "));
    Serial.print(target.y);
    Serial.print(F(", "));
    Serial.print(target.z);
    Serial.println(F(")."));
    #endif

    return fp_ok;
}

fp_return_code FlyingPlatform::moveBy(fp_Point3D delta) {
    if (!calibrated) {
        return fp_ncp;
    }
    fp_Point3D current = where();

    #ifdef FP_DEBUG_MB
    Serial.print(F("moveBy() from: ("));
    Serial.print(current.x);
    Serial.print(F(", "));
    Serial.print(current.y);
    Serial.print(F(", "));
    Serial.print(current.z);
    Serial.print(F("), by: ("));
    Serial.print(delta.x);
    Serial.print(F(", "));
    Serial.print(delta.y);
    Serial.print(F(", "));
    Serial.print(delta.z);
    Serial.println(F(")."));
    #endif

    return moveTo(fp_Point3D {current.x + delta.x, current.y + delta.y, current.z + delta.z});
}

fp_return_code FlyingPlatform::go() {
    if (!calibrated) {
        return fp_ncp;
    }
    return moveTo(newTarget());
}

fp_return_code FlyingPlatform::turn(fp_hTurns dir) {
    hChange = dir;
    if (isRunning() && !stopping) {
        return go();
    }
    return fp_ok;
}

fp_return_code FlyingPlatform::turn(fp_vTurns dir) {
    vChange = dir;
    if (isRunning() && !stopping) {
        return go();
    }
    return fp_ok;
}

void FlyingPlatform::stop() {
    stopping = true;
    if (doCal) {
        doCal = false;
        calibrated = false;
    }
}

void FlyingPlatform::status() {
    Serial.print(F(
        "Status\n"
        " Enabled: "));
    Serial.print(enabled);
    Serial.print(F(", Calibrated: "));
    Serial.print(calibrated);
    Serial.print(F(",  hasWork: "));
    Serial.print(timerISRHasWork);
    Serial.print(F(", nextReady: "));
    Serial.println(nextReady);
    if (!captured) {
        dump = true;
    }
}

fp_Point3D FlyingPlatform::where() {
    if (!calibrated) {
        return fp_Point3D {0, 0, 0};
    }
    fp_CableBundle cs;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { // Needed because the ISR might be actively updating cableSteps.
        for (byte i = 0; i < 4; i++) {
            cs.c[i] = (float)cableSteps[i];
        }
    }
    fp_Point3D answer = cbToP3D(cs);

    #ifdef FP_DEBUG_GEO_VERBOSE
    if (answer.x < marginsMin.x || answer.y < marginsMin.y || answer.z < marginsMin.z ||
        answer.x > marginsMax.x || answer.y > marginsMax.y || answer.z > marginsMax.z) {
        Serial.print(F("where() called while out of bounds. x: "));
        Serial.print(answer.x);
        Serial.print(F(", y: "));
        Serial.print(answer.y);
        Serial.print(F(", z: "));
        Serial.print(answer.z);
        Serial.print(F(", hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", cableSteps: ("));
        for (byte i = 0; i < 4; i++) {
            Serial.print((long)(0.5 + cs.c[i]));
            Serial.print(i < 3 ? F(", ") : F(")\n"));
        }
    }
    #endif

    return answer;
}

bool FlyingPlatform::isRunning() {
    return timerISRHasWork;
}

bool FlyingPlatform::isEnabled() {
    return enabled;
}

bool FlyingPlatform::isCalibrated() {
    return calibrated;
}

fp_Point3D FlyingPlatform::newTarget() {
    #ifdef FP_DEBUG_GEO
    bool isStopped = true;
    #endif
    fp_Point3D here;
    if (timerISRHasWork) {
        here = cbToP3D(nextCableSteps); // If we're running, we'll start with next batch
        #ifdef FP_DEBUG_GEO
        isStopped = false;
        #endif
    } else {
        here = where();                 // If we're stopped, we'll start where we are
    }
    #ifdef FP_DEBUG_GEO
    if (here.x < marginsMin.x || here.y < marginsMin.y || here.z < marginsMin.z ||
        here.x > marginsMax.x || here.y > marginsMax.y || here.z > marginsMax.z) {
        Serial.print(F("newTarget() starting out of bounds while "));
        Serial.print(isStopped ? F("stopped") : F("moving"));
        Serial.print(F(".\n  here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", vSlope: "));
        Serial.println(vSlope[vHeading]);
    }
    #endif
    // Figure y intercept
    float by = here.y - (hSlope[hHeading] * here.x);

    // Assume we should aim to hit the left or right of our allowed space (i.e., max or min x)
    float x = fp_xIsRising(hHeading) ? targetsMax.x : targetsMin.x;
    float y = hSlope[hHeading] * x + by;

    // If that puts x and y at (almost exactly) the same place as here, we're already up 
    // against the target boundary
    if (abs(here.x - x) < 1.0 && abs(here.y - y) < 1.0) {
        #ifdef FP_DEBUG_NT
        Serial.print(F("newTarget() At boundary from x. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.println(F(")."));
        #endif
        return here;
    }

    // Calculate what z is based on vSlope x, here.x, y, here.y and here.z.
    float z = here.z + vSlope[vHeading] * sqrt((x - here.x) * (x - here.x) + (y - here.y) * (y - here.y));
    // Save x, y, and z so we'll have them at this stage in case assertion fails
    float x0 = x;
    float y0 = y;
    float z0 = z;

    // If aiming at the left or right would result in going beyond the front, 
    // back, top or bottom and we're not going parallel to the front/back, 
    // assume we should aim at the front or back. Which it is depends on whether 
    // x is rising and on hSlope. E.g., when x is rising and hSlope is positive, 
    // y is also rising so it's the back.
    if (y < marginsMin.y || y > marginsMax.y || z < marginsMin.z || z > marginsMax.z) {
        y = fp_xIsRising(hHeading) == (hSlope[hHeading] > 0) ? targetsMax.y : targetsMin.y;
        x = (y - by) / hSlope[hHeading];
    }

    // If that puts x and y at (almost exactly) the same place as here, we're already up 
    // against the target boundary
    if (abs(here.x - x) < 1.0 && abs(here.y - y) < 1.0) {
        #ifdef FP_DEBUG_NT
        Serial.print(F("newTarget() At boundary from y. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.println(F(")."));
        #endif
        return here;
    }

    // Save x, y and z at this stage so we'll have them in case assertion fails
    float x1 = x;
    float y1 = y;
    float z1 = z;

    // If all of the preceeding would result in going beyond the top or bottom, 
    // aim for the top or bottom.
    if (z < marginsMin.z || z > marginsMax.z) {
        z = z < targetsMin.z ? targetsMin.z : targetsMax.z;

        // The distance in the x-y plane from (here.x, here.y, here.z) to the point in the x-y 
        // plane where we would reach z, given vSlope is 
        //   d = (z - here.z) / vSlope.
        // The square of the same distance in terms of x, here.x, y and here.y is 
        //   d**2 = (x - here.x)**2 + (y - here.y)**2. 
        // We also know 
        //   (y - here.y) / (x - here.x) = hSlope.
        // With these and some algebra we can solve for x in terms of z: 
        //   x = here.x + (z - here.z) / (sqrt(hSlope**2 + 1) * (+-)vSlope)
        // the vSlope term is + if x is rising, - if it is falling
        if (fp_xIsRising(hHeading)){
            x = here.x + (z - here.z) / (sqrt(1 + hSlope[hHeading] * hSlope[hHeading]) * vSlope[vHeading]);
        } else {
            x = here.x - (z - here.z) / (sqrt(1 + hSlope[hHeading] * hSlope[hHeading]) * vSlope[vHeading]);
        }
        // Since we know hSlope, x and by, finding y is trivial
        y =  hSlope[hHeading] * x + by;
    }

    fp_Point3D answer = {(long)(x + 0.5), (long)(y + 0.5), (long)(z + 0.5)};

    // Assert: The answer should be inside of the margins
    if (answer.x < marginsMin.x || answer.y < marginsMin.y || answer.z < marginsMin.z ||
        answer.x > marginsMax.x || answer.y > marginsMax.y || answer.z > marginsMax.z) {
        Serial.print(F("Assertion failed in newTarget(): Target is outside the margins.\n  here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), target: ("));
        Serial.print(answer.x);
        Serial.print(F(", "));
        Serial.print(answer.y);
        Serial.print(F(", "));
        Serial.print(answer.z);
        Serial.print(F("), margins: (<"));
        Serial.print(marginsMin.x);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.x);
        Serial.print(F(">, <"));
        Serial.print(marginsMin.y);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.y);
        Serial.print(F(">, <"));
        Serial.print(marginsMin.z);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.z);
        Serial.print(F(">)\n  x0: "));
        Serial.print(x0);
        Serial.print(F(", y0: "));
        Serial.print(y0);
        Serial.print(F(", z0: "));
        Serial.print(z0);
        Serial.print(F(", x1: "));
        Serial.print(x1);
        Serial.print(F(", y1: "));
        Serial.print(y1);
        Serial.print(F(", z1: "));
        Serial.print(z1);
        Serial.print(F(", x is "));
        Serial.print(fp_xIsRising(hHeading) ? F("rising") : F("falling"));
        Serial.print(F(", hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", vSlope: "));
        Serial.print(vSlope[vHeading]);
        Serial.print(F(", by: "));
        Serial.println(by);
        while (true) {
            // Spin
        }
    }
    return answer;
}

fp_Point3D FlyingPlatform::cbToP3D(fp_CableBundle bundle) {
    fp_Point3D point;
    // From Weisstein, Eric W. "Sphere-Sphere Intersection." From 
    // MathWorld--A Wolfram Web Resource. 
    // https://mathworld.wolfram.com/Sphere-SphereIntersection.html, the x
    // coordinate of the circle (which necessarily lies parallel to the y-z 
    // plane) that is the intersection of two spheres, one which is centered 
    // at the origin and has radius R and the other of which has radius r and 
    // is centered on the x axis at distance d from the origin is given by
    //
    //      x = (d^2 - r^2 + R^2) / (2 * d) 
    //
    // or, rearranging slightly
    //
    //      x = d / 2 + (R^2 - r^2) / (2 * d)
    //
    // So we have

    point.x = 0.5 + space.x / 2.0 + (bundle.c[0] * bundle.c[0] - bundle.c[1] * bundle.c[1]) / (space.x * 2.0);

    // Similarly for y

    point.y = 0.5 + space.y / 2.0 + (bundle.c[0] * bundle.c[0] - bundle.c[2] * bundle.c[2]) / (space.y * 2.0);

    // From the formula for a sphere and knowing x and y we can use any of the 
    // cableSteps[0 .. 3] to calculate z. We'll use the largest for the best 
    // resolution.
    //
    // x^2 + y^2 + z^2 = r^2        solve for z^2
    // z^2 = r^2 - x^2 - y^2        sqrt both sides
    // z = sqrt(r^2 - x^2 - y^2)
    //
    // Remembering that here, the z is measured from the top of the flying space 
    // and we need it from the origin.
    
    float dx = 0.0, dy = 0.0, r = bundle.c[0]; // Assume cable 0 is longest
    #ifdef FP_DEBUG_GEO
    int8_t c = 0;
    #endif

    for (int8_t n = 1; n < 4; n++) {
        if (bundle.c[n] > r) {          // If cable[n] is longer, use it
            r = bundle.c[n];
            dx = n % 2 ? space.x : 0.0; // Cables 1 and 3 have x coordinate == space.x; for cables 0 and 2, x == 0.0
            dy = n > 1 ? space.y : 0.0; // Cables 2 and 3 have y coordinate == space.y; for cables 0 and 1, y == 0.0
            #ifdef FP_DEBUG_GEO
            c = n;
            #endif
        }
    }

    float innerds = r * r - (point.x - dx) * (point.x - dx) - (point.y - dy) * (point.y - dy);
    if (innerds < 0) {
        #ifdef FP_DEBUG_GEO
        Serial.print(F("Flakey cbToP3D result. Setting innerds to 0. cb ("));
        Serial.print(bundle.c[0]);
        Serial.print(F(" "));
        Serial.print(bundle.c[1]);
        Serial.print(F(" "));
        Serial.print(bundle.c[2]);
        Serial.print(F(" "));
        Serial.print(bundle.c[3]);
        Serial.print(F("), point.x: "));
        Serial.print(point.x);
        Serial.print(F(", point.y: "));
        Serial.print(point.y);
        Serial.print(F(", innerds: "));
        Serial.print(innerds);
        Serial.print(F(", using cable: "));
        Serial.print(c);
        Serial.print(F(", r: "));
        Serial.print(r);
        Serial.print(F(", dx: "));
        Serial.print(dx);
        Serial.print(F(", dy: "));
        Serial.println(dy);
        #endif
        innerds = 0.0;
    }
    point.z = 0.5 + space.z - sqrt(innerds);
    return point;
}

/**
 * 
 * Convert an fp_Point3D to the equivalent fp_CableBundle
 * 
 **/

fp_CableBundle FlyingPlatform::p3DToCb(fp_Point3D point) {
    fp_CableBundle bundle;
    bundle.c[0] = sqrt(point.x * point.x + point.y * point.y + (space.z - point.z) * (space.z - point.z));
    bundle.c[1] = sqrt((space.x - point.x) * (space.x - point.x) + point.y * point.y + (space.z - point.z) * (space.z - point.z));
    bundle.c[2] = sqrt(point.x * point.x + (space.y - point.y) * (space.y - point.y) + (space.z - point.z) * (space.z - point.z));
    bundle.c[3] = sqrt((space.x - point.x) * (space.x - point.x) + (space.y - point.y) * (space.y - point.y) + (space.z - point.z) * (space.z - point.z));
    return bundle;
}
