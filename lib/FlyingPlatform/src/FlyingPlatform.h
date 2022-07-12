/****
 * 
 * This file is a portion of the package FlyingPlatform, a library that 
 * provides an Arduino sketch running on an Arduino Mega 2560 Rev3 with the 
 * ability to control four stepper motors (through stepper motor drivers) to 
 * operate a "flying camera" setup (technically, a parallel cable-driven 
 * robot) in which a platform is connected by cables to four stepper-motor-
 * controlled winches. Each winch consists of a closed-loop toothed belt that 
 * runs from the stepper at the loop's lower extreme to the cable hoist point 
 * at its upper end. The cable hoist points are located at the top corners of 
 * a rectangular prism known as "the flying space." The cable controlled by a 
 * given winch is attached to the winch's belt at a single point and so gets 
 * shortened or lengthened as the stepper moves the belt. By judiciously 
 * shortening and lengthening the cables, the platform can be moved around in 
 * three dimensions in the flying space.
 * 
 * When viewed from the front, the origin of the flying space's coordinate 
 * system is at its lower, front, left corner. Its faces are parallel to the 
 * axes. The +x direction is to the to the right, +y is toward the back, and 
 * +z is up. The measurements are all in steps. A step is the amount of cable 
 * a winch reels in or lets out when its stepper motor is moved by one step.
 * 
 * The flying space is buffered by "safety margins." That is, the platform is
 * not allowed to be driven right to the edge of the flying space because it 
 * could crash into the mechanism or some other obstacle. Instead it may only 
 * be driven in the volume set by setSafetyMargins(). And, because of the 
 * non-linearities involved in translating straight line paths in the flying 
 * space into cable pulls, we only target going places in a slightly smaller 
 * space set internally and defined in targetsMin and targetsMax.
 * 
 * Initialization is in two parts. At the time the FlyingPlatform object is 
 * constructed, the basic physical parameters are set -- I/O pins and flying 
 * space size. Then, when the sketch's setup() is run, the begin() member 
 * function must be invoked to do the rest of the initialization. Once 
 * initialization is complete, the basic operation is in five parts. 
 * 
 * First, there is an ISR that services interrupts from the calReader Arduino. 
 * The calReader sends data about the passage of magnets attached to each 
 * winch's belt. When a magnet passes over its associated a Hall-effect 
 * sensor, the length of that winch's cable is at its calibration point. When 
 * all four cables are at their calibration point, the system is at its home 
 * position. 
 * 
 * The calReader Arduino sends the Hall-effect sensors' states as a continuous 
 * stream of bits. The ISR reads the data from calReader and keeps track of 
 * whether each cable is longer than, shorter than or just at its calibration 
 * point. The  ISR is also used to drive the cables to their calibration points 
 * when a calbration is done.
 * 
 * Second, there is an ISR that's driven by hardware Timer/Counter4. The ISR 
 * is used to send data to the Arduino attached to the platform. This Arduino 
 * controls the yaw position of the diver so that she points in the direction 
 * in which she moves. It also controls the two RGB LEDs attached to her out-
 * placement device.
 * 
 * Data to be sent to the diver-control Arduino is in the form of 8-bit 
 * packets. A packet can contain the heading (yaw angle) the diver is to 
 * assume, a color of one or the other of the diver's LEDs, or command data. 
 * At present, there are two commands, Idle (a NOP that can safely be sent at 
 * any time) and Calibrate, which instructs the diver-control Arduino to use 
 * hardware calibration to make the diver's yaw correspond to point in the 
 * direction of hHeading == 0. run() communicates with the ISR through the 
 * shared variables packets[] and packetCount.
 * 
 * Third, there is an ISR that's driven by the hardware Timer/Counter3. It 
 * executes whenever the value in Timer 3 equals the value in register OCR3A. 
 * The ISR adjusts the value in OCR3A to cause interrupts to happen as needed 
 * to drive the stepper motors. Note that because we comandeer Timer 3, it 
 * can't be used for anything else. In particular, this is the timer that the 
 * Arduino tone() function uses, so don't use tone() in sketches using 
 * FlyingPlatform. 
 * 
 * Anyway, the Timer/Counter3 ISR is what actually sends direction and step 
 * signals to the winch stepper motor drivers. It gets its marching orders 
 * from the run() member function. 
 * 
 * Fourth, there is the run() member function. A call to run() should be 
 * placed in the loop() function of the sketch. It needs to be invoked often. 
 * Run does the calculations that turn moves in 3-space into cable length-
 * change instructions. It places these in nextPendingSteps, nextShortening 
 * and nextDsInterval. nextPendingSteps is a count of the steps to be taken 
 * by each of the motors, and nextShortening tells whether each cable is to 
 * be lengthened or shortened. And nextDsInterval says how many μs there 
 * should be between steps. The timer ISR and run() coordinate their use of 
 * these buffers through the semaphore nextReady. run() also communicates 
 * with the Timer/Counter4 ISR to instruct the diver-control Arduino.
 * 
 * The fifth component of the basic operation are the other member functions. 
 * The sketch invokes these as needed to request moves in 3-space, to set 
 * operational parameters and to get information about the state of the 
 * FlyingPlatform.
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
#ifndef FLYING_PLATFORM
    #define FLYING_PLATFORM

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

    /**
     * 
     * Uncomment to turn on debugs
     * 
     **/
    #define FP_DEBUG_ISR        // Timer/Counter3 ISR (A bunch of it is in run())
    #define FP_DEBUG_DIVER      // Timep/Counter4 ISR (Diver pointing & lights)
    //#define FP_DEBUG_CAL_ISR    // calReader ISR
    #define FP_DEBUG_GEO        // Wacky geometry checking
    //#define FP_DEBUG_RU         // run()
    //#define FP_DEBUG_MT         // moveTo()
    //#define FP_DEBUG_MB         // moveBy()
    //#define FP_DEBUG_NT         // newTarget()

    /**
     * 
     * Some compile-time constants
     * 
     **/
    #define FP_MAX_SPEED        (800)       // Default maximum speed (in steps/sec) of four the main steppers
    #define FP_BATCH_STEPS      (128)       // Default size of batches (in steps)
    #define FP_N_HHEADINGS      (40)        // Number of horizontal headings
    #define FP_N_VHEADINGS      (11)        // Number of vertical headings
    #define FP_LEVEL            (5)         // The vertical heading for fp_level
    #define FP_TURN_INTERVAL    (450000)    // Heading change interval (in μs)
    #define FP_MAX_ISR_TIME     (240)       // A little longer than it takes to run the Timer/Counter3 ISR (in μs)
    #define FP_TARGETS_BUFFER   (100)       // Number of steps between a target and a margin
    #define FP_CAL_SYNC         (0x01E0)    // calReader sync pattern when complete packet received
    #define FP_CAL_SYNC_MASK    (0x01F0)    // Mask for FP_CAL_SYNC
    #define FP_CAL_N            (0)         // calReader bitcode for no magnet present
    #define FP_CAL_B            (1)         // calReader bitcode for magnet present
    #define FP_CAL_MORE         (100)       // Number of steps to take to eliminate backlash
    #ifdef FP_DEBUG_ISR
    #define FP_ISR_T_PIN        (A3)        // Digital pin used for temporary debugging purposes
    #define FP_ISR_I_PIN        (A4)        // Digital pin toggled HIGH at start of Timer/Counter3 ISR, LOW at end
    #define FP_ISR_W_PIN        (A5)        // Digital pin set HIGH when there's work for Timer/Counter3 ISR, LOW when none
    #endif
    #define FP_DIVER_BPS        (120)       // Transmission rate to diver in bits/sec (Want to do 24 bits in ~ 0.2 sec)
    #define FP_DIVER_NOM_TRANS      ((unsigned long)(1000000 / FP_DIVER_BPS))   // Nominal number of μs before the 2nd - Nth bit in a diver transmission group
    #define FP_DIVER_MIN_SILENCE    (2 * FP_DIVER_NOM_TRANS)            // Minimum μs of diver clock == LOW before transmission start
    #define FP_DIVER_TICKS_PER_INT  (250000 / (3 * FP_DIVER_BPS))       // CPU clock ticks per Timer/Counter4 ISR interrupt (64μs/tick; 3 interrupts per bit)
    #define FP_DIVER_TT_HEAD    (0x00)      // Diver packet type for heading
    #define FP_DIVER_TT_PIX0    (0x40)      // Diver packet type for pixel0
    #define FP_DIVER_TT_PIX1    (0x80)      // Diver packet type for pixel1
    #define FP_DIVER_TT_CMD     (0xC0)      // Diver packet type for command
    #define FP_DIVER_HOME_PACKET    (FP_DIVER_TT_CMD + 1)               // The "recalibrate to home" command to diver
    #define FP_DIVER_IDLE_PACKET    (FP_DIVER_TT_CMD + 0)               // The "do nothing" command to diver
    #ifdef FP_DEBUG_DIVER
    #define FP_LED_PIN          (53)        // The LED attached to this pin is on when sending a packet to the diver, off otherwise
    #endif


    /**
     * 
     * Macros
     * 
     **/

    // dirPin value to shorten  and lengthen the given cable
    #define fp_shorten(CABLE)      (((CABLE) == 0 || (CABLE) == 3) ? LOW : HIGH)
    #define fp_lengthen(CABLE)     (((CABLE) == 0 || (CABLE) == 3) ? HIGH : LOW)

    // True if, while on this heading, x rises with the passage of time. False otheriwse.
    #define fp_xIsRising(hh)     (hh < FP_N_HHEADINGS / 4 || hh >=  3 * (FP_N_HHEADINGS / 4))

    /**
     * 
     * Some enums for readability
     *
     **/
    enum fp_hTurns : byte {fp_left, fp_straight, fp_right};
    enum fp_vTurns : byte {fp_falling, fp_level, fp_rising};

    /**
     * 
     * Cartesian representation of a point in the flying space, in steps
     * 
     **/
    struct fp_Point3D {
        long x;
        long y;
        long z;
    };

    /**
     * 
     * Cable bundle representation of a point in the flying space, in steps
     * 
     * The four cables, distinguished by the index, are
     * 
     *   0 -- the cable whose hoist point is at (0, 0, space.z)
     *   1 -- the cable whose hoist point is at (space.x, 0, space.z)
     *   2 -- the cable whose hoist point is at (0, space.y, space.z)
     *   3 -- the cable whose hoist point is ar (space.x, space.y, space.z)
     * 
     **/
    struct fp_CableBundle {
        float c[4];             // The lengths of the cables
    };

    /**
     * 
     * Codes returned by various member functions
     * 
     * fp_ok:   Things went well
     * fp_oob:  The resulting move would have moved the platform outside of 
     *          the safe area. No movement was made.
     * fp_ncp:  The current position has not been set (i.e., we're not 
     *          calibrated), so the platform can't be moved. 
     * fp_dis:  The steppers are disabled, so the platform can't be moved.
     * fp_mov:  The operation can't be done because a move is underway.
     * fp_nom:  The operation can't be done because no move is underway.
     * 
     **/
    enum fp_return_code : byte{fp_ok, fp_oob, fp_ncp, fp_dis, fp_mov, fp_nom};

    class FlyingPlatform {
        public:
            /**
             * 
             * Make a new FlyingPlatform object representing a flying platform 
             * controlled by four stepper motors driven by TB6600 motor 
             * controllers driving winches located in the specified places. 
             * The hoist points for the four cables, measured in steps, are 
             * located at:
             *      cable 0:    (0, 0, spaceHeight)
             *      cable 1:    (spaceWidth, 0, spaceHeight)
             *      cable 2:    (0, spaceDepth, spaceHeight)
             *      cable 3:    (spaceWidth, spaceDepth, spaceHeight)
             * 
             * Parameters:
             *  pin0D       The direction pin for stepper 0. High means 
             *              lengthen cable.
             *  pin0P       The step pin for stepper 0. Normally low. Pulse 
             *              to move one step in direction indicated by pin0D.
             *  pin1D       Same but for stepper 1
             *  pin1P
             *  pin2D       Same but for stepper 2
             *  pin2p
             *  pin3D       Same but for stepper 3
             *  pin4P
             *  pinEn       Enable pin -- common to all steppers. Active low.
             *  pinCC       The calReader clock pin
             *  pinCD       The calReader data pin
             *  pinDC       The diver clock pin
             *  pinDD       The diver data pin
             *  spaceWidth  The width, in steps, of the space in which the 
             *              platform operates
             *  spaceDepth  Its depth
             *  spaceHeight Its height
             * 
             **/
            FlyingPlatform(
                byte pin0D, byte pin0P, 
                byte pin1D, byte pin1P, 
                byte pin2D, byte pin2P, 
                byte pin3D, byte pin3P,
                byte pinEn, 
                byte pinCC, byte pinCD,
                byte pinDC, byte pinDD,
                long spaceWidth, long spaceDepth, long spaceHeight);

            /**
             * 
             * Do the final initialization. Put this in the sketch's setup() 
             * function.
             * 
             **/
            void begin();

            /**
             * 
             * Have the FlyerPlatform object do its thing running the motors. 
             * Put an invocation of this in the sketch's loop() function.
             * 
             * Returns true if any of the motors still have steps to go; false 
             * otherwise.
             * 
             **/
            bool run();

            /**
             * 
             * Set the maximum speed, in steps per second, at which we're 
             * allowed to emit step pulses. Defaults to FP_MAX_SPEED.
             * 
             **/
            void setMaxSpeed(float speed);

            /**
             * 
             * Set the safety margins, the distances from each of the sides of 
             * the flying space from which the platform is excluded. Setting
             * these keeps the platform from accidententally crashing into 
             * parts of the mechanism that protrude into the flying space and 
             * from overstressing the mechanism. The overstressing part is 
             * particularly important for the top of the flying space since 
             * getting near to the top puts considerable strain on the cables.
             * Before this is called, the margins are all 0.
             * 
             * Parameters
             *  leftMargin      The platform's x coordinate must be more than 
             *                  leftMargin.
             *  rightMargin     The platform's x coordinate must be less than
             *                  (space.x - rightMargin).
             *  frontMargin     Analogous, but for minimum y
             *  backMargin      Analogous, but for maximum y
             *  bottomMargin    Analogous, but for minimum z
             *  topMargin       Analogous, but for maximum z
             * 
             * Returns fp_ok if all went well, fp_oob if the specified margins 
             * would cause the platform's current position to be out of 
             * bounds.
             * 
             **/
            fp_return_code setSafetyMargins(
                long leftMargin, long rightMargin, 
                long frontMargin, long backMargin, 
                long bottomMargin, long topMargin);

            /**
             * 
             * Set the batch size, the length, in steps, of of a batch of 
             * steps. This controls the integration interval used to get the 
             * flyer approximate straight tracks through 3-space as it moves. 
             * A shorter integration interval causes the deviation from a 
             * straight line the path the platform takes to diminish, 
             * but it increases the computational load. Maybe something about 
             * a cm in size? Defaults to 128 steps.
             * 
             **/
            void setBatchSize(long steps);

            /**
             * 
             * Set the current position to the given location in the flying 
             * space and reset the headings. That is, don't move anything, 
             * just assume that the platform is at the position specified, set 
             * the assumed length of the cables to match that, and set 
             * hHeading to 0, vHeading to FP_LEVEL.
             * 
             * Returns fp_ok if all went well, fp_oob if the specified 
             * position is out of bounds of the safety margins or fp_mov if the 
             * platform is currently moving. No change to the current location 
             * is made if fp_ok is not returned.
             * 
             **/
            fp_return_code setCurrentPosition (fp_Point3D tgt);

            /**
             * 
             * Calibrate the system. That is, move the platform to the 
             * sensor-defined "home" position and set the current position to 
             * the given location.
             * 
             * Returns fp_ok if all went well, fp_oob if the specified 
             * position is out of bounds of the safety margins or fp_mov if the 
             * platform is currently moving. No change to the current location 
             * is made if fp_ok is not returned.
             * 
             **/
            fp_return_code calibrate(fp_Point3D tgt);

            /**
             * 
             * Enable the motor drivers. When enabled, the steppers are 
             * capable of stepping. Returns fp_ok.
             * 
             **/
            fp_return_code enableOutputs();

            /**
             * 
             * Disable the motor drivers. Returns fp_ok if successful, If a
             * move is underway, returns fp_mov and motors are not disabled.
             * 
             * When disabled, the steppers can't step, but can be turned by 
             * hand. Also, disabled motors don't draw much power or create 
             * heat, probably cutting down on wear and tear on the mechanism. 
             * On the other hand, when powered down, the motors don't hold 
             * position very well at all and will turn by themselves if under 
             * load.
             * 
             **/
            fp_return_code disableOutputs();

            /**
             * 
             * Set things up so the platform will move to tgt. This call is 
             * non-blocking; the actual movement occurs over time as run() is 
             * repeatedly invoked and the ISR issues steps. If moveTo() is 
             * invoked while the platform is moving, its journey is rerouted 
             * from whatever it was to the new destination. If the plaform is 
             * stopped, it will start moving.
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_oob  if the specified position is outside the safety 
             *              margins. In this case the platform will continue 
             *              on its path if it was moving or remain at rest if 
             *              it wasn't. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/
            fp_return_code moveTo(fp_Point3D tgt);

            /**
             * 
             * Kick off the process of moving by delta from the current 
             * location. This is non-blocking; the actual work occurs over 
             * time as run() and the ISR are repeatedly invoked. If moveBy() 
             * is invoked while the platform is moving, its journey is 
             * rerouted from whatever it was to the new destination. If the
             * platform is stopped, it will start moving.
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_oob  if the specified position is outside the safety 
             *              margins. In this case the platform will continue 
             *              on its path if it was moving or remain at rest if 
             *              it wasn't. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/

            fp_return_code moveBy(fp_Point3D delta);

            /**
             * 
             * Set the platform moving along its heading-defined path. The 
             * heading-defined path consists of its horizontal and vertical 
             * headings, hHeading and vHeading, and how those headings are
             * changing, hChange and vChange. 
             * 
             * The variable hChange can have values fp_left, fp_straight, or 
             * fp_right. When it is fp_left the hHeading decreases (modulo the 
             * number of hHeadings) once every FP_TURN_INTERVAL. If it is 
             * fp_right, it increases in the same way. If it is fp_straight, 
             * the hHeading doesn't change.
             * 
             * The variable vChange is similar but a little different. It can
             * have the values fp_rising, fp_level, or fp_falling. if it is 
             * fp_rising, the vHeading increases once every FP_TURN_INTERVAL 
             * until it reaches its maximum value. If it is fp_falling, it
             * decreases in the same way until it reaches 0. If it is fp_level 
             * it either increases or decreases every FP_TURN_INTERVAL until 
             * it reaches FP_LEVEL, at which point it stops changing.
             *
             * Returns 
             *      fp_ok   if successful. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/
            fp_return_code go();

            /**
             * 
             * Change how the platform moves in the x-y direction. You can 
             * change direction while moving or not moving. If not moving the 
             * turning direction changes, but motion doesn't start.
             * 
             * See go() for details on how turns work.
             * 
             * Parameter
             *      dir     The direction to turn: fp_left, fp_striaght or 
             *              fp_right
             * 
             * Returns 
             *      fp_ok   always
             * 
             **/
            fp_return_code turn(fp_hTurns dir);

            /**
             * 
             * Change how the platform moves in the z direction. You can 
             * change direction while moving or not moving. If not moving the 
             * turning direction changes, but motion doesn't start.
             * 
             * See go() for details on how turns work.
             * 
             * Parameter
             *      dir     The direction to turn: fp_rising, fp_level or
             *              fp_falling.
             * 
             * Returns 
             *      fp_ok   if successful. 
             * 
             **/
            fp_return_code turn(fp_vTurns dir);

            /**
             * 
             * Stop all the motors as soon as the currently queued steps have 
             * been processed by the ISR.
             * 
             **/
            void stop();

            /**
             * Send current status information to Serial
             * 
             * Basically, for debugging
             * 
             */
            void status();

            /**
             * 
             * Return the platform's current location in the flying space.
             * 
             * NB: If the platform is moving, the location returned is the 
             * actual location at the time of the call, and the platform
             * continues to move.
             * 
             * Returns fp_Point3D {-1, -1, -1} if current position hasn't 
             * been set.
             * 
             **/
            fp_Point3D where();

            /**
             * 
             * Returns true if any of the motors currently have steps to make; 
             * false otherwise.
             * 
             **/
            bool isRunning();

            /**
             * 
             * Returns true if the platform motors are enabled.
             * 
             **/
            bool isEnabled();

            /**
             * 
             * Returns true if the patform is calibrated.
             * 
             **/
            bool isCalibrated();

            #ifdef FP_DEBUG_GEO
            /**
             * 
             * Returns an fp_Point3D that is the round trip through conversion of 
             * incoming to a fp_CableBundle and back again to a fp_Point3D
             * 
             **/
            fp_Point3D p3DToP3D(fp_Point3D incoming);
            #endif

        private:

            /**
             * 
             * Based on the starting position -- the position the platform 
             * will occupy when it could next make a change in direction -- 
             * and on hHeading and vHeading, return a fp_Point3D of the 
             * corresponding target. 
             * 
             * The target is the point where the line from the starting point 
             * and proceeding in the direction described by hHeading and 
             * vHeading first intersect one of the six planes described by 
             * targetsMin and targetsMax.
             * 
             * NB: The code assumes that currentIsSet but, for efficiency 
             * doesn't check, so be careful calling it.
             * 
             **/
            fp_Point3D newTarget();

            /**
             * 
             * Convert a fp_CableBundle to the corresponding fp_Point3D
             * 
             **/
            fp_Point3D cbToP3D(fp_CableBundle bundle);

            /**
             * 
             * Convert an fp_Point3D to the corresponding fp_CableBundle
             * 
             **/
            fp_CableBundle p3DToCb(fp_Point3D point);

            byte enablePin;                     // Common pin to enable/disable motor drivers. Active LOW.

            fp_Point3D space;                   // The right, back, top corner of the flying space
            fp_Point3D marginsMin;              // Safety margins minimum values (left, front, bottom)
            fp_Point3D marginsMax;              // Safety margins maximum values (right, back, top)
            fp_Point3D targetsMin;              // Target minimum values (left, front,bottom)
            fp_Point3D targetsMax;              // Target maximum values (right, back, top)
            long batchSteps;                    // The length along the direction of motion, in steps, of a batch

            fp_Point3D target;                  // Where we're trying to go
            fp_Point3D source;                  // Where we're coming from
            fp_CableBundle nextCableSteps;      // Cable lengths at the beginning of next batch of steps
            fp_Point3D nextPoint;               // The FP_Point3D corresponding to nextCableSteps
            float dt;                           // Fraction of the move a batch is
            float t;                            // How far along we are in the move [0..1]

            bool newMove;                       // True if a new move has been set but not started
            bool stopping;                      // True if waiting for the current batch to finish so we can stop

            float maxSpeed;                     // Max speed in steps per second.

            /**
             * 
             * In addition to its position, the flying platform has horizontal 
             * (x-y) and vertical (z along the x-y path) headings.
             * hSlope is the list of slopes (the m in y = mx + b) for the 40 
             * possible horizontal headings. 
             * 
             * The values are tan((4.5 + 9 * heading) / 360). That way we 
             * avoid slopes of 0 and ∞.
             *
             * NB: To simplify things, the code relies on non-zero and 
             * non-infinite values for slope.
             * 
             **/
            float hSlope[FP_N_HHEADINGS] =
            {  0.07870170682, 0.2400787591,  0.4142135624,  0.6128007881,  0.8540806855, 
               1.170849566,   1.631851687,   2.414213562,   4.16529977,   12.70620474,
             -12.70620474,   -4.16529977,   -2.414213562,  -1.631851687,  -1.170849566,
              -0.8540806855, -0.6128007881, -0.4142135624, -0.2400787591, -0.07870170682,
               0.07870170682, 0.2400787591,  0.4142135624,  0.6128007881,  0.8540806855,
               1.170849566,   1.631851687,   2.414213562,   4.16529977,   12.70620474,
             -12.70620474,   -4.16529977,   -2.414213562,  -1.631851687,  -1.170849566,
              -0.8540806855, -0.6128007881, -0.4142135624, -0.2400787591, -0.07870170682};
            int hHeading;                       // Which horizontal (x-y) slope the diver is currently on
            fp_hTurns hChange;                  // How hHeading is changing fp_left, fp_straight, fp_right
            fp_hTurns diverHLight;              // What diver's head light is showing (temporary)
            int diverhHeading;                  // Diver's yaw angle (same space as hHeading)

            /**
             * 
             * vSlope is the list of slopes for the 11 possible diver 
             * vHeadings -- the m in dz = m * sqrt(dx**2 + dy**2).
             * They are tan(10 * (vHeading - 5)), so 10 degrees apart, with 
             * vHeading = 5 as fp_level
             * 
             **/
            float vSlope[FP_N_VHEADINGS] = 
            {-1.1917535930, -0.8390996312, -0.5773502692, -0.3639702343, -0.1763269807,
              0, // <-- vChange == fp_level
              0.1763269807,  0.3639702343,  0.5773502692,  0.8390996312,  1.191753593};
            int vHeading;                       // Which vSlope diver is currently on
            fp_vTurns vChange;                  // How diver's vertical direction is changing
            fp_vTurns diverVLight;              // What diver's tail light is showing (temporary)
            unsigned long turnMicros;           // micros() when horizontal and vertical turns changed
            #ifdef FP_DEBUG_GEO
            float maxCableLength;               // Cables should never be longer than this
            #endif
    };
#endif