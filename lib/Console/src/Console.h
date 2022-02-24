/****
 * 
 * This file is a portion of the package Console, a library that provides 
 * an Arduino sketch with the representation of console for the PTMSC abalone 
 * diver exhibit. 
 * 
 * The console consists of an arcade joystick that actuates four switches -- 
 * forward, right, backward and left depending on the direction the stick is 
 * pushed. If the stick is not pushed, none of the switches are closed. The 
 * joystick can distinguish 8 directons because adjacent pairs of switches -- 
 * forward and left, for exanple -- can be actuated at the same time. The 
 * console also has three push-button switches with LED lights built in. The 
 * light for two of the switches -- Down and Up -- are wired to be on whenever 
 * power is on. The third switch -- Place -- can be truned on and off. 
 * 
 * The design of the push buttons is such that +5v is common to the switch and 
 * accompanying LED. To turn on the LED, the pin to which it attached is set 
 * to LOW. To turn it off, the pin is set to HIGH. That means when the switch 
 * is open, the pin to which it is attached would go nowhere. That's not good, 
 * so the console incorporates pull-down resistors weakly pulling all the 
 * switch pins to ground. When a switch closes, the pin to which it's attached 
 * sees HIGH. To keep things sensible, this is also true for the four switches 
 * in the joystick.
 * 
 * This library implements the Console object representing an instance of the 
 * console. It allows sketches to react to console state changes and to 
 * control the Place button LED. The sketch is informed of the state changes 
 * it is interested in as they occur at runtime. To inform the Console object 
 * that the sketch is interested in a state change, it uses attachHandler() to 
 * register a parameterless, void function called an event handlers that the 
 * Console object invokes at runtime as the corresponding events happen.
 * 
 * The genral pattern for using Console is to instantiate a Console object as 
 * a global. Then in setup() invoke begin(). With this done, invoke 
 * attacheHandler() as needed to register the events of interest. Then in 
 * loop() invoke run() every time through.
 * 
 * The Place LED may be controlled using 
 * 
 *****
 * 
 * Console V0.1, July 2021
 * Copyright (C) 2021 D.L. Ehnebuske
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
#ifndef CONSOLE
    #define CONSOLE

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

    #define CL_DEBOUNCE_MILLIS          (10)        //Switch debound=ce time in ms

    // The indexes for the switches we have
    enum clSwitch : int8_t {l, f, r, b, d, u, p, CL_SWITCH_MAX = p};

    // The events the console detects
    enum clEvent : int8_t {evForward, evStop, evBack, evLeft, evNeutral, evRight, 
        evPlacePressed, evPlaceReleased, evDownPressed, evDownReleased, evUpPressed, evUpReleased, CL_EVENT_MAX = evUpReleased};

    extern "C" {
        // User-supplied handler functions always follow the signature: void cmd(void);
        typedef void(*clHandler) (void);
    }
    class Console {
        public:
            /**
             * 
             * Make a new COnsole object. 
             * 
             * Parameters
             *  pinL    The pin to which the joystick left switch is attached
             *  pinF    The pin to which the joystick forward switch is attached
             *  pinR    The pin to which the joystick right switch is attached
             *  pinB    The pin to which the joystick back switch is attached
             *  pinD    The pin to which the Up button switch is attached
             *  pinU    The pin to which the Down button switch is attached
             *  pinP    The pin to which the Place button switch is attached
             *  pinLED  The pin to which the LED on the Place button is attached
             * 
             **/
            Console(int8_t pinL, int8_t pinF, int8_t pinR, int8_t pinB, int8_t pinD, int8_t pinU, int8_t pinP, int8_t pinLED);

            /**
             * 
             * Initialize Console Object. Invoke this in the sketch's setup()
             * 
             **/
            void begin();

            /**
             * 
             * Look at the console hardware, detect any state changes, and
             * invoke the requisite user supplied handler functions.
             * 
             **/
            void run();

            /**
             * 
             * Add a handler for a specigfied event. When the console detects 
             * the event, the handler is invoked. Adding a handler for a state 
             * that already has a handle, replaces the old one with the new 
             * one. If you add NULL as a handler, the existing one is removed, 
             * and no new one is added.
             * 
             * Parameters
             *  s       The clEvent that the supplied handler is for
             *  h       The handler function, a void function with no 
             *          parameters
             * 
             **/
            void attachHandler(clEvent s, clHandler h);

            /**
             * 
             * Control the state of the Place LED.
             * 
             * Parameters
             *  state   A bool that turns the light on if true, off if false
             * 
             **/
            void setPlaceLED(bool state);

            /**
             * 
             * Query the state of the Place LED. Returns true if the LED is 
             * on, false if off. Initially it is set off.
             * 
             **/
            bool placeLedIsOn();

        private:
            clHandler handler[CL_EVENT_MAX + 1];                        // The user-supplied event handlers

            bool switches[CL_SWITCH_MAX + 1];                           // The state of the switches
            unsigned long lastDisMillis[CL_SWITCH_MAX + 1];             // millis() the last time we saw the switch reading disagree with switches[]

            int8_t pin[CL_SWITCH_MAX + 1];                              // The GPIO pins the switches are attached to
            int8_t pLed;                                                // The GPIO pin the Place LED is attached to
            bool pLedState;                                             // The current state of the Place LED
            clEvent eventMap[CL_SWITCH_MAX + 1][2] =                    // Switch new state to event map: new state == true, then == false
                {{clEvent::evLeft, clEvent::evNeutral},                 // l (e.g., left switch goes true ==> evLeft event; goes false ==> evNeutral event)
                 {clEvent::evForward, clEvent::evStop},                 // f
                 {clEvent::evRight, clEvent::evNeutral},                // r
                 {clEvent::evBack, clEvent::evStop},                    // b
                 {clEvent::evUpPressed, clEvent::evUpReleased},         // u
                 {clEvent::evDownPressed, clEvent::evDownReleased},     // d
                 {clEvent::evPlacePressed, clEvent::evPlaceReleased}    // p
                };
    };
#endif