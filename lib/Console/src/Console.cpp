/****
 * 
 * This file is a portion of the package Console, a library that provides 
 * an Arduino sketch with the representation of console for the PTMSC abalone 
 * diver exhibit. See console.h for details.
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

#include "Console.h"

Console::Console(int8_t pinL, int8_t pinF, int8_t pinR, int8_t pinB, int8_t pinD, int8_t pinU, int8_t pinP, int8_t pinLED) {
    pin[clSwitch::l] = pinL;
    pin[clSwitch::f] = pinF;
    pin[clSwitch::r] = pinR;
    pin[clSwitch::b] = pinB;
    pin[clSwitch::d] = pinD;
    pin[clSwitch::u] = pinU;
    pin[clSwitch::p] = pinP;
    for(int8_t i = 0; i <= CL_EVENT_MAX; i++) {
        handler[i] = NULL;
    }
    pLed = pinLED;
}

void Console::begin() {
    for (int8_t sw = 0; sw <= CL_SWITCH_MAX; sw++) {
        pinMode(pin[sw], INPUT);
        lastDisMillis[sw] = 0;
    }
    pinMode(pLed, OUTPUT);
    digitalWrite(pLed, HIGH);
    pLedState = false;
}

void Console::run() {
    unsigned long now = max(millis(), 1);   // 1 is special: used to indicate no reading change since switches[sw] last set
    for (int8_t sw = 0; sw <= CL_SWITCH_MAX; sw++) {
        bool newValue = digitalRead(pin[sw]) == HIGH ? true : false;
        if (lastDisMillis[sw] == 0) {
            if (newValue != switches[sw]) {
                lastDisMillis[sw] = now;                            // Debounce starts now
            }
        } else if (now - lastDisMillis[sw] > CL_DEBOUNCE_MILLIS) {  // If it's been long enough for bouncing to stop
            lastDisMillis[sw] = 0;
            switches[sw] = newValue;
            clHandler h = handler[eventMap[sw][newValue == true ? 0 : 1]];
            if (h != NULL){
                (*h)();                                             // Invoke handler if ther is one
            }
        }
    }
}

void Console::attachHandler(clEvent s, clHandler h) {
    handler[s] = h;
}

void Console::setPlaceLED(bool state) {
    digitalWrite(pLed, state == true ? LOW : HIGH);
    pLedState = state;
}

bool Console::placeLedIsOn() {
    return pLedState;
}