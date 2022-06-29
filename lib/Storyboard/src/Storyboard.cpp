/****
 * 
 * This file is a portion of the package Storyboard, a library that provides 
 * the state machine underlying "gameplay" for the PTMSC abalone diver 
 * exhibit. 
 * 
 * See Storyboard.h for details.
 * 
 *****
 * 
 * Storyboard V0.3, June 2022
 * Copyright (C) 2022 D.L. Ehnebuske
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
#include "Storyboard.h"

Storyboard* theInstance = nullptr;                     // The singleton Storyboard object

// The default action handler. If all goes well, never invoked
void _sb_defaultActionHandler(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId) {
    #ifdef _SB_DEBUG
    Serial.print(F("Default action handler called for stateId: "));
    Serial.print(stateId);
    Serial.print(F(", actionId: "));
    Serial.print(actionId);
    Serial.print(F(", and clipId: "));
    Serial.println(clipId);
    #endif
}

// The default trigger handler. If all goes well, never invoked
bool _sb_defaultTriggerHandler(sb_stateid_t stateId, sb_trigid_t triggerId) {
    #ifdef _SB_DEBUG
    Serial.print(F("Default trigger handler called for stateId: "));
    Serial.print(stateId);
    Serial.print(F(" and triggerId: "));
    Serial.println(triggerId);
    #endif
    return false;
}

Storyboard *Storyboard::getInstance() {
    if (theInstance == nullptr) {
        theInstance = new Storyboard();
    }
    return theInstance;
}

void Storyboard::begin() {
    #ifdef _SB_DEBUG
    // Validate that the number of the state enumeration and the index into sbState are identical.
    // When changing sbState to add, remove or rearrange states, it's easy to mess this up And
    // hard to figure out.
    for (uint8_t stateIx = 0; stateIx < SB_N_STATES; stateIx++) {
        if (stateIx != (uint8_t)sbState[stateIx].id) {
            Serial.print(F("State numbering mismatch. stateIx: "));
            Serial.print(stateIx);
            Serial.print(F(" doesn't match sbState[stateIx].id: "));
            Serial.println(sbState[stateIx].id);

        }
    }
    #endif
    currentStateId = SB_INIT_STATE;
    // Run through the new state's actions
    for (uint8_t actionIx = 0; actionIx < SB_MAX_ACTS && sbState[currentStateId].actionId[actionIx] != 0; actionIx++) {
        // Carry each one out in turn by invoking the corresponding action handler
        actionHandler[sbState[currentStateId].actionId[actionIx]](currentStateId, sbState[currentStateId].actionId[actionIx], sbState[currentStateId].clipId[actionIx]);
    }
}

void Storyboard::run() {
    // We only look for state transitions every SB_SCAN_MILLIS millis()
    if (millis() - lastScanMillis < SB_SCAN_MILLIS) {
        return;
    }
    // Run through the triggers for the current state
    for (uint8_t triggerIx = 0; triggerIx < SB_MAX_TRIGS && sbState[currentStateId].trigId[triggerIx] != 0; triggerIx++) {
        // If the trigger handler for triggerIx reports the trigger has occurred
        if (triggerHandler[sbState[currentStateId].trigId[triggerIx]](currentStateId, sbState[currentStateId].trigId[triggerIx])) {
            // Change state to the next state for that trigger
            #ifdef _SB_DEBUG
            Serial.print(F("Changing storyboard state from "));
            Serial.print(currentStateId);
            Serial.print(F(" to "));
            #endif
            currentStateId = sbState[currentStateId].nextStateId[triggerIx];
            #ifdef _SB_DEBUG
            Serial.println(currentStateId);
            Serial.print(F("Checking definition of state "));
            Serial.print(currentStateId);
            Serial.print(F(". Triggers "));
            for (uint8_t t = 0; t < SB_MAX_TRIGS && sbState[currentStateId].trigId[t] != 0; t++) {
                Serial.print(t);
                Serial.print(F(":"));
                Serial.print(sbState[currentStateId].trigId[t]);
                Serial.print(F(" "));
            }
            Serial.print(F("Actions "));
            for (uint8_t a = 0; a < SB_MAX_ACTS && sbState[currentStateId].actionId[a] != 0; a++) {
                Serial.print(a);
                Serial.print(F(":"));
                Serial.print(sbState[currentStateId].actionId[a]);
                Serial.print(F(" "));
            }
            Serial.print(F("\n"));
            #endif
            // Then run through the new state's actions
            for (uint8_t actionIx = 0; actionIx < SB_MAX_ACTS && sbState[currentStateId].actionId[actionIx] != 0; actionIx++) {
                // Carry each one out in turn by invoking the corresponding action handler
                actionHandler[sbState[currentStateId].actionId[actionIx]](currentStateId, sbState[currentStateId].actionId[actionIx], sbState[currentStateId].clipId[actionIx]);
            }
            // And we're done with this scan
            lastScanMillis = millis();
            return;
        }
    }
    lastScanMillis = millis();
}

void Storyboard::attachActionHandler(sb_actid_t id, sb_actionhandler handler) {
    actionHandler[id] = handler;
}

void Storyboard::attachTriggerHandler(sb_trigid_t triggerId, sb_triggerhandler_t handler) {
   triggerHandler[triggerId] = handler;
}

Storyboard::Storyboard() {
        for (uint8_t ix = 0; ix < SB_N_ACTS; ix++) {
            actionHandler[ix] = _sb_defaultActionHandler;
        }
        for (uint8_t ix = 0; ix < SB_N_TRIGS; ix++) {
            triggerHandler[ix] = _sb_defaultTriggerHandler;
        }
        lastScanMillis = millis();
    }
