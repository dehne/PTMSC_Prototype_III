/****
 * 
 * This file is a portion of the package Storyboard, a library that provides 
 * the state machine underlying "gameplay" for the PTMSC abalone diver 
 * exhibit. 
 * 
 * The storyboard for the exhibit consists of a singleton steps works through 
 * the story described by the state machine in sbState[]. Each element of 
 * sbState describes a state of the machine: its id, the action(s) to be taken 
 * upon entry to the state, the video clip associated with the state (if any) 
 * the precedence-ordered environmental trigger(s) (e.g., finishing playing 
 * the associated video clip, the diver getting near a site in the exhibit, 
 * etc.), and the new state to be entered when a trigger occurs. To separate 
 * the concerns of the state machine from other things, determining how each 
 * action is carried out and whether a given trigger has occurred is all 
 * contained in callback functions -- action handlers and trigger sensors, 
 * resepctively. These are registered during initialization -- the Arduino 
 * setup() function.
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
#pragma once
#include <Arduino.h>
#include "FlyingPlatform.h"
#include "Storyboardtypes.h"

#define _SB_DEBUG                                           // Uncomment to enable debug printing
//#define _SB_VERBOSE                                         // Uncomment for even more
#define SB_SCAN_MILLIS  (300)                               // millis() between scanning for triggers


struct sb_site_t {                                          // A description of a outplacement site
    fp_Point3D loc;                                         //   Its location in "flying space" (mm)
    bool isFull;                                            //   True id an outplacement has been placed there.
};
                                                            // The location of the sites in the flying space (mm)
const sb_state_t sbState[] = {
    {diving,                                                        // State
        {setLoop}, {divingLoop},                                    //   Action(s) Clip(s)
        {nearOpenSite1Cohorts, nearOpenSite2Cohorts, nearOpenSite3Cohorts, nearOpenSite4Cohorts, nearOpenSite5Cohorts,
         nearFullSite1Cohorts, nearFullSite1Cohorts, nearFullSite3Cohorts, nearFullSite4Cohorts, nearFullSite5Cohorts,
         nearSite1NoCohorts,   nearSite2NoCohorts,   nearSite3NoCohorts,   nearSite4NoCohorts,   nearSite5NoCohorts,
         nearBoatCohorts, nearBoatNoCohorts, asynchTimer},          //   Trigger(s)
        {arriveOpenSite1,      arriveOpenSite2,      arriveOpenSite3,      arriveOpenSite4,      arriveOpenSite5,
         arriveFullSite1,      arriveFullSite2,      arriveFullSite3,      arriveFullSite4,      arriveFullSite5,
         arriveSite1,          arriveSite2,          arriveSite3,          arriveSite4,          arriveSite5, 
         arriveBoatA,          arriveboatB,          timerPop}},    //   Next state(s)

    {resting,                                               // State
        {setLoop, prepareNew}, {restingLoop},               //   Action(s) Clip(s)
        {touchJoystick},                                    //   Trigger(s)
        {instruct}},                                        //   Next state(s)
        
    {timerPop,                                              // State
        {disableControls}, {noClip},                        //   Action(s) Clip(s)
        {always},                                           //   Trigger(s)
        {abandoned}},                                       //   Next state(s)
        
    {powerUp,                                               // State
        {setLoop}, {calibrateLoop},                         //   Action(s) Clip(s)
        {calibrated},                                       //   Trigger(s)
        {resting}},                                         //   Next state(s)
        
    {abandoned,                                             // State
        {playClip, setLoop}, {abandonedClip, restingLoop},  //   Action(s) Clip(s)
        {videoEnds},                                        //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {arriveFullSite1,                                       // State
        {setLoop}, {fullSite1Loop},                         //   Action(s) Clip(s)
        {awayFromSite1, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveFullSite2,                                       // State
        {setLoop}, {fullSite2Loop},                         //   Action(s) Clip(s)
        {awayFromSite2, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveFullSite3,                                       // State
        {setLoop}, {fullSite3Loop},                         //   Action(s) Clip(s)
        {awayFromSite3, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveFullSite4,                                       // State
        {setLoop}, {fullSite4Loop},                         //   Action(s) Clip(s)
        {awayFromSite4, asynchTimer},                       //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite5,                                       // State
        {setLoop}, {fullSite5Loop},                         //   Action(s) Clip(s)
        {awayFromSite5, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveOpenSite1,                                       // State
        {setLoop}, {openSite1Loop},                         //   Action(s) Clip(s)
        {awayFromSite1, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite1, timerPop}},                     //   Next state(s)

    {arriveOpenSite2,                                       // State
        {setLoop}, {openSite2Loop},                         //   Action(s) Clip(s)
        {awayFromSite2, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite2, timerPop}},                     //   Next state(s)

    {arriveOpenSite3,                                       // State
        {setLoop}, {openSite3Loop},                         //   Action(s) Clip(s)
        {awayFromSite3, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite3, timerPop}},                     //   Next state(s)

    {arriveOpenSite4,                                       // State
        {setLoop}, {openSite4Loop},                         //   Action(s) Clip(s)
        {awayFromSite4, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite4, abandoned}},                    //   Next state(s)

    {arriveOpenSite5,                                       // State
        {setLoop}, {openSite5Loop},                         //   Action(s) Clip(s)
        {awayFromSite5, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite5, timerPop}},                     //   Next state(s)

    {fillSite1,                                             // State
        {playClip, deposit1}, {fillSite1Clip},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {atSite1, outAtSite1, timerPop}},                   //   Next state(s)

    {fillSite2,                                             // State
        {playClip, deposit2}, {fillSite2Clip},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {atSite2, outAtSite2, timerPop}},                   //   Next state(s)

    {fillSite3,                                             // State
        {playClip, deposit3}, {fillSite3Clip},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {atSite3, outAtSite3, timerPop}},                   //   Next state(s)

    {fillSite4,                                             // State
        {playClip, deposit4}, {fillSite4Clip},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {atSite4, outAtSite4, timerPop}},                   //   Next state(s)

    {fillSite5,                                             // State
        {playClip, deposit5}, {fillSite5Clip},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {atSite5, outAtSite5, timerPop}},                   //   Next state(s)

    {atSite1,                                               // State
        {setLoop}, {atSite1Loop},                           //   Action(s) Clip(s)
        {awayFromSite1, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {atSite2,                                               // State
        {setLoop}, {atSite2Loop},                           //   Action(s) Clip(s)
        {awayFromSite2, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {atSite3,                                               // State
        {setLoop}, {atSite3Loop},                           //   Action(s) Clip(s)
        {awayFromSite3, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {atSite4,                                               // State
        {setLoop}, {atSite4Loop},                           //   Action(s) Clip(s)
        {awayFromSite4, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {atSite5,                                               // State
        {setLoop}, {atSite5Loop},                           //   Action(s) Clip(s)
        {awayFromSite5, asynchTimer},                       //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveSite1,                                           // State
        {playClip}, {site1NoCohortsClip},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite1, timerPop}},                               //   Next state(s)

    {arriveSite2,                                           // State
        {playClip}, {site2NoCohortsClip},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite2, timerPop}},                               //   Next state(s)

    {arriveSite3,                                           // State
        {playClip}, {site3NoCohortsClip},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite3, timerPop}},                               //   Next state(s)

    {arriveSite4,                                           // State
        {playClip}, {site4NoCohortsClip},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite4, timerPop}},                               //   Next state(s)

    {arriveSite5,                                           // State
        {playClip}, {site5NoCohortsClip},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite5, timerPop}},                               //   Next state(s)

    {outAtSite1,                                            // State
        {playClip}, {outAtSite1Clip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite1, timerPop}},                               //   Next state(s)
        
    {outAtSite2,                                            // State
        {playClip}, {outAtSite2Clip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite2, timerPop}},                               //   Next state(s)
        
    {outAtSite3,                                            // State
        {playClip}, {outAtSite3Clip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite3, timerPop}},                               //   Next state(s)
        
    {outAtSite4,                                            // State
        {playClip}, {outAtSite4Clip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite4, timerPop}},                               //   Next state(s)
        
    {outAtSite5,                                            // State
        {playClip}, {outAtSite5Clip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atSite5, timerPop}},                               //   Next state(s)
        
    {arriveBoatA,                                           // State
        {setLoop}, {boatCohortsLoop},                       //   Action(s) Clip(s)
        {awayFromBoat, asynchTimer},                        //   Trigger(s)
        {diving, timerPop}},                                //   Next state(s)

    {arriveboatB,                                           // State
        {playClip, disableControls}, {transitionClip},      //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atBoatB, timerPop}},                               //   Next state(s)

    {atBoatB,                                               // State
        {survival}, {noClip},                               //   Action(s) Clip(s)
        {sequenceFinished},                                 //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {instruct,                                              // State
        {setLoop}, {instructLoop},                          //   Action(s) Clip(s)
        {awayFromBoat},                                     //   Trigger(s)
        {diving}}                                           //   Next state(s)
};
#define SB_INIT_STATE           (sb_stateid_t::powerUp)     // The id of the initial state

// The definition of the outplanting sites. Each consists of a location in 3-space (mm) and an 
// indication of whether the visitor has done an outplanting in it
static sb_site_t sb_site[] = {
    {{200, 420, 15}, false}, 
    {{260, 270, 15}, false}, 
    {{440, 360, 15}, false}, 
    {{900, 480, 15}, false}, 
    {{720, 460, 15}, false}};
#define SB_SITE_COUNT      ((int)(sizeof(sb_site) / sizeof(sb_site[0])))    // The number of sites there are

class Storyboard {
    public:
        /****
         * 
         * getInstance()
         *      Return the storyboard instance
         * 
         ****/
        static Storyboard* getInstance();

        /****
         * 
         * begin()
         *      Start the state machine running from state initialStateId. Set up the
         *      trigger and action handlers before invoking.
         * 
         ****/
        void begin();

        /****
         * 
         * run()
         *      Have the storyboard do its thing -- checking the triggers for the current state to 
         *      see if one has ocurred and, if so entering the next state and doing that state's 
         *      actions.
         * 
         ****/
        void run();

        /****
         * 
         * attachActionHandler()
         *      Attach the handler function for the specified action. Attaching a handler
         *      for an action that already has a handler replaces the old one with the new one.
         * 
         * Parameters
         *      actionId    The id of the action whose handler is to be attached
         *      handler     The handler function to be attached
         * 
         ****/
        void attachActionHandler(sb_actid_t actionId, sb_actionhandler handler);

        /****
         * 
         * attachTriggerHandler()
         *      Attach the handler function for the specified trigger sensor. Attaching a 
         *      handler for a trigger that already has an handler replaces the old one with the 
         *      new one.
         * Parameters
         *      triggerId   The id of the trigger whose handler is to be attached
         *      handler     The handler function to be attached
         * 
         ****/
        void attachTriggerHandler(sb_trigid_t triggerId, sb_triggerhandler_t handler);

    private:
        /****
         * 
         * Constructor -- instantiate the storyboard
         * 
         ****/
        Storyboard();

        sb_stateid_t currentStateId;                        // The current state of the storyboard state machine
        sb_actionhandler actionHandler[SB_N_ACTS];          // The registry of action handler functions
        sb_triggerhandler_t triggerHandler[SB_N_TRIGS];     // The registry of trigger sensor functions
        unsigned long lastScanMillis;                       // millis() at the point we last scanned for triggers having occurred
};