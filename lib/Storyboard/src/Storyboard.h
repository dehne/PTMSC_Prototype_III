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
 * Storyboard V0.1, May 2022
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

#define _SB_DEBUG               //Uncomment to enable debug printing


struct sb_site_t {                                          // A description of a outplacement site
    fp_Point3D loc;                                         //   Its location in "flying space" (mm)
    bool isFull;                                            //   True id an outplacement has been placed there.
};
                                                            // The location of the sites in the flying space (mm)

#define SB_MAX_TRIGS    (18)                                // The maximum number of triggers a state can have (adjust as needed)
#define SB_MAX_ACTS     (2)                                 // The maximum number of actions a state can have (adjust as needed)
struct sb_state_t {                                         // The representation of a state
    sb_stateid_t id;                                        //   Which state this is
    sb_actid_t actionId[SB_MAX_ACTS];                       //   The action(s) to take, in sequence
    sb_clipid_t clipId[SB_MAX_ACTS];                        //   The associated video clip; noClip if none
    sb_trigid_t trigId[SB_MAX_TRIGS];                       //   The trigger(s) that cause a state change in precedence order
    sb_stateid_t nextStateId[SB_MAX_TRIGS];                 //   The id of the new state upon occurance of corresponding trigger
};

const sb_state_t sbState[] = {
    {diving,                                                // State
        {setLoop}, {divingLoop},                            //   Action(s) Clip(s)
        {nearOpenSiteCohorts1, nearOpenSiteCohorts2, nearOpenSiteCohorts3, nearOpenSiteCohorts4, nearOpenSiteCohorts5,
         nearFullSiteCohorts1, nearFullSiteCohorts2, nearFullSiteCohorts3, nearFullSiteCohorts4, nearFullSiteCohorts5,
         nearSiteNoCohorts1,   nearSiteNoCohorts2,   nearSiteNoCohorts3,   nearSiteNoCohorts4,   nearSiteNoCohorts5,
         nearBoatCohorts, nearBoatNoCohorts, asynchTimer},  //   Trigger(s)
        {arriveOpenSite1,      arriveOpenSite2,      arriveOpenSite3,      arriveOpenSite4,      arriveOpenSite5,
         arriveFullSite1,      arriveFullSite2,      arriveFullSite3,      arriveFullSite4,      arriveFullSite5,
         arriveSite1,          arriveSite2,          arriveSite3,          arriveSite4,          arriveSite5, 
         arriveBoatA,     arriveboatB,       abandoned}},   //   Next state(s)

    {resting,                                               // State
        {setLoop, prepareNew}, {restingLoop},               //   Action(s) Clip(s)
        {touchJoystick},                                    //   Trigger(s)
        {diving}},                                          //   Next state(s)
        
    {timerPop,                                              // State
        {disableControls}, {noClip},                        //   Action(s) Clip(s)
        {always},                                           //   Trigger(s)
        {abandoned}},                                       //   Next state(s)
        
    {abandoned,                                             // State
        {playClip, setLoop}, {abandonedClip, restingLoop},  //   Action(s) Clip(s)
        {videoEnds},                                        //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {arriveFullSite1,                                       // State
        {playClip}, {fullSiteClip1},                        //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite2,                                       // State
        {playClip}, {fullSiteClip2},                         //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite3,                                       // State
        {playClip}, {fullSiteClip3},                        //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite4,                                       // State
        {playClip}, {fullSiteClip4},                        //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite5,                                       // State
        {playClip}, {fullSiteClip5},                        //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveOpenSite1,                                       // State
        {setLoop}, {openSiteLoop1},                         //   Action(s) Clip(s)
        {awayFromSite1, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite1, abandoned}},                    //   Next state(s)

    {arriveOpenSite2,                                       // State
        {setLoop}, {openSiteLoop2},                         //   Action(s) Clip(s)
        {awayFromSite2, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite2, abandoned}},                    //   Next state(s)

    {arriveOpenSite3,                                       // State
        {setLoop}, {openSiteLoop3},                         //   Action(s) Clip(s)
        {awayFromSite3, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite3, abandoned}},                    //   Next state(s)

    {arriveOpenSite4,                                       // State
        {setLoop}, {openSiteLoop4},                         //   Action(s) Clip(s)
        {awayFromSite4, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite4, abandoned}},                    //   Next state(s)

    {arriveOpenSite5,                                       // State
        {setLoop}, {openSiteLoop5},                         //   Action(s) Clip(s)
        {awayFromSite5, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite5, abandoned}},                    //   Next state(s)

    {fillSite1,                                             // State
        {playClip, deposit1}, {fillSiteClip1},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite2,                                             // State
        {playClip, deposit2}, {fillSiteClip2},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite3,                                             // State
        {playClip, deposit3}, {fillSiteClip3},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite4,                                             // State
        {playClip, deposit4}, {fillSiteClip4},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite5,                                             // State
        {playClip, deposit5}, {fillSiteClip5},              //   Action(s) Clip(s)
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {arriveSite1,                                           // State
        {playClip}, {siteNoCohortsClip1},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite2,                                           // State
        {playClip}, {siteNoCohortsClip2},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite3,                                           // State
        {playClip}, {siteNoCohortsClip3},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite4,                                           // State
        {playClip}, {siteNoCohortsClip4},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite5,                                           // State
        {playClip}, {siteNoCohortsClip5},                   //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveBoatA,                                           // State
        {playClip}, {boatCohortsClip},                      //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveboatB,                                           // State
        {playClip, disableControls}, {transitionClip},      //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atBoatB, abandoned}},                              //   Next state(s)

    {atBoatB,                                               // State
        {doSurvivalSequence}, {noClip},                     //   Action(s) Clip(s)
        {sequenceFinished},                                 //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {outplanted,                                            // State
        {playClip}, {outplantedClip},                       //   Action(s) Clip(s)
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}}                                //   Next state(s)
};

// The type of action handler invoked to carry out a specific action. The meaning of id is handler dependent
typedef void(*sb_actionhandler)(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId);

// The type of a trigger handler invoked to say whether a particular trigger has occurred
typedef bool(*sb_triggerhandler_t)(sb_stateid_t stateId, sb_trigid_t triggerId);

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

        sb_stateid_t currentStateId = resting;              // The current state of the storyboard state machine
        sb_actionhandler actionHandler[SB_N_ACTS];          // The registry of action handler functions
        sb_triggerhandler_t triggerHandler[SB_N_TRIGS];     // The registry of trigger sensor functions
};