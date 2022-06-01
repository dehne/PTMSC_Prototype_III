/****
 * 
 * This file is a portion of the package Storyboard, a library that provides 
 * the the state machine underlying "gameplay" for the PTMSC abalone diver 
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
#include <Arduino.h>
#pragma once

#define _SB_DEBUG               //Uncomment to enable debug printing

// The storyboard state values and SB_N_STATES, the number of states in the storyboard
enum sb_stateid_t : uint8_t {diving, resting, abandoned, 
                            arriveFullSite1, arriveFullSite2, arriveFullSite3, arriveFullSite4, arriveFullSite5,
                            arriveOpenSite1, arriveOpenSite2, arriveOpenSite3, arriveOpenSite4, arriveOpenSite5,
                            fillSite1,       fillSite2,       fillSite3,       fillSite4,       fillSite5,
                            arriveSite1,     arriveSite2,     arriveSite3,     arriveSite4,     arriveSite5,
                            arriveBoatA, arriveboatB, atBoatB, outplanted, SB_N_STATES};

// The storyboard trigger values and SB_N_TRIGS, the number of triggers in the storyboard
enum sb_trigid_t : uint8_t {nullTrigger, asynchTimer, videoEnds, touchJoystick, 
                            nearOpenSiteCohorts1, nearOpenSiteCohorts2, nearOpenSiteCohorts3, nearOpenSiteCohorts4, nearOpenSiteCohorts5,
                            nearFullSiteCohorts1, nearFullSiteCohorts2, nearFullSiteCohorts3, nearFullSiteCohorts4, nearFullSiteCohorts5,
                            nearSiteNoCohorts1,   nearSiteNoCohorts2,   nearSiteNoCohorts3,   nearSiteNoCohorts4,   nearSiteNoCohorts5,
                            awayFromSite1,        awayFromSite2,        awayFromSite3,        awayFromSite4,        awayFromSite5,
                            videoEndsCohorts, videoEndsNoCohorts, nearBoatCohorts, nearBoatNoCohorts, pressPlaceButton, 
                            sequenceFinished, SB_N_TRIGS};

// The storyboard action values
enum sb_actid_t : uint8_t  {nullAction, startLoop, playClip, enableControls, disableControls, 
                            deposit1, deposit2, deposit3, deposit4, deposit5,
                            doSurvivalSequence, SB_N_ACTS = doSurvivalSequence};

// The storyboard clip values
enum sb_clipid_t : uint8_t {noClip, divingLoop, restingLoop, abandonedClip, 
                            fullSiteClip1,      fullSiteClip2,      fullSiteClip3,      fullSiteClip4,      fullSiteClip5,
                            siteNoCohortsClip1, siteNoCohortsClip2, siteNoCohortsClip3, siteNoCohortsClip4, siteNoCohortsClip5,
                            openSiteLoop1,      openSiteLoop2,      openSiteLoop3,      openSiteLoop4,      openSiteLoop5,
                            fillSiteClip1,      fillSiteClip2,      fillSiteClip3,      fillSiteClip4,      fillSiteClip5,
                            boatCohortsClip, outplantedClip, transitionClip, SB_N_CLIPS};

#define SB_MAX_TRIGS    (18)                                // The maximum number of triggers a state can have (adjust as needed)
#define SB_MAX_ACTS     (2)                                 // The maximum number of actions a state can have (adjust as needed)
struct sb_state_t {                                         // The representation of a state
    sb_stateid_t id;                                        //   Which state this is
    sb_actid_t actionId[SB_MAX_ACTS];                       //   The action(s) to take, in sequence
    sb_clipid_t clipId;                                     //   The associated video clip or loop
    sb_trigid_t trigId[SB_MAX_TRIGS];                       //   The trigger(s) that cause a state change in precedence order
    sb_stateid_t nextStateId[SB_MAX_TRIGS];                 //   The id of the new state upon occurance of corresponding trigger
};

const sb_state_t sbState[] = {
    {diving,                                                // State
        {startLoop}, divingLoop,                            //   Action(s) Clip
        {nearOpenSiteCohorts1, nearOpenSiteCohorts2, nearOpenSiteCohorts3, nearOpenSiteCohorts4, nearOpenSiteCohorts5,
         nearFullSiteCohorts1, nearFullSiteCohorts2, nearFullSiteCohorts3, nearFullSiteCohorts4, nearFullSiteCohorts5,
         nearSiteNoCohorts1,   nearSiteNoCohorts2,   nearSiteNoCohorts3,   nearSiteNoCohorts4,   nearSiteNoCohorts5,
        nearBoatCohorts, nearBoatNoCohorts, asynchTimer},  //   Trigger(s)
        {arriveOpenSite1,      arriveOpenSite2,      arriveOpenSite3,      arriveOpenSite4,      arriveOpenSite5,
         arriveFullSite1,      arriveFullSite2,      arriveFullSite3,      arriveFullSite4,      arriveFullSite5,
         arriveSite1,          arriveSite2,          arriveSite3,          arriveSite4,          arriveSite5, 
         arriveBoatA,     arriveboatB,       abandoned}},   //   Next state(s)

    {resting,                                               // State
        {startLoop, enableControls}, restingLoop,           //   Action(s) Clip
        {touchJoystick},                                    //   Trigger(s)
        {diving}},                                          //   Next state(s)
        
    {abandoned,                                             // State
        {disableControls, playClip}, abandonedClip,         //   Action(s) Clip
        {videoEnds},                                        //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {arriveFullSite1,                                       // State
        {playClip}, fullSiteClip1,                          //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite2,                                       // State
        {playClip}, fullSiteClip2,                          //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite3,                                       // State
        {playClip}, fullSiteClip3,                          //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite4,                                       // State
        {playClip}, fullSiteClip4,                          //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveFullSite5,                                       // State
        {playClip}, fullSiteClip5,                          //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveOpenSite1,                                       // State
        {startLoop}, openSiteLoop1,                         //   Action(s) Clip
        {awayFromSite1, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite1, abandoned}},                    //   Next state(s)

    {arriveOpenSite2,                                       // State
        {startLoop}, openSiteLoop2,                         //   Action(s) Clip
        {awayFromSite2, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite2, abandoned}},                    //   Next state(s)

    {arriveOpenSite3,                                       // State
        {startLoop}, openSiteLoop3,                         //   Action(s) Clip
        {awayFromSite3, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite3, abandoned}},                    //   Next state(s)

    {arriveOpenSite4,                                       // State
        {playClip}, openSiteLoop4,                          //   Action(s) Clip
        {awayFromSite4, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite4, abandoned}},                    //   Next state(s)

    {arriveOpenSite5,                                       // State
        {playClip}, openSiteLoop5,                          //   Action(s) Clip
        {awayFromSite5, pressPlaceButton, asynchTimer},     //   Trigger(s)
        {diving, fillSite5, abandoned}},                    //   Next state(s)

    {fillSite1,                                             // State
        {deposit1, playClip}, fillSiteClip1,                //   Action(s) Clip
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite2,                                             // State
        {deposit2, playClip}, fillSiteClip2,                //   Action(s) Clip
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite3,                                             // State
        {deposit3, playClip}, fillSiteClip3,                //   Action(s) Clip
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite4,                                             // State
        {deposit4, playClip}, fillSiteClip4,                //   Action(s) Clip
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {fillSite5,                                             // State
        {deposit5, playClip}, fillSiteClip5,                //   Action(s) Clip
        {videoEndsCohorts, videoEndsNoCohorts, asynchTimer},//   Trigger(s)
        {diving, outplanted, abandoned}},                   //   Next state(s)

    {arriveSite1,                                           // State
        {playClip}, siteNoCohortsClip1,                     //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite2,                                           // State
        {playClip}, siteNoCohortsClip2,                     //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite3,                                           // State
        {playClip}, siteNoCohortsClip3,                     //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite4,                                           // State
        {playClip}, siteNoCohortsClip4,                     //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveSite5,                                           // State
        {playClip}, siteNoCohortsClip5,                     //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveBoatA,                                           // State
        {playClip}, boatCohortsClip,                        //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {diving, abandoned}},                               //   Next state(s)

    {arriveboatB,                                           // State
        {disableControls, playClip}, transitionClip,        //   Action(s) Clip
        {videoEnds, asynchTimer},                           //   Trigger(s)
        {atBoatB, abandoned}},                              //   Next state(s)

    {atBoatB,                                               // State
        {doSurvivalSequence}, noClip,                       //   Action(s) Clip
        {sequenceFinished},                                 //   Trigger(s)
        {resting}},                                         //   Next state(s)

    {outplanted,                                            // State
        {playClip}, outplantedClip,                         //   Action(s) Clip
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