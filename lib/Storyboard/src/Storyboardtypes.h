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
 * Storyboard V0.2, June 2022
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
// The storyboard state values and SB_N_STATES, the number of states in the storyboard
enum sb_stateid_t : uint8_t {diving, resting, timerPop, powerUp, abandoned, 
                            arriveFullSite1, arriveFullSite2, arriveFullSite3, arriveFullSite4, arriveFullSite5,
                            arriveOpenSite1, arriveOpenSite2, arriveOpenSite3, arriveOpenSite4, arriveOpenSite5,
                            fillSite1,       fillSite2,       fillSite3,       fillSite4,       fillSite5,
                            arriveSite1,     arriveSite2,     arriveSite3,     arriveSite4,     arriveSite5,
                            arriveBoatA, arriveboatB, atBoatB, outplanted, instruct, SB_N_STATES};

// The storyboard trigger values and SB_N_TRIGS, the number of triggers in the storyboard
enum sb_trigid_t : uint8_t {nullTrigger, always, asynchTimer, videoEnds, touchJoystick, 
                            nearOpenSiteCohorts1, nearOpenSiteCohorts2, nearOpenSiteCohorts3, nearOpenSiteCohorts4, nearOpenSiteCohorts5,
                            nearFullSiteCohorts1, nearFullSiteCohorts2, nearFullSiteCohorts3, nearFullSiteCohorts4, nearFullSiteCohorts5,
                            nearSiteNoCohorts1,   nearSiteNoCohorts2,   nearSiteNoCohorts3,   nearSiteNoCohorts4,   nearSiteNoCohorts5,
                            awayFromSite1,        awayFromSite2,        awayFromSite3,        awayFromSite4,        awayFromSite5,
                            videoEndsCohorts, videoEndsNoCohorts, nearBoatCohorts, nearBoatNoCohorts, pressPlaceButton, 
                            sequenceFinished, calibrated, SB_N_TRIGS};

// The storyboard action values
enum sb_actid_t : uint8_t  {nullAction, setLoop, playClip, prepareNew, disableControls, 
                            deposit1, deposit2, deposit3, deposit4, deposit5,
                            doSurvivalSequence, SB_N_ACTS};

// The storyboard clip values
enum sb_clipid_t : uint8_t {noClip, divingLoop, restingLoop, abandonedClip, 
                            fullSiteClip1,      fullSiteClip2,      fullSiteClip3,      fullSiteClip4,      fullSiteClip5,
                            siteNoCohortsClip1, siteNoCohortsClip2, siteNoCohortsClip3, siteNoCohortsClip4, siteNoCohortsClip5,
                            openSiteLoop1,      openSiteLoop2,      openSiteLoop3,      openSiteLoop4,      openSiteLoop5,
                            fillSiteClip1,      fillSiteClip2,      fillSiteClip3,      fillSiteClip4,      fillSiteClip5,
                            boatCohortsClip, outplantedClip, transitionClip, calibrateLoop, instructClip, SB_N_CLIPS};

// The type an "action handler" -- a function invoked to carry out a specific action -- must have. The id of the state which 
// caused the invocation, the id of the action invoked, and its associated clip id are all pessed. The action handler should 
// carry out the action and return.
typedef void(*sb_actionhandler)(sb_stateid_t stateId, sb_actid_t actionId, sb_clipid_t clipId);

// The type a "trigger handler" -- a function invoked to say whether a particular trigger has occurred -- must have. The ids 
// of the state and of the trigger that caused the invocation are both passed. The trigger handler evaluates the situation and 
// returns true if the trigger has occurred, false if not.
typedef bool(*sb_triggerhandler_t)(sb_stateid_t stateId, sb_trigid_t triggerId);

// The definiton of a state in the state machine
#define SB_MAX_TRIGS    (18)                                // The maximum number of triggers a state can have (adjust as needed)
#define SB_MAX_ACTS     (2)                                 // The maximum number of actions a state can have (adjust as needed)
struct sb_state_t {                                         // The representation of a state
    sb_stateid_t id;                                        //   Which state this is
    sb_actid_t actionId[SB_MAX_ACTS];                       //   The action(s) to take, in sequence
    sb_clipid_t clipId[SB_MAX_ACTS];                        //   The associated video clip; noClip if none
    sb_trigid_t trigId[SB_MAX_TRIGS];                       //   The trigger(s) that cause a state change in precedence order
    sb_stateid_t nextStateId[SB_MAX_TRIGS];                 //   The id of the new state upon occurance of corresponding trigger
};
