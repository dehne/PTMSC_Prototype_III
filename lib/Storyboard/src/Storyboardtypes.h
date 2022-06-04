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
// The storyboard state values and SB_N_STATES, the number of states in the storyboard
enum sb_stateid_t : uint8_t {diving, resting, timerPop, abandoned, 
                            arriveFullSite1, arriveFullSite2, arriveFullSite3, arriveFullSite4, arriveFullSite5,
                            arriveOpenSite1, arriveOpenSite2, arriveOpenSite3, arriveOpenSite4, arriveOpenSite5,
                            fillSite1,       fillSite2,       fillSite3,       fillSite4,       fillSite5,
                            arriveSite1,     arriveSite2,     arriveSite3,     arriveSite4,     arriveSite5,
                            arriveBoatA, arriveboatB, atBoatB, outplanted, SB_N_STATES};

// The storyboard trigger values and SB_N_TRIGS, the number of triggers in the storyboard
enum sb_trigid_t : uint8_t {nullTrigger, always, asynchTimer, videoEnds, touchJoystick, 
                            nearOpenSiteCohorts1, nearOpenSiteCohorts2, nearOpenSiteCohorts3, nearOpenSiteCohorts4, nearOpenSiteCohorts5,
                            nearFullSiteCohorts1, nearFullSiteCohorts2, nearFullSiteCohorts3, nearFullSiteCohorts4, nearFullSiteCohorts5,
                            nearSiteNoCohorts1,   nearSiteNoCohorts2,   nearSiteNoCohorts3,   nearSiteNoCohorts4,   nearSiteNoCohorts5,
                            awayFromSite1,        awayFromSite2,        awayFromSite3,        awayFromSite4,        awayFromSite5,
                            videoEndsCohorts, videoEndsNoCohorts, nearBoatCohorts, nearBoatNoCohorts, pressPlaceButton, 
                            sequenceFinished, SB_N_TRIGS};

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
                            boatCohortsClip, outplantedClip, transitionClip, SB_N_CLIPS};
