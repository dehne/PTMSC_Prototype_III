# Storyboard Description

## States

### diving

This is the "normal" state the exhibit is in when she's swimming around under control of the visitor. The loop, "divingLoop", is a longish sequence showing the outplanting team in scuba gear swimming around doing their thing. The loop has no text. The main thing this state does is detect detect whether any a big list of events -- like getting near an outplanting site while carrying one or more outplanting cohorts -- has happened. The occurrence of one of these, causes a state change that depends on what event happened.

### resting

This is the state the exhibit is in after calibration has been done and later when no one has interacted with it for a while. The loop, "restingLoop", shows the same sort of swimming around as "divingLoop", but has text explaining what the exhibit is all about and inviting visitors to interact with the exhibit. The state changes to **instruct** if the joystick is touched.

### timerPop

This is the state the exhibit enters from most other states if a round of interaction has begun, but no one touches the controls for a while. Its main purpose is to disable the controls in preparation for an automated return to the dive boat. No change to whatever is displaying is made. Once the controls are disabled, the state changes to **abandoned**.

### powerUp

This is the state the exhibit enters after the hardware and software initialize themselves at power on. It displays the initial loop, "calibrateLoop". This shows a neutral underwater scene with textual instructions explaining how to calibrate the exhibit, readying it for use. Once the exhibit has been calibrated, the state changes to **resting**.

### abandoned

The exhibit enters this state from **timerPop** once the controls have been disabled. It plays the clip "AbandonedClip" that shows a neutral underwater scene with text from the diver complaining that the mission isn't being accomplished and explaining that she's decided to return to the dive boat.

### arriveFullSite<n>

These states, **arriveFullSite1** to **arriveFullSite5**, are entered from **diving** when the diver gets near the corresponding outplanting site and that site has already had a cohort planted in it. The state plays the loop "fullSite<n>Loop" showing the site as background and displaying the text saying something cheeky about having already used this site. It switches back to **diving** when the diver moves away from the site.

### arriveOpenSite<n>

These states, **arriveOpenSite1** to **arriveOpenSite5**, are entered from **diving** when the diver with one or more cohorts to outplant gets near the corresponding outplanting site that has not had a cohort planted in it. The state plays the loop "openSite<n>Loop" showing the site as background and text asking the visitor whether they'd like to do an outplanting here by pressing the Place button. If the visitor makes the diver swim away from the site, the state changes to **diving**. If the Place button is pressed, the state changes to **fillSite<n>**.

### fillSite<n>

These states, **fillSite1** to **fillSite5**, are entered from **arriveOpenSite<n>** when the Place button is pressed. It plays the clip "fillSite<n>Clip" a short clip showing the site and text indicating that the outplanting has happened. When the clip finishes, if the diver still has one or more cohorts to outplant the state changes to **atSite<n>**. If the just planted cohort is the last one the diver had, the state changes to **outAtSite<n>**.

### atSite<n>

These states, **atSite1** to **atSite5**, are entered from **fillSite<n>** or from **outAtSite<n>**. It displays "atSite<n>Loop" which displays the site scene with no text. When the diver moves away from the site, the state changes to **diving**.

### arriveSite<n>

These states, **arriveSite1** to **arriveSite5**, are entered from **diving** when the diver, carrying no cohorts to be outplanted, gets near the corresponding site. The states play "site<n>NoCohortsClip" which shows the site as background with text saying that we really need to get back to the boat since the mission is complete. When the clip finishes, the state changes to **atSite<n>**.

### outAtSite<n>

These states, **outAtStie1** to **outAtSite5**, are entered from **fillSite<n>** when the last cohort the diver is carrying has just been paced. The state plays "outAtSite<n>Clip", which shows the site as background and text saying that the last cohort was just outplanted, so now it's time to go back to the boat. When the clip finishes, the state changes to **atSite<n>**.

### arriveBoatA

This state is entered from **diving** when the diver is carrying one or more cohorts and comes near the dive boat. The clip "boatCohortsLoop" is played. This shows the surface of the water near the dive boat and has text asking why we're back at the dive boat when we still have cohorts to plant. It exits to **diving** when the diver swims away from the boat.

### arriveboatB

This state is entered from **diving** when the diver gets near the dive boat with no cohorts left to plant. The controls are disabled and "transitionClip" is played. This clip shows the baby abalone growing in time-lapse for a year. When the clip ends, things move to the state **atBoatB**.

### atBoatB

This state has yet to be implemented. For now it's stubbed out to just log its passing and then transition to **resting**.

### instruct

This state is entered from **resting** when a visitor touches the joystick, It plays the clip "instructLoop" which uses the same background "restingLoop" uses, but the text gives information about abalone restoration and how to operate the exhibit. When the diver moves away from the boat, we transition to **diving**.