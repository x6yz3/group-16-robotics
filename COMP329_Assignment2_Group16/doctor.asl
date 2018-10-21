// doctor agant

/* Initial beliefs */

/* Initial goal */

!checkPsbLoc(slot). 

/* Plans */

// find where robot is
+!checkPsbLoc(slot) : not onlyLocation
   <- trimLoc(slot);
      !!checkPsbLoc(slot).
+!checkPsbLoc(slot) : onlyLocation
   <- !!findVictim(slot).
   
// find victim if have not found all 3 victims 
+!findVictim(slot) : not noLeftVic
   <- pickNextVic(slot);
      findNearestRoute(slot);
      !!goToNextVic(slot).
+!findVictim(slot) : noLeftVic
   <- deleteScoutAndPsbVic(slot);
      .print(" * FIND ALL VICTIMS").

// if not at a victim, go to the nearest victim
+!goToNextVic(slot) : not atVic
   <- goToNearestVic(slot);
      !!goToNextVic(slot).
+!goToNextVic(slot) : atVic
   <- !!checkVic(slot).
   
// verify current victim
+!checkVic(slot) : true
   <- checkIfVic(slot);
      checkLeftVic(slot);
	  !!findVictim(slot).
   
   
   
