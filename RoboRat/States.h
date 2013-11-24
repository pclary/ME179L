#ifndef STATES_H
#define STATES_H


enum State
{
    // End of chain marker
    state_endOfChain = 0,

    // Program entry
    state_enterActions,
    
    // Alignment
    state_alignOutgoing,
    state_alignOutgoingCenter,
    state_alignIncoming,
    state_alignIncomingCenter,
    state_alignToDropOff,
    
    // Line following
    state_lineFollowBack,
    state_lineFollowSlow,
    
    // Movement with settable parameters
    state_wallFollowOut,
    state_wallFollowBack,
    state_forward,
    
    // Movement options
    state_setCloseFollowDistance,
    state_setSideFollowDistance,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_setStopAtBump,
    state_setStopAtHalfRampDistance,
    state_setStopAtShortDistance,
    
    // Dropping off and picking up cheese
    state_straighten,
    state_pickUpCheeseFromWall,
    state_pickUpCheeseFromRamp,
    state_pickUpCheeseFromFloor,
    state_dropOffCheese,
    
    // Claw movements
    state_openClaw,
    state_closeClaw,
    state_clawMaxHeight,
    state_clawWallLevel,
    state_clawRampLevel,
    state_clawSideLevel,
    state_clawFloorLevel,
    
    // Miscellaneous movement
    state_shortPause,
    state_backUpToTurn,
    state_backUpToPlaceCheese,
    state_turnInwards90,
    state_turn3PointInwards90,
    state_turnOutwards90,
    state_turnInwards180,
    state_turnOutwards180,
};

#endif // STATES_H