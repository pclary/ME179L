#ifndef STATES_H
#define STATES_H


enum State
{
    // End of chain marker
    state_endOfChain = 0,

    // Program entry
    state_enterActions = 1,
    
    // Alignment
    state_alignOutgoing = 10,
    state_alignOutgoingNearest,
    state_alignOutgoingNear,
    state_alignOutgoingStarting,
    state_alignOutgoingFar,
    state_alignOutgoingFurthest,
    state_alignOutgoingNearCenter,
    state_alignOutgoingFarCenter,
    
    state_alignIncoming = 20,
    state_alignIncomingNearest,
    state_alignIncomingNear,
    state_alignIncomingStarting,
    state_alignIncomingFar,
    state_alignIncomingFurthest,
    state_alignIncomingNearCenter,
    state_alignIncomingFarCenter,
    
    state_alignToDropOff = 30,
    state_alignToDropOffNear,
    state_alignToDropOffFar,
    
    // Line following
    state_lineFollowBack = 40,
    state_lineFollowSlow,
    
    // Movement with settable parameters
    state_wallFollowOut = 50,
    state_wallFollowBack,
    state_forward,
    
    // Movement options
    state_setCloseFollowDistance = 60,
    state_setSideFollowDistance,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_setStopAtBump,
    state_setStopAtHalfRampDistance,
    state_setStopAtShortDistance,
    state_setStopAtMediumDistance,
    state_setLongHoldoff,
    state_setNoHoldoff,
    
    // Dropping off and picking up cheese
    state_straighten = 70,
    state_pickUpCheeseFromWall,
    state_pickUpCheeseFromRamp,
    state_pickUpCheeseFromFloor,
    state_dropOffCheese,
    
    // Claw movements
    state_openClaw = 80,
    state_closeClaw,
    state_clawMaxHeight,
    state_clawWallLevel,
    state_clawRampLevel,
    state_clawSideLevel,
    state_clawFloorLevel,
    
    // Miscellaneous movement
    state_shortPause = 90,
    state_backUpToTurn,
    state_backUpShortToTurn,
    state_backUpToPlaceCheese,
    state_turnInwards90,
    state_turn3PointInwards90,
    state_turnOutwards90,
    state_turnInwards180,
    state_turnOutwards180,
};

#endif // STATES_H