#include "Chains.h"
#include "States.h"


/*****************************************************************************
 * Universal
 *****************************************************************************/
const State startingChain[] = 
{
    state_clawSideLevel,
    state_closeClaw,
    state_alignOutgoing,
    state_endOfChain
};

const State returnSameSide[] = 
{
    state_setNormalFollowDistance,
    state_setStopAtHalfRampDistance,
    state_wallFollowBack,
    state_clawMaxHeight,
    state_alignIncoming,
    state_endOfChain
};

const State returnCenter[] = 
{
    state_lineFollowBack,
    state_alignIncomingCenter,
    state_endOfChain
};

const State returnOpposite[] = 
{
    state_setNormalFollowDistance,
    state_setStopAtHalfRampDistance,
    state_wallFollowOut,
    state_alignIncoming,
    state_endOfChain
};

const State dropOff[] = 
{
    state_setStopAtBump,
    state_forward,
    state_straighten,
    state_backUpToPlaceCheese,
    state_dropOffCheese,
    state_backUpToTurn,
    state_endOfChain
};


/*****************************************************************************
 * Ramp Cheese
 *****************************************************************************/
const State getRampCheese[] = 
{
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_openClaw,
    state_clawRampLevel,
    state_turn3PointInwards90,
    state_lineFollowSlow,
    state_straighten,
    state_pickUpCheeseFromRamp,
    state_backUpToTurn,
    state_clawMaxHeight,
    state_endOfChain
};

const State rampToSameTransition[] = 
{
    state_turnInwards90,
    state_endOfChain
};

const State rampToCenterTransition[] = 
{
    state_turnOutwards90,
    state_setNormalFollowDistance,
    state_setStopAtHalfRampDistance,
    state_wallFollowOut,
    state_turnInwards90,
    state_setStopAtLine,
    state_forward,
    state_turnInwards90,
    state_endOfChain
};

const State rampToOppositeTransition[] = 
{
    state_turnOutwards90,
    state_setNormalFollowDistance,
    state_setStopAtHalfRampDistance,
    state_wallFollowOut,
    state_turnInwards90,
    state_setStopAtBump,
    state_forward,
    state_backUpToTurn,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_endOfChain
};

const State rampToHitWallTransition[] = 
{
    state_turnOutwards90,
    state_setNormalFollowDistance,
    state_setStopAtBump,
    state_wallFollowOut,
    state_clawWallLevel,
    state_turnInwards90,
    state_setCloseFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_endOfChain
};


/*****************************************************************************
 * Side Cheese
 *****************************************************************************/
const State getSideCheese[] = 
{
    state_openClaw,
    state_clawSideLevel,
    state_setSideFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_setStopAtHalfRampDistance,
    state_wallFollowOut,
    state_clawFloorLevel,
    state_setStopAtShortDistance,
    state_forward,
    state_closeClaw,
    state_shortPause,
    state_clawMaxHeight,
    state_endOfChain
};

const State sideToSameTransition[] = 
{
    state_turnInwards180,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowBack,
    state_endOfChain
};

const State sideToCenterTransition[] = 
{
    state_turnInwards90,
    state_setStopAtLine,
    state_forward,
    state_turnInwards90,
    state_endOfChain
};

const State sideToOppositeTransition[] = 
{
    state_turnInwards90,
    state_setStopAtBump,
    state_forward,
    state_backUpToTurn,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_endOfChain
};

const State sideToHitWallTransition[] = 
{
    state_setNormalFollowDistance,
    state_setStopAtBump,
    state_wallFollowOut,
    state_clawWallLevel,
    state_turnInwards90,
    state_setCloseFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_endOfChain
};

