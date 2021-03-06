#include "Chains.h"
#include "States.h"


/*****************************************************************************
 * Universal
 *****************************************************************************/
const State startingChain[] = 
{
    state_clawMaxHeight,
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
    state_alignIncoming,
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
    //state_straighten,
    //state_backUpToPlaceCheese,
    state_dropOffCheese,
    state_clawMaxHeight,
    state_backUpToTurn,
    state_endOfChain
};


/*****************************************************************************
 * Ramp Cheese
 *****************************************************************************/
const State getRampCheese[] = 
{
    state_setLongHoldoff,
    state_setNormalFollowDistance,
    state_setStopAtLine,
    state_wallFollowOut,
    state_openClaw,
    state_setStopAtShortDistance,
    state_forward,
    state_turn3PointInwards90,
    state_clawRampLevel,
    state_setStopAtBump,
    state_forward,
    //state_straighten,
    state_pickUpCheeseFromRamp,
    state_clawMaxHeight,
    state_backUpToTurn,
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
    state_clawSideLevel,
    state_setStopAtBump,
    state_forward,
    state_backUpToTurn,
    state_clawMaxHeight,
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
    state_backUpShortToTurn,
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
    state_setStopAtMediumDistance,
    state_wallFollowOut,
    state_clawFloorLevel,
    state_setStopAtShortDistance,
    state_forward,
    state_pickUpCheeseFromFloor,
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
    state_setNormalFollowDistance,
    state_setStopAtBump,
    state_wallFollowOut,
    state_backUpShortToTurn,
    state_clawWallLevel,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnInwards90,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnOutwards90,
    state_turnOutwards90,
    state_setNormalFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnInwards90,
    state_turnInwards90,
    state_setNormalFollowDistance,
    state_wallFollowOut,
    state_backUpToTurn,
    state_turnOutwards90,
    state_turnOutwards90,
    state_endOfChain
};

const State sideToOppositeTransition[] = 
{
    state_turnInwards90,
    state_clawSideLevel,
    state_setStopAtBump,
    state_forward,
    state_backUpToTurn,
    state_clawMaxHeight,
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
    state_backUpShortToTurn,
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