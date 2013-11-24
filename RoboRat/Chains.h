#ifndef CHAINS_H
#define CHAINS_H

#include "States.h"

extern const State startingChain[];
extern const State returnSameSide[];
extern const State returnCenter[];
extern const State returnOpposite[];
extern const State dropOff[];

extern const State getRampCheese[];
extern const State rampToSameTransition[];
extern const State rampToCenterTransition[];
extern const State rampToOppositeTransition[];
extern const State rampToHitWallTransition[];

extern const State getSideCheese[];
extern const State sideToSameTransition[];
extern const State sideToCenterTransition[];
extern const State sideToOppositeTransition[];
extern const State sideToHitWallTransition[];

#endif // CHAINS_H