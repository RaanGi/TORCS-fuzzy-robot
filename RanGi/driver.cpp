#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 15.0f/180.0f*PI;	// [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float Driver::UNSTUCK_TIME_LIMIT = 2.0f;				// [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 5.0f;				// [m/s] Below this speed we consider being stuck.
const float Driver::MIN_UNSTUCK_DIST = 3.0f;				// [m] If we are closer to the middle we assume to be not stuck.
const float Driver::G = 9.81f;								// [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;				// [m/s] Margin reduce oscillation of brake/acceleration.
const float Driver::SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.
const float Driver::ABS_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::ABS_RANGE = 5.0f;						// [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;					// [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;						// [m/s] range [0..10]
const float Driver::LOOKAHEAD_CONST = 17.0f;				// [m]
const float Driver::LOOKAHEAD_ACCEL_BRAKE_FACTOR = 0.02f;	// [-]
const float Driver::LOOKAHEAD_STEER_FACTOR = 0.005f;		// [-]
const float Driver::WIDTHDIV = 3.0f;						// [-] Defines the percentage of the track to use (2/WIDTHDIV).
const float Driver::SIDECOLL_MARGIN = 1.5f;					// [m] Distance between car centers to avoid side collisions.
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;			// [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 5.0f;			// [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 6.0f;					// [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;				// [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;							// [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 84.0f;						// [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.000725f;			// [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;					// [m/s]
const float Driver::CENTERDIV = 0.1f;						// [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 200.0f;					// [m] How far to look, terminate while loops.
const float Driver::MAX_INC_FACTOR = 5.0f;					// [m] Increment faster if speed is slow [1.0..10.0].
const float Driver::CATCH_FACTOR = 10.0f;					// [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;			// [s] Time to apply full clutch.



// Static variables.
Cardata *Driver::cardata = NULL;
double Driver::currentsimtime;

Driver::Driver(int index)
{
	INDEX = index;
}

Driver::~Driver()
{
	delete opponents;
	delete pit;
	delete strategy;
	delete learn;
	if (cardata != NULL) {
            delete cardata;
            cardata = NULL;
	}
}

// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;
	char buffer[256];
	sprintf(buffer, "drivers/RanGi/%d/default.xml", INDEX);
    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);

	// Create a pitstop strategy object
	strategy = new SimpleStrategy();

	// Set fuel for the race
	strategy->setFuelAtRaceStart(t, carParmHandle, s, INDEX);

	// Load and set parameters.
	MU_FACTOR = GfParmGetNum(*carParmHandle, RANGI_SECT_PRIV, RANGI_ATT_MUFACTOR, (char*)NULL, 1.5f);
}

// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
    float deltaTime = (float) RCM_MAX_DT_ROBOTS;
	MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/deltaTime);
	OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED*deltaTime;
	MAX_ABS_OFFSET = (car->_trkPos.seg->width / 2.75f);
	this->car = car;
	CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0f);
	myoffset = 0.0f;	// BORRAR

	// Init variables.
	oldOffset = 0.0f;
	nextTurnId = car->_trkPos.seg->id;
	prevTurnType = TR_STR;
	overtakeMove = 0;
	letPassOpp = 0;
	stuck = 0;
	alone = 1;
	speedRedFactor = 1.0f;

	lastFuelChecked = car->_fuel;
	lastDamageChecked = car->_dammage;
	
	initCa();
	initCw();
	initTireMu();

    fc = new FuzzyControl();
    fc->startFuzzyEngines(INDEX);

	// Create just one instance of cardata shared by all drivers.
	if (cardata == NULL) {
            cardata = new Cardata(s);
	}
	mycardata = cardata->findCar(car);
	currentsimtime = s->currentTime;

	// initialize the list of opponents.
	opponents = new Opponents(s, this, cardata);
	opponent = opponents->getOpponentPtr();

	learn = new RadiusLearn(track, s, INDEX);

    // Initialize radius of segments.
	radius = learn->getRadiusArray();
	if(!(learn->learned()))
	{
		computeRadius(radius);
	}

	// Create the pit object
	pit = new Pit(s, this);
}

// Drive during race.
void Driver::drive(tSituation *s)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	update(s);

	//pit->setPitstop(true);

	if (isStuck() && !pit->getInPit()) {
		car->_steerCmd = -getCarAngle() / car->_steerLock;
		car->_gearCmd = -1;		// Reverse gear.
		car->_accelCmd = filterTCL(1.0f);	// 100% accelerator pedal.
		car->_brakeCmd = 0.0f;	// No brakes.
		car->_clutchCmd = 0.0f;	// Full clutch (gearbox connected with engine).
	} else {
		letPassOpp = checkLetPass();
		car->_steerCmd = filterSColl(getSteer());
		car->_gearCmd = getGear();
		getAccelBrake(&(car->_accelCmd), &(car->_brakeCmd));
  		}
		car->_clutchCmd = getClutch();
}

// Set pitstop commands.
int Driver::pitCommand(tSituation *s)
{
	car->_pitRepair = strategy->pitRepair(car, s);
	car->_pitFuel = strategy->pitRefuel(car, s);
	// This should be the only place where the pit stop is set to false!
	pit->setPitstop(false);
	return ROB_PIT_IM; // return immediately.
}

// End of the current race.
void Driver::endRace(tSituation *s)
{
	
}

void Driver::shutDown()
{
	fc->terminateFuzzyEngines();
}

/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/

void Driver::computeRadius(float *radius)
{
	float lastturnarc = 0.0f;
	int lastsegtype = TR_STR;

	tTrackSeg *currentseg, *startseg = track->seg;
	currentseg = startseg;

	do {
		if (currentseg->type == TR_STR) {
			lastsegtype = TR_STR;
			radius[currentseg->id] = FLT_MAX;
		} else {
			if (currentseg->type != lastsegtype) {
				float arc = 0.0f;
				tTrackSeg *s = currentseg;
				lastsegtype = currentseg->type;

				while (s->type == lastsegtype && arc < PI/2.0f) {
					arc += s->arc;
					s = s->next;
				}
				lastturnarc = arc/(PI/2.0f);
			}
			radius[currentseg->id] = (currentseg->radius + currentseg->width/2.0)/lastturnarc;
		}
		currentseg = currentseg->next;
	} while (currentseg != startseg);

}

void Driver::computeStraight()
{
	float offset = 30.0f;		// [ m ] -> Amount substracted from straight length

	if(car->_trkPos.seg->id > straightInfo.endSegId || car->_laps > straightInfo.lap)
	{
		tTrackSeg *seg = car->_trkPos.seg;
		int done = 0;
		int prevTurn, nextTurn;
		memset((void *) &straightInfo, 0, sizeof(straightInfo));
		while (done < 2)
		{
			if(seg->type != TR_STR && done == 0)
			{
				prevTurn = seg->type;
			}
			else if(seg->type == TR_STR && done == 0)
			{
				straightInfo.startSegId = seg->id;
				done++;
			}
			else if(seg->next->type != TR_STR)
			{
				straightInfo.endSegId = seg->id;
				nextTurn = seg->next->type;
				done++;
			}
			if(seg->type == TR_STR) straightInfo.length += seg->length;
			seg = seg->next;
		}
		straightInfo.lap = car->_laps;
		if(straightInfo.endSegId >= straightInfo.startSegId)
		{
			straightInfo.segNum = straightInfo.endSegId - straightInfo.startSegId;
		}
		else
		{
			straightInfo.segNum = track->nseg - straightInfo.endSegId + straightInfo.startSegId;
		}
		
	}
}

// Compute the allowed speed on a segment.
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
	if(segment->type == TR_STR)
	{
		return 400.0f;
	}
	float mu;
	if(getSpeed() * 3.6 < 200.0f)	// 132.28f -> speed when 1.75 - currentspeedsqr * 0.0001f = 0
	{
		mu = segment->surface->kFriction*TIREMU*(MU_FACTOR + 2.4f - currentspeedsqr * 0.0008f);
	}
	else if(getSpeed() * 3.6 > 220.0f)
	{
		mu = segment->surface->kFriction*TIREMU*(MU_FACTOR - currentspeedsqr * 0.0000175f);
	}	
	else
	{
		mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
	}
	
	float r = radius[segment->id];
	float allowedSpeed;
    
    allowedSpeed = MIN(400.0f, sqrt((mu*G*r)/(1.0f - MIN(1.0f, r*CA*mu/mass))));
    
	return allowedSpeed * speedRedFactor;
}

// Compute the length to the end of the segment.
float Driver::getDistToSegEnd()
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

void Driver::getAccelBrake(float *accel, float *brake)
{
    double cSpeed, speedError;
    tTrackSeg *segptr = car->_trkPos.seg;
    float mu = segptr->surface->kFriction;
    float lookahead = LOOKAHEAD_CONST + currentspeedsqr * LOOKAHEAD_ACCEL_BRAKE_FACTOR;
	float allowedSpeed, wheelSpin, wheelSlip;
	
	lookahead = overtakeMove ? lookahead * 0.875f : lookahead;
    segptr = getLookaheadSeg(lookahead);
	int segCount = 0;
	tTrackSeg *segArray[] = {segptr, car->_trkPos.seg->next->next};
	float weights[] = {0.9f, 0.1f};
	allowedSpeed = 0.0f;
	while(segCount < 2)
	{
		allowedSpeed += getAllowedSpeed(segArray[segCount]) * weights[segCount];
		segCount++;
	}
    
    cSpeed = getSpeed() * 3.6;
    speedError = allowedSpeed - cSpeed;
	//std::cout << "AllowedSpeed: " << allowedSpeed << "\n";
    if(speedError > 400) speedError = 400;
    else if(speedError < -400) speedError = -400;

	wheelSpin = speedError >= 0.0 ? 0.0 : FLT_MAX;
	for(int i = 0; i < 4; i++)
	{
		wheelSpin = speedError >= 0.0 ? 
						MAX(car->_wheelSpinVel(i) * car->_wheelRadius(i), wheelSpin) :
						MIN(car->_wheelSpinVel(i) * car->_wheelRadius(i), wheelSpin);
	}

	if(wheelSpin > getSpeed() && getSpeed() > 3)
	{
		wheelSlip = (wheelSpin - getSpeed()) / wheelSpin;
		//std::cout << "SLIP_TCL: " << wheelSlip << "\n";
	}
	else if(wheelSpin < getSpeed() && getSpeed() > 3)
	{
		wheelSlip = -(getSpeed() - wheelSpin) / getSpeed();
		//std::cout << "SLIP_ABS: " << wheelSlip << "\n";
	}
	else
	{
		wheelSlip = 0;
	}

    fc->setSpeedError(speedError);
	fc->setSlip(wheelSlip);
    fc->setState(ACCEL_BRAKE);
    fc->processFuzzyData();
    *brake = filterBColl(filterBPit(fc->getBrake()));
	if(*brake == 0.0f) 
	{
		*accel = (!letPassOpp ? filterTrk(fc->getAccel()) : filterTrk(fc->getAccel()) * 0.3f);
	}
	else
	{
		*accel = 0.0f;
	}
	
}

// Compute gear.
int Driver::getGear()
{
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}

// Compute steer value.
float Driver::getSteer()
{
	float targetAngle;
	v2d target = getTargetPoint();

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}

float Driver::getOffset(float lookahead)
{
	tTrackSeg *segptr;
	tTrackSeg *nextTurnSeg = car->_trkPos.seg;
	float posMaxOffset = MAX_ABS_OFFSET;
	float negMaxOffset = -MAX_ABS_OFFSET;
	float currentSpeed = getSpeed() * 3.6f;
	
	computeStraight();
	segptr = getLookaheadSeg(lookahead);

	{
		tTrackSeg *aux = car->_trkPos.seg;
		if(segptr->type == TR_STR && car->_trkPos.seg->type != TR_STR)
		{
			while (aux->id != straightInfo.startSegId)
			{
				aux = aux->next;
			}
			nextTurnSeg = aux->prev;
		} 
		else if(segptr->type == TR_STR && car->_trkPos.seg->type == TR_STR)
		{
			while (aux->id != straightInfo.endSegId)
			{
				aux = aux->next;
			}
			nextTurnSeg = aux->next;
		} 
		else nextTurnSeg = segptr;
	}

	{
		Opponent* opp = checkOvertake();
		Opponent* oppSide[] = {checkSColl(1), checkSColl(-1)};
		float oppWidth, oppToMiddle, oppOffset;
		int sign;

		if(letPassOpp)
		{
			if(segptr->type == TR_STR)
			{
				if(nextTurnSeg->type == TR_RGT)
				{
					posMaxOffset = negMaxOffset * 0.75;
				}
				else
				{
					negMaxOffset = posMaxOffset * 0.75f;
				}
			}
			else if(segptr->type == TR_RGT)
			{
				negMaxOffset = posMaxOffset * 0.5f;
			}
			else
			{
				posMaxOffset = negMaxOffset * 0.5;
			}
			
		}

		for(int i = 0; i < 2; i++)
		{
			if(oppSide[i] != NULL)
			{
				oppWidth = oppSide[i]->getWidth() * 1.5f;
				oppToMiddle = oppSide[i]->getCarPtr()->_trkPos.toMiddle;
				sign = (oppToMiddle - car->_trkPos.toMiddle) > 0.0f ? 1 : -1;
				oppOffset = oppToMiddle - oppWidth * sign;
				if(sign == -1)
				{
					negMaxOffset = oppOffset;
					posMaxOffset *= 1.1f;
				} 
				else 
				{
					posMaxOffset = oppOffset;
					negMaxOffset *= 1.1f;
				}
			}
		}
		if(opp != NULL && oppSide[0] == NULL && oppSide[1] == NULL)
		{
			if(opp->getState() & OPP_FRONT)
			{
				oppWidth = opp->getWidth() * 1.2f;
				oppToMiddle = opp->getCarPtr()->_trkPos.toMiddle;
				sign = (oppToMiddle - car->_trkPos.toMiddle) > 0.0f ? 1 : -1;
				oppOffset = oppToMiddle - oppWidth * sign;
				//std::cout << "WIDTH: " << mycardata->getWidthOnTrack() << "\n";
				// Get the slipstream
				if(opp->getDistance() > 2.5f && !overtakeMove)
				{
					if(opp->getCarPtr()->_trkPos.seg->id < straightInfo.endSegId &&
					segptr->type == TR_STR && car->_trkPos.seg->type == TR_STR)
					{
						posMaxOffset = MAX(MIN(oppToMiddle, posMaxOffset), negMaxOffset) + 0.01f;
						negMaxOffset = MIN(MAX(oppToMiddle, negMaxOffset), posMaxOffset) - 0.01f;
					}
				}
				else
				{
					if(opp->getDistance() < 4.0f && !overtakeMove)
					{
						overtakeMove = 1;
					}	
					else if(opp->getDistance() > 4.0f && overtakeMove)
					{
						overtakeMove = 0;
					}
					
					if(nextTurnSeg->type == TR_RGT && (oppToMiddle - oppWidth / 2.0f) - negMaxOffset > mycardata->getWidthOnTrack() &&
						car->_trkPos.toMiddle - oppToMiddle < mycardata->getWidthOnTrack())
					{
						posMaxOffset = MIN((oppOffset + negMaxOffset) / 2.0f, posMaxOffset);				
					}
					else if(nextTurnSeg->type == TR_LFT && posMaxOffset - (oppToMiddle + oppWidth / 2.0f) > mycardata->getWidthOnTrack() &&
						oppToMiddle - car->_trkPos.toMiddle < mycardata->getWidthOnTrack())
					{
						negMaxOffset = MAX((oppOffset + posMaxOffset) / 2.0f, negMaxOffset);						
					}
					else if(car->_trkPos.seg->type == TR_STR)
					{
						if(nextTurnSeg->type == TR_RGT) negMaxOffset = MAX((oppOffset + posMaxOffset) / 2.0f, negMaxOffset);
						else posMaxOffset = MIN((oppOffset + negMaxOffset) / 2.0f, posMaxOffset);	
					}
					else
					{
						overtakeMove = 0;
					}
					
				}
			}
		}
		else if(overtakeMove)
		{
			overtakeMove = 0;
		}
		if(overtakeMove)	speedRedFactor = 1.0f - MIN(MIN(fabs(posMaxOffset - car->_trkPos.toMiddle), fabs(negMaxOffset - car->_trkPos.toMiddle)), 0.2f);
		else 	speedRedFactor = MAX((posMaxOffset + fabs(negMaxOffset)) / (MAX_ABS_OFFSET * 2.0f), 0.8f);
	}

	tTrackSeg *segArray[] = {segptr, segptr->prev->prev};
	float weights[] = {0.7f, 0.3f};
	float offset = 0.0f;

	fc->setSpeed(getSpeed() * 3.6f);
	fc->setCarToLimit(nextTurnSeg->type == TR_RGT ? 
						(MIN(0.0f, car->_trkPos.toMiddle) / negMaxOffset) : 
						(MAX(0.0f, car->_trkPos.toMiddle) / posMaxOffset));
	fc->setState(STEER);
	for(int segCount = 0; segCount < 2; segCount++)
	{
		fc->processFuzzyData();
		if(segArray[segCount]->type == TR_STR)
		{
			if((nextTurnSeg->type == TR_RGT && car->_trkPos.seg->type == TR_STR && straightInfo.segNum > 2) ||
				(car->_trkPos.seg->type == TR_RGT))
				{
					offset += fc->getOffset() * posMaxOffset * weights[segCount];
				}
			else if((nextTurnSeg->type == TR_LFT && car->_trkPos.seg->type == TR_STR && straightInfo.segNum > 2) ||
				(car->_trkPos.seg->type == TR_LFT)) 
				{
					offset += fc->getOffset() * negMaxOffset * weights[segCount];
				}
		}
		else if(segArray[segCount]->type == TR_RGT)
		{
			offset += fc->getOffset() * negMaxOffset * weights[segCount] * 0.75f;
		}
		else if(segArray[segCount]->type == TR_LFT) 
		{
			offset += fc->getOffset() * posMaxOffset * weights[segCount] * 0.75f;
		}
	}

	if(offset > posMaxOffset) offset = posMaxOffset;
	if(offset < negMaxOffset) offset = negMaxOffset;

	/*std::cout << "BIAS: " << offset << "\tNEXT_TURN: " << (nextTurnSeg->type == TR_RGT ? "RIGHT" : "LEFT") 
	<< "\tRADIUS: " << radius[car->_trkPos.seg->id] << "\tLIMITS: " << negMaxOffset << " " << posMaxOffset << "\n";*/

	return offset;

	//std::cout << "BIAS: " << offset << "\tTM: " << car->_trkPos.toMiddle << "\n";	
	
	//std::cout << "SC: " << segCount << "\tSID: " << segptr->id << "\tCID: " << car->_trkPos.seg->id << "\n";
}

float Driver::updateOffset(float proportion, float lookahead)
{
	float offset = getOffset(lookahead);
	float oldOffsetProportion = 1.0f - proportion;
	float specialProp = proportion / 4.0f;
	float specialOldOffsetProp = 1.0f - specialProp;
	float length = getDistToSegEnd();
	tTrackSeg *seg = car->_trkPos.seg;

	// Search for the segment containing the target point.
	while (length < lookahead)
	{
		seg = seg->next;
		length += seg->length;
	}

	length = lookahead - length + seg->length;
	float fromstart = seg->lgfromstart;
	fromstart += length;

	if(pit->getInPit())
	{
		offset = pit->getPitOffset(offset, fromstart);
	}
	else if(seg->type == TR_STR && car->_trkPos.seg->type == TR_STR)
	{
		offset = pit->getPitOffset(offset, fromstart) * specialProp + oldOffset * specialOldOffsetProp;
	}
	else if(seg->type == TR_STR && car->_trkPos.seg->type != TR_STR)
	{
		offset = pit->getPitOffset(offset, fromstart) * specialProp + oldOffset * specialOldOffsetProp;
	}
	else
	{
		offset = pit->getPitOffset(offset, fromstart) * proportion + oldOffset * oldOffsetProportion;
	}
	if(!isnan(offset)) oldOffset = offset;

	return offset;
}

// Compute the clutch value.
float Driver::getClutch()
{
	if (car->_gear > 1) {
		clutchtime = 0.0f;
		return 0.0f;
	} else {
		float drpm = car->_enginerpm - car->_enginerpmRedLine/2.0f;
		clutchtime = MIN(CLUTCH_FULL_MAX_TIME, clutchtime);
		float clutcht = (CLUTCH_FULL_MAX_TIME - clutchtime)/CLUTCH_FULL_MAX_TIME;
		if (car->_gear == 1 && car->_accelCmd > 0.0f) {
			clutchtime += (float) RCM_MAX_DT_ROBOTS;
		}

		if (drpm > 0) {
			float speedr;
			if (car->_gearCmd == 1) {
				// Compute corresponding speed to engine rpm.
				float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
				float wr = car->_wheelRadius(2);
				speedr = (CLUTCH_SPEED + MAX(0.0f, car->_speed_x))/fabs(wr*omega);
				float clutchr = MAX(0.0f, (1.0f - speedr*2.0f*drpm/car->_enginerpmRedLine));
				return MIN(clutcht, clutchr);
			} else {
				// For the reverse gear.
				clutchtime = 0.0f;
				return 0.0f;
			}
		} else {
			return clutcht;
		}
	}
}

v2d Driver::getTargetPoint()
{
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead;
	float length = getDistToSegEnd();
	float offset;

	if (pit->getInPit()) 
	{
		// To stop in the pit we need special lookahead values.
		if (currentspeedsqr > pit->getSpeedlimitSqr()) {
			lookahead = PIT_LOOKAHEAD + car->_speed_x*LOOKAHEAD_STEER_FACTOR;
		} else {
			lookahead = PIT_LOOKAHEAD;
		}
		offset = updateOffset(0.0f, lookahead);
	}
	else 
	{
		// Usual lookahead.
		lookahead = LOOKAHEAD_CONST + currentspeedsqr * LOOKAHEAD_STEER_FACTOR;
		// Prevent "snap back" of lookahead on harsh braking.
		float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
		if (lookahead < cmplookahead) {
			lookahead = cmplookahead;
		}
		offset = updateOffset(0.1f, lookahead);
		//std::cout << "N_LH: " << lookahead << "\t";
	}

	oldlookahead = lookahead;

	// Search for the segment containing the target point.
	while (length < lookahead)
	{
		seg = seg->next;
		length += seg->length;
	}
	length = lookahead - length + seg->length;

	// Compute the target point.
	
	//std::cout << "BIAS: " << offset << "\n";

	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0f;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0f;

	if ( seg->type == TR_STR) {
		v2d d, n;
		n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
		n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
		n.normalize();
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length + offset*n;
	} else {
		v2d c, n;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
		arc = arc*arcsign;
		s = s.rotate(c, arc);

		n = c - s;
		n.normalize();
		return s + arcsign*offset*n;
	}
}

// Update my private data every timestep.
void Driver::update(tSituation *s)
{
	// Update global car data (shared by all instances) just once per timestep.
	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
        cardata->update();
		if(lastFuelChecked < car->_fuel) lastFuelChecked = car->_fuel;
		if(lastDamageChecked > car->_dammage) lastDamageChecked = car->_dammage;
	}

	// Update the local data rest.
	speedangle = getCarAngle() - atan2(car->_speed_Y, car->_speed_X);
	NORM_PI_PI(speedangle);
	mass = CARMASS + car->_fuel;
	currentspeedsqr = car->_speed_x*car->_speed_x;
	opponents->update(s, this);
	strategy->update(car, s);
	if(!pit->getPitstop() && (car->_fuel + 1.0f < lastFuelChecked || car->_dammage > lastDamageChecked))
	{
		pit->setPitstop(strategy->needPitstop(car, s, fc));
		lastFuelChecked = car->_fuel;
		lastDamageChecked = car->_dammage;
	}
	if(car->_fuel > lastFuelChecked || car->_dammage < lastDamageChecked)
	{
		lastFuelChecked = car->_fuel;
		lastDamageChecked = car->_dammage;
	}
	pit->update();
	alone = isAlone();
	learn->update(s, track, car, getLookaheadSeg(LOOKAHEAD_CONST + currentspeedsqr * LOOKAHEAD_STEER_FACTOR), alone, pit->getInPit());
}

int Driver::isAlone()
{
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if ((opponent[i].getState() & (OPP_COLL | OPP_LETPASS | OPP_SIDE)) && overtakeMove) {
			return 0;	// Not alone.
		}
	}
	return 1;	// Alone.
}

// Returns the segment that is at a given distance (lookahead) from the car segment.
tTrackSeg* Driver::getLookaheadSeg(float lookahead)
{
	float length = getDistToSegEnd();
	tTrackSeg *seg = car->_trkPos.seg;
	while (length < lookahead)
	{
		seg = seg->next;
		length += seg->length;
	}
	return seg;
}

// Check if I'm stuck.
bool Driver::isStuck()
{
	if (fabs(getCarAngle()) > MAX_UNSTUCK_ANGLE &&
		car->_speed_x < MAX_UNSTUCK_SPEED &&
		fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
		if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*getCarAngle() < 0.0) {
			return true;
		} else {
			stuck++;
			return false;
		}
	} else {
		stuck = 0;
		return false;
	}
}

// Compute aerodynamic downforce coefficient CA.
void Driver::initCa()
{
	const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
	float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
	float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
	float wingca = 1.23f*rearwingarea*sin(rearwingangle);

	float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0f) +
			   GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0f);
	float h = 0.0f;
	int i;
	for (i = 0; i < 4; i++)
		h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
	h*= 1.5f; h = h*h; h = h*h; h = 2.0f * exp(-3.0f*h);
	CA = h*cl + 4.0f*wingca;
}


// Compute aerodynamic drag coefficient CW.
void Driver::initCw()
{
	float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0f);
	float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0f);
	CW = 0.645f*cx*frontarea;
}


// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
	const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
	float tm = FLT_MAX;
	int i;

	for (i = 0; i < 4; i++) {
		tm = MIN(tm, GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char*) NULL, 1.0f));
	}
	TIREMU = tm;
}

// Compute the needed distance to brake.
float Driver::brakedist(float allowedspeed, float mu)
{
	float c = mu*G;
	float d = (CA*mu + CW)/mass;
	float v1sqr = currentspeedsqr;
	float v2sqr = allowedspeed*allowedspeed;
	return -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0f*d);
}

float Driver::filterABS(float brake)
{
	if(getSpeed() < 3) return brake;
	float slip = 0.0f;
	for(int i = 0; i < 4; i++)
	{
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / getSpeed();
	}
	return brake / (slip / 4);
}

// Brake filter for pit stop.
float Driver::filterBPit(float brake)
{
	if (pit->getPitstop() && !pit->getInPit()) {
		float dl, dw;
		RtDistToPit(car, track, &dl, &dw);
		if (dl < PIT_BRAKE_AHEAD) {
			float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
			if (brakedist(0.0f, mu) > dl) {
				return 1.0f;
			}
		}
	}

	if (pit->getInPit()) {
		float s = pit->toSplineCoord(car->_distFromStartLine);
		// Pit entry.
		if (pit->getPitstop()) {
			float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
			if (s < pit->getNPitStart()) {
				// Brake to pit speed limit.
				float dist = pit->getNPitStart() - s;
				if (brakedist(pit->getSpeedlimit(), mu) > dist) {
					return 1.0f;
				}
			} else {
				// Hold speed limit.
				if (currentspeedsqr > pit->getSpeedlimitSqr()) {
					return pit->getSpeedLimitBrake(currentspeedsqr);
				}
			}
			// Brake into pit (speed limit 0.0 to stop)
			float dist = pit->getNPitLoc() - s;
			if (pit->isTimeout(dist)) {
				pit->setPitstop(false);
				return 0.0f;
			} else {
				if (brakedist(0.0f, mu) > dist) {
					return 1.0f;
				} else if (s > pit->getNPitLoc()) {
					// Stop in the pit.
			 		return 1.0f;
				}
			}
		} else {
			// Pit exit.
			if (s < pit->getNPitEnd()) {
				// Pit speed limit.
				if (currentspeedsqr > pit->getSpeedlimitSqr()) {
					return pit->getSpeedLimitBrake(currentspeedsqr);
				}
			}
		}
	}

	return brake;
}

// Steer filter for collision avoidance.
float Driver::filterSColl(float steer)
{
	int i;
	float sidedist = 0.0f, fsidedist = 0.0f, minsidedist = FLT_MAX;
	Opponent *o = NULL;

	// Get the index of the nearest car (o).
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_SIDE) {
			sidedist = opponent[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				o = &opponent[i];
			}
		}
	}

	// If there is another car handle the situation.
	if (o != NULL) {
		float d = fsidedist - o->getWidth();
		// Near, so we need to look at it.
		if (d < SIDECOLL_MARGIN) {
			/* compute angle between cars */
			tCarElt *ocar = o->getCarPtr();
			float diffangle = ocar->_yaw - car->_yaw;
			NORM_PI_PI(diffangle);
			// We are near and heading toward the car.
			if (diffangle*o->getSideDist() < 0.0f) {
				const float c = SIDECOLL_MARGIN/2.0f;
				d = d - c;
				if (d < 0.0f) {
					d = 0.0f;
				}

				// Steer delta required to drive parallel to the opponent.
				float psteer = diffangle/car->_steerLock;
				myoffset = car->_trkPos.toMiddle;

				// Limit myoffset to suitable limits.
				float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
				if (fabs(myoffset) > w) {
					myoffset = (myoffset > 0.0f) ? w : -w;
				}
				
				// On straights the car near to the middle can correct more, in turns the car inside
				// the turn does (because if you leave the track on the turn "inside" you will skid
				// back to the track.
				if (car->_trkPos.seg->type == TR_STR) {
					if (fabs(car->_trkPos.toMiddle) > fabs(ocar->_trkPos.toMiddle)) {
						// Its me, I do correct not that much.
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						// Its the opponent, so I correct more.
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				} else {
					// Who is outside, heavy corrections are less dangerous
					// if you drive near the middle of the track.
					float outside = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;
					float sign = (car->_trkPos.seg->type == TR_RGT) ? 1.0f : -1.0f;
					if (outside*sign > 0.0f) {
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				}

				if (psteer*steer > 0.0f && fabs(steer) > fabs(psteer)) {
					return steer;
				} else {
					return psteer;
				}
			}
		}
	}
	return steer;
}

// Brake filter for collision avoidance.
float Driver::filterBColl(float brake)
{
	float mu = car->_trkPos.seg->surface->kFriction;
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_COLL) {
			if (brakedist(opponent[i].getSpeed(), mu) > opponent[i].getDistance()) {
				return 1.0f;
			}
		}
	}
	return brake;
}


// Checks if there is an opponent near us for collision avoidance.
// sign == 1 --> check left side
// sign == -1 --> check right side
Opponent* Driver::checkSColl(int sign)
{
	int i;
	float sidedist = 0.0f, fsidedist = 0.0f, minsidedist = FLT_MAX;
	Opponent* opp = NULL;

	// Get the index of the nearest car (o).
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if ((opponent[i].getState() & OPP_SIDE) && (opponent[i].getCarPtr()->_trkPos.toMiddle * sign > car->_trkPos.toMiddle)) {
			sidedist = opponent[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				opp = &opponent[i];
			}
		}
	}
	return opp;				
}

// Checks if there is an opponent to overtake
Opponent* Driver::checkOvertake()
{
	int i;
    float catchdist, mincatchdist = FLT_MAX;
    Opponent *opp = NULL;
    /*for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_FRONT) {
            catchdist = opponent[i].getCatchDist();
            if (catchdist < mincatchdist) {
                mincatchdist = catchdist;
                opp = &opponent[i];
            }
        }
    }*/
	// Front opponent 100 meters or less 
    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_FRONT) {
            catchdist = opponent[i].getDistance();
            if (catchdist < mincatchdist && catchdist < 100.0f) {
                mincatchdist = catchdist;
                opp = &opponent[i];
            }
        }
    }
	return opp;
}

int Driver::checkLetPass()
{
	int i;
    Opponent *opp = NULL;
    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_LETPASS) {
			return 1;
        }
    }
	return 0;
}

float Driver::filterTCL(float accel)
{
	if(getSpeed() < 1) return accel;
	float slip = 0.0f;
	for(int i = 0; i < 4; i++)
	{
		slip = MAX(car->_wheelSpinVel(i) * car->_wheelRadius(i), slip);
		//slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / getSpeed();
	} 
	slip = 1 - (slip - getSpeed()) / slip;
	//std::cout << "SLIP: " << slip << "\n";
	return slip < 0.9 ? accel * slip / 1.5f : accel;
}

// Hold car on the track.
float Driver::filterTrk(float accel)
{
	tTrackSeg* seg = car->_trkPos.seg;
	float widthDiv = 5.5f;

	if (pit->getInPit())	// Pit stop
	{
		return accel;
	}

	if(radius[seg->id] < 1000)
	{
		if(seg->type == TR_RGT)
		{
			if(car->_trkPos.toMiddle > seg->width / widthDiv)
			{
				return accel * 0.2;
			}
			else if(car->_trkPos.toMiddle > seg->width * 1.05)
			{
				return accel * 0.05;
			}
			else
			{
				return accel;
			}
			
		}
		else if(seg->type == TR_LFT)
		{
			if(car->_trkPos.toMiddle < -seg->width / widthDiv)
			{
				return accel * 0.2;
			}
			else
			{
				return accel;
			}
		}
	}
	else if(car->_trkPos.toMiddle > seg->width * 1.05f && getSpeed() > 10.0f)
	{
		return accel * 0.2;
	}

	return accel;
}

float Driver::getCarAngle()
{
    return RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
}
