#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>

#include <iostream>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include <portability.h>

#include "fuzzycontrol.h"
#include "linalg.h"
#include "cardata.h"
#include "opponent.h"
#include "strategy.h"
#include "pit.h"
#include "learn.h"

#define RANGI_SECT_PRIV "RanGi private"
#define RANGI_ATT_MUFACTOR "mufactor"
#define RANGI_ATT_FUELPERLAP "fuelperlap"

class FuzzyControl;
class Opponents;
class Opponent;
class Pit;
class AbstractStrategy;

struct straight
{
	int startSegId = 0;
	int endSegId = 0;
	int actualSegId = 0;
	int cross = 0;
	int segNum = 0;
	float length = 0.0f;
	float completed = 0.0f;
	float done = 0.0f;
	int lap = -1;
};

class Driver
{
    public:
        Driver(int index);
        ~Driver();

        // Callback functions called from TORCS.
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		void drive(tSituation *s);
		int pitCommand(tSituation *s);
		void endRace(tSituation *s);
		void shutDown();

		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return car->_speed_x; }
		
    private:
        // Utility functions.
		bool isStuck();
		void update(tSituation *s);
		float getAllowedSpeed(tTrackSeg *segment);
		void getAccelBrake(float*, float*);
		float getDistToSegEnd();
		int getGear();
		float getSteer();
		float getOffset(float lookahead);
		float updateOffset(float proportion, float lookahead);
		float getClutch();
		v2d getTargetPoint();
		float getOffset();
		float brakedist(float allowedspeed, float mu);

		float filterABS(float brake);
		float filterBPit(float brake);
		float filterBColl(float brake);

		float filterSColl(float steer);

		float filterTCL(float accel);
		float filterTrk(float accel);

		Opponent* checkSColl(int sign);
		Opponent* checkOvertake();
		int checkLetPass();

        float getCarAngle();    // Return car angle based on track angle

        void initCa();
		void initCw();
		void initTireMu();

        void computeRadius(float *radius);
		void computeStraigth();
		int isAlone();
		tTrackSeg* getLookaheadSeg(float lookahead);


        // Per robot global data.
		int stuck;
		float speedangle;		// the angle of the speed vector relative to trackangle, > 0.0 points to right.
		float mass;				// Mass of car + fuel.
		float myoffset;			// Offset to the track middle.
		tCarElt *car;			// Pointer to tCarElt struct.

		Opponents *opponents;	// The container for opponents.
		Opponent *opponent;		// The array of opponents.

		Pit *pit;						// Pointer to the pit instance.
		AbstractStrategy *strategy;		// Pit stop strategy.

        FuzzyControl *fc;        // Control for all fuzzy logic based systems
		
		static Cardata *cardata;		// Data about all cars shared by all instances.
		SingleCardata *mycardata;		// Pointer to "global" data about my car.
		static double currentsimtime;	// Store time to avoid useless updates.

        float currentspeedsqr;	// Square of the current speed_x.
		float clutchtime;		// Clutch timer.
		float oldlookahead;		// Steering lookahead in the previous step.
		float oldOffset;
		float speedRedFactor;	// Reduction factor when turning alongside an opponent
		int nextTurnId;
		int prevTurnType;
		int overtakeMove;
		int letPassOpp;

		float lastFuelChecked;	// Car fuel the last time pitstop was checked
		int lastDamageChecked;			// Car damage the last time pitstop was checked

        float *radius;
		RadiusLearn *learn;
		int alone;
		
		struct straight straightInfo;

        // Data that should stay constant after first initialization.
		int MAX_UNSTUCK_COUNT;
		int INDEX;
		float CARMASS;		// Mass of the car only [kg].
		float CA;			// Aerodynamic downforce coefficient.
		float CW;			// Aerodynamic drag coefficient.
		float TIREMU;		// Friction coefficient of tires.
		float (Driver::*GET_DRIVEN_WHEEL_SPEED)();
		float OVERTAKE_OFFSET_INC;		// [m/timestep]
		float MU_FACTOR;				// [-]
		float MAX_ABS_OFFSET;				// [m]

		// Class constants.
		static const float MAX_UNSTUCK_ANGLE;
		static const float UNSTUCK_TIME_LIMIT;
		static const float MAX_UNSTUCK_SPEED;
		static const float MIN_UNSTUCK_DIST;
		static const float G;
		static const float FULL_ACCEL_MARGIN;
		static const float SHIFT;
		static const float SHIFT_MARGIN;
		static const float ABS_SLIP;
		static const float ABS_RANGE ;
		static const float ABS_MINSPEED;
		static const float TCL_SLIP;
		static const float LOOKAHEAD_CONST;
		static const float LOOKAHEAD_ACCEL_BRAKE_FACTOR;
		static const float LOOKAHEAD_STEER_FACTOR;
		static const float WIDTHDIV;
		static const float SIDECOLL_MARGIN;
		static const float BORDER_OVERTAKE_MARGIN;
		static const float OVERTAKE_OFFSET_SPEED;
		static const float PIT_LOOKAHEAD;
		static const float PIT_BRAKE_AHEAD;
		static const float PIT_MU;
		static const float MAX_SPEED;
		static const float TCL_RANGE;
		static const float MAX_FUEL_PER_METER;
		static const float CLUTCH_SPEED;
		static const float CENTERDIV;
		static const float DISTCUTOFF;
		static const float MAX_INC_FACTOR;
		static const float CATCH_FACTOR;
		static const float CLUTCH_FULL_MAX_TIME;

		static const float TEAM_REAR_DIST;
		static const int TEAM_DAMAGE_CHANGE_LEAD;

		// Track variables.
		tTrack* track;

};

#endif // _DRIVER_H_