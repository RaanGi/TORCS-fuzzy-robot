/***************************************************************************

    file                 : RanGi.cpp
    created              : dom jul 26 13:46:50 CEST 2020
    copyright            : (C) 2002 Marcos Arroyo Ruiz

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#define BUFSIZE 20
#define NBBOTS 10

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <iostream>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "fuzzycontrol.h"
#include "linalg.h"
#include "driver.h"

static tTrack	*curTrack;

static const char* botname[NBBOTS] = {
	"RanGi 1", "RanGi 2", "RanGi 3", "RanGi 4", "RanGi 5",
	"RanGi 6", "RanGi 7", "RanGi 8", "RanGi 9", "RanGi 10"
};


/*static const char* botname[NBBOTS] = {
	"RanGi 1", "RanGi 2", "RanGi 3"
};*/
static Driver *driver[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
RanGi(tModInfo *modInfo) 
{
    int i;

    memset(modInfo, 0, 10*sizeof(tModInfo));

    for(i = 0; i < NBBOTS; i++)
    {
        modInfo[i].name    = strdup(botname[i]);		/* name of the module (short) */
        modInfo[i].desc    = strdup("");	/* description of the module (can be long) */
        modInfo[i].fctInit = InitFuncPt;		/* init function */
        modInfo[i].gfId    = ROB_IDENT;		/* supported framework version */
        modInfo[i].index   = i;
    }

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 
    driver[index] = new Driver(index);
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = pitcmd;    /* Pit commands */
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    driver[index]->initTrack(track, carHandle, carParmHandle, s);
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->newRace(car, s);
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->drive(s);
}

/* Pitstop callback. */
static int 
pitcmd(int index, tCarElt* car, tSituation *s)
{
	return driver[index]->pitCommand(s);
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    driver[index]->endRace(s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    driver[index]->shutDown();
    delete driver[index];
}

