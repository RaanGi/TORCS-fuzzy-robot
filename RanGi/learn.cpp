/***************************************************************************

    file                 : learn.cpp
    created              : Wed Aug 28 16:36:00 CET 2004
    copyright            : (C) 2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: learn.cpp,v 1.3 2006/03/06 22:43:50 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 * This version has been modified. The way to learn the track is different *
 * from de original.                                                       *
 * Marcos Arroyo Ruiz, 2020                                                *
 *                                                                         *
 ***************************************************************************/


#include "learn.h"
#include <portability.h>

#define MAGIC1		0x34be1f01
// Change MAGIC2 if the learning file format changes.
#define MAGIC2		0x45aa9fbe
#define STRINGID	"TORCS"

const float RadiusLearn::MAX_SEG_BIG_MOD = 6;
const float RadiusLearn::MAX_SEG_MOD = 10;
const int RadiusLearn::MIN_GOOD_LAPS = 2;
const int RadiusLearn::MAX_CONSECUTIVE_DECREASE = 2;
const int RadiusLearn::MAX_SECTORS = 6;


RadiusLearn::RadiusLearn(tTrack* t, tSituation *s, int driverindex)
{
	radius = new float[t->nseg];
    updateLap = new int[t->nseg];
	consecutiveDecrease = new int[t->nseg];
	bestSetup.radius = new float[t->nseg];
	bestSetup.time = 0.0;
	bestSetupBigFuel.radius = new float[t->nseg];
	bestSetupBigFuel.time = 0.0;
	actualSetup.radius = new float[t->nseg];
	actualSetup.time = FLT_MAX;
	sectTime = new double[MAX_SECTORS];
	bestSectorSetup = new SectorSetup[MAX_SECTORS];
	bestSectorSetupBigFuel = new SectorSetup[MAX_SECTORS];
    nseg = t->nseg;
	laps = 0;
	goodLaps = -1;
	increasedRadiusLap = -1;
	decreasedRadiusLap = -1;
	trackLearned = false;
	validLap = true;

	int nSegSector = int(nseg / MAX_SECTORS);
	for(int i = 0; i < MAX_SECTORS; i++)
	{
		bestSectorSetup[i].nseg = i < (MAX_SECTORS - 1) ? nSegSector : nseg - nSegSector * i;
		bestSectorSetup[i].radius = &(bestSetup.radius[nSegSector * i]);
		bestSectorSetup[i].time = FLT_MAX;
		bestSectorSetup[i].fuel = 0.0f;
		
		bestSectorSetupBigFuel[i].nseg = i < (MAX_SECTORS - 1) ? nSegSector : nseg - nSegSector * i;
		bestSectorSetupBigFuel[i].radius = &(bestSetupBigFuel.radius[nSegSector * i]);
		bestSectorSetupBigFuel[i].time = FLT_MAX;
		bestSectorSetupBigFuel[i].fuel = 0.0f;
	}


	fileExist = readRadius(t, radius, driverindex);

	memset(bestSetup.radius, 0.0f, sizeof(bestSetup.radius[0]) * nseg);
	memset(bestSetupBigFuel.radius, 0.0f, sizeof(bestSetupBigFuel.radius[0]) * nseg);
    memset(updateLap, -1, sizeof(updateLap[0]) * nseg);
	memset(consecutiveDecrease, 0, sizeof(consecutiveDecrease[0]) * nseg);
	memset(sectTime, 0.0, sizeof(sectTime[0]) * MAX_SECTORS);
}


RadiusLearn::~RadiusLearn()
{
	if(laps > 2)
	{
		processTraining();
		writeRadius();
	} 
	delete [] radius;
	delete [] actualSetup.radius;
	delete [] bestSetup.radius;
	delete [] bestSetupBigFuel.radius;
	delete [] sectTime;
	delete [] bestSectorSetup;
	delete [] bestSectorSetupBigFuel;
}


void RadiusLearn::update(tSituation *s, tTrack *t, tCarElt *car, tTrackSeg *lookaheadSeg, int alone, bool inPit)
{
	if(((alone && !trackLearned) || (s->_raceType == RM_TYPE_PRACTICE && trackLearned)) && !inPit)
    {
		for(int i = 0; i < MAX_SECTORS; i++)
		{
			int segId = bestSectorSetup[0].nseg;
			for(int j = 0; j < i; j++)
			{
				segId += bestSectorSetup[j].nseg;
			}
			if(car->_trkPos.seg->id == fmod(segId, nseg) && laps > 1)
			{
				sectTime[i] = car->_curLapTime;
				for(int j = 0; j < i; j++)
				{
					sectTime[i] -= sectTime[j];
				}
				updateBestSectorSetup(car, i);
			}
		}
		if(car->_laps > laps) 
		{
			if(decreasedRadiusLap != laps)	goodLaps++;
			else	goodLaps = 0;
			laps = car->_laps;
			if(goodLaps >= MIN_GOOD_LAPS)
			{
				if(!trackLearned)	trackLearned = true;
				for(int i = 0; i < nseg; i++)
				{ 
					if(radius[i] < FLT_MAX)	radius[i]++;
				}
				std::cout << "\nRADIUS INCREASED\n";
				increasedRadiusLap = laps;
			}
			validLap = true;
			memset(sectTime, 0.0, sizeof(sectTime[0]) * 3);
		}

        tTrackSeg* seg = car->_trkPos.seg;
        float widthDiv = 5.0f;

        if(radius[seg->id] < 1000)
        {
            if(
				(seg->type == TR_RGT 
				&& ((seg->type == lookaheadSeg->type && car->_trkPos.toMiddle > seg->width / widthDiv)
				|| (seg->type != lookaheadSeg->type && car->_trkPos.toMiddle > seg->width / (widthDiv - 0.75f)))) || 
				(seg->type == TR_LFT 
				&& ((seg->type == lookaheadSeg->type && car->_trkPos.toMiddle < -seg->width / widthDiv)
				|| (seg->type != lookaheadSeg->type && car->_trkPos.toMiddle < -seg->width / (widthDiv - 0.75f)))) ||
				(seg->type == TR_STR && fabs(car->_trkPos.toMiddle) > seg->width * 0.35f && car->_speed_x > 10.0f)
			)
            {
                int i = 0;
				while(i < MAX_SEG_MOD)
                {
                    //seg = seg->next->next;
                    if(updateLap[seg->id] < car->_laps && consecutiveDecrease[seg->id] < MAX_CONSECUTIVE_DECREASE)
                    {
						if(updateLap[seg->id] == laps - 1)	consecutiveDecrease[seg->id]++;
                        std::cout << "LAP: " << laps << "\tSEG: " << seg->id << "\tRadius corrected: " << radius[seg->id] << " --> "; 
                        radius[seg->id] -= (i < MAX_SEG_BIG_MOD ? 3.0f : 1.0f);
                        updateLap[seg->id] = car->_laps;
                        if(radius[seg->id] <= 0)	radius[seg->id] = 2.0f;
						else if(radius[seg->id] > 500.0f)	radius[seg->id] = 300.0f;
                        std::cout << radius[seg->id] << "\n";
						i++;
                    }
					else if(consecutiveDecrease[seg->id] >= MAX_CONSECUTIVE_DECREASE)
					{
						consecutiveDecrease[seg->id] = 0;
						std::cout << "LAP: " << laps << "\tSEG: " << seg->id << "\tCONSECUTIVE\n";
					}
					else
					{
						i++;
					}
					
                } 
				decreasedRadiusLap = car->_laps;              
            }
        }
		if(validLap && !(car->_commitBestLapTime))	validLap = false;
    }
}

void RadiusLearn::updateBestSectorSetup(tCarElt *car, int sector)
{
	if(validLap)
	{
		if(sectTime[sector] < bestSectorSetup[sector].time && sectTime[sector] > 0.0)
		{
			for(int i = 0; i < bestSectorSetup[sector].nseg; i++)
			{
				bestSectorSetup[sector].radius[i] = radius[i + bestSectorSetup[0].nseg * sector];
			}
			bestSectorSetup[sector].time = sectTime[sector];
			bestSectorSetup[sector].fuel = car->_fuel;
			std::cout << "\nUPDATE SECTOR " << (sector + 1) << " SETUP LAP: " <<  car->_laps << "\n";
		}
		if(sectTime[sector] < bestSectorSetupBigFuel[sector].time && sectTime[sector] > 0.0 && car->_fuel / car->_tank > 0.75f)
		{
			for(int i = 0; i < bestSectorSetupBigFuel[sector].nseg; i++)
			{
				bestSectorSetupBigFuel[sector].radius[i] = radius[i + bestSectorSetupBigFuel[0].nseg * sector];
			}
			bestSectorSetupBigFuel[sector].time = sectTime[sector];
			bestSectorSetupBigFuel[sector].fuel = car->_fuel;
			std::cout << "UPDATE SECTOR " << (sector + 1) << " BIG FUEL SETUP LAP: " <<  car->_laps << "\n";
		}
	}
}

void RadiusLearn::writeRadius()
{
	// Build the directory name.
	char path[sizeof(filename)];
	strncpy(path, filename, sizeof(path));
	char* end = strrchr(path, '/');
	if (end != NULL) {
		*end = '\0';
	}

	// Create the directory and try to write data.
	if (GfCreateDir(path) == GF_DIR_CREATED) {
		// Try to write data.
		FILE *fd = fopen(filename, "wb");
		if (fd != NULL) {
			// Create header: Magic Number, #segments, string, version.
			int magic = MAGIC1;
			int magic2 = MAGIC2;
			char string[] = STRINGID;
			
			// The magic numbers are used to catch 32/64 bit mismatches and little/big
			// endian mismatches. Here the patterns I expect at the beginning of the files.
			// I call 4 bytes a UNIT, MAGIC1 bytes A1, A2, A3, A4, MAGIC2 bytes B1, B2, B3, B4,
			// a zeroed byte 00):
			// 32bit big endian   : UNIT1-A1A2A3A4; UNIT2-B1B2B3B4; ...
			// 32bit little endian: UNIT1-A4A3A2A1; UNIT2-B4B3B2B1; ...
			// 64bit big endian   : UNIT1-00000000; UNIT2-A1A2A3A4; UNIT3-00000000; UNIT4-B1B2B3B4; ...
			// 64bit little endian: UNIT1-A4A3A2A1; UNIT2-00000000; UNIT3-B4B3B2B1; UNIT4-00000000: ...
			//
			// Like you can see there is created a unique pattern for each architecture in
			// UNIT1 and UNIT2.

			fwrite(&magic, sizeof(magic), 1, fd);			// magic number.
			fwrite(&magic2, sizeof(magic2), 1, fd);			// magic number 2.
			fwrite(&nseg, sizeof(nseg), 1, fd);				// # segments.
			fwrite(string, sizeof(string), 1, fd);			// string.
			
			for (int i = 0; i < nseg; i++) {
				fwrite(&(bestSetup.radius[i]), sizeof(radius[0]), 1, fd);
        	}
			fwrite(&(bestSetup.time), sizeof(actualSetup.time), 1, fd);
			fwrite(&trackLearned, sizeof(trackLearned), 1, fd);
        	fclose(fd);
		}
	}
}


bool RadiusLearn::readRadius(tTrack* track, float *radius, int driverindex)
{
	FILE* fd = getRadiusFile(track, driverindex);

	if (fd != NULL) {
		// Check if the file is valid.
		int magic = 0;
		int magic2 = 0;
		int nseg = 0;
		char string[sizeof(STRINGID)] = "";
	
		fread(&magic, sizeof(magic), 1, fd);
		fread(&magic2, sizeof(magic2), 1, fd);
		fread(&nseg, sizeof(nseg), 1, fd);
		fread(string, sizeof(string), 1, fd);
	
		if (magic == MAGIC1 && magic2 == MAGIC2 &&
			nseg == track->nseg &&
			strncmp(string, STRINGID, sizeof(string)) == 0
			)
		{
			for (int i = 0; i < track->nseg; i++) {
				fread(&radius[i], sizeof(radius[0]), 1, fd);
				actualSetup.radius[i] = radius[i];
        	}
			fread(&(actualSetup.time), sizeof(actualSetup.time), 1, fd);
			fread(&trackLearned, sizeof(trackLearned), 1, fd);
			fclose(fd);
			return true;
		}
		fclose(fd);
	}
	return false;
}


FILE* RadiusLearn::getRadiusFile(tTrack* track, int driverindex)
{
	const int TBUFSIZE = 256;
	char tbuf[TBUFSIZE];
	char* trackname = strrchr(track->filename, '/') + 1;
	char* tracknameend = strchr(trackname, '.') - 1;

	strncpy(tbuf, trackname, tracknameend-trackname+1);
	tbuf[tracknameend-trackname+1] = 0;

	FILE* fd;
	char buffer[sizeof(filename)];
    const char *path = "%sdrivers/RanGi/%d/learned/%s.radius";

    // First construct a path to the local directory ($HOME/...).
	snprintf(buffer, sizeof(buffer), path, GetLocalDir(), driverindex, tbuf);
    strncpy(filename, buffer, sizeof(filename));

	// Try to open the local file.		
	if ((fd = fopen(buffer, "rb")) != NULL) {
		return fd;
	}

	// Not found, try the global path.
	snprintf(buffer, sizeof(buffer), path, GetDataDir(), driverindex, tbuf);
	return fopen(buffer, "rb");    
}

void RadiusLearn::processTraining()
{
	std::cout << "\n\nTRACK LEARNED: " << (trackLearned ? "TRUE\n\n" : "FALSE\n\n");

	for(int i = 0; i < nseg; i++)
	{
		if(bestSetup.radius[i] == 0.0f)	bestSetup.radius[i] = radius[i];
		if(bestSetupBigFuel.radius[i] == 0.0f)	bestSetupBigFuel.radius[i] = radius[i];
	}

	for(int i = 0; i < MAX_SECTORS; i++)
	{
		bestSetup.time += bestSectorSetup[i].time;
		bestSetupBigFuel.time += bestSectorSetupBigFuel[i].time;
	}
	bestSetup.time = bestSetup.time * 0.8 + bestSetupBigFuel.time * 0.2;

	if(bestSetup.time < actualSetup.time || !fileExist)
	{
		for(int i = 0; i < nseg; i++)
		{
			bestSetup.radius[i] = bestSetup.radius[i] * 0.8 + bestSetupBigFuel.radius[i] * 0.2;
		}
		std::cout << "SETUP UPDATED\n";
	}
	else
	{
		for(int i = 0; i < nseg; i++)
		{
			bestSetup.radius[i] = fileExist ? actualSetup.radius[i] : radius[i];
		}
		bestSetup.time = fileExist ? actualSetup.time : FLT_MAX;
		std::cout << "ACTUAL SETUP IS OK\n";
	}
}
