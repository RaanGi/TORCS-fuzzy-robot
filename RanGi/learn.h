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

class RadiusLearn {
    public:
        RadiusLearn(tTrack* t, tSituation *s, int driverindex);
        ~RadiusLearn();

        float* getRadiusArray() { return radius; }
        void update(tSituation *s, tTrack *t, tCarElt *car, tTrackSeg *lookaheadSeg, int alone, bool inPit);
        bool learned() { return fileExist; }

    private:
        float *radius;
        int *updateLap;
        int *consecutiveDecrease;
		int nseg;
        int laps;
        int goodLaps;
        int increasedRadiusLap;
        int decreasedRadiusLap;
        char filename[1024];
        bool fileExist;
        bool trackLearned;
        bool validLap;
        double *sectTime;

        typedef struct radiusSetup
        {
            float *radius;
            double time;
        };

        radiusSetup actualSetup;
        radiusSetup bestSetup;
        radiusSetup bestSetupBigFuel; 

        typedef struct SectorSetup
        {
            float *radius;
            float fuel;
            int nseg;
            double time;
        };

        SectorSetup *bestSectorSetup;
        SectorSetup *bestSectorSetupBigFuel;
        
        

        // Class constants
        static const float MAX_SEG_BIG_MOD;
        static const float MAX_SEG_MOD;
        static const int MIN_GOOD_LAPS;
        static const int MAX_CONSECUTIVE_DECREASE;
        static const int MAX_SECTORS;

        void writeRadius();
        bool readRadius(tTrack* track, float *radius, int driverindex);
        FILE* getRadiusFile(tTrack* track, int driverindex);
        void updateBestSectorSetup(tCarElt *car, int sector);
        void processTraining();
};