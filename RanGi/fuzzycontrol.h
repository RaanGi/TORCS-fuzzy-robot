#ifndef _FUZZYCONTROL_H_
#define _FUZZYCONTROL_H_

#include "shm_common.h"
#include "driver.h"

class Driver;

class FuzzyControl
{
private:
    shData *m_data;
    int shmid, enginesID;
    struct timespec ts;
public:
    FuzzyControl();
    ~FuzzyControl();

    void startFuzzyEngines(int);
    void terminateFuzzyEngines();
    
    void setSpeedError(float);
    void setSlip(float);

    void setSpeed(float);
    void setCarToLimit(float);

    void setFuel(float);
    void setFuelLaps(float);
    void setDamage(int);

    void setState(int);


    float getAccel();
    float getBrake();

    float getOffset();

    float getFuelLaps();
    int getPitstop();

    void processFuzzyData();
};

#endif