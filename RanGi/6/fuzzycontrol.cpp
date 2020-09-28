#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/wait.h> 
#include <iostream>

#include <ctime>

#include "fuzzycontrol.h"

FuzzyControl::FuzzyControl()
{
    
}

FuzzyControl::~FuzzyControl()
{

}

void FuzzyControl::startFuzzyEngines(int index)
{
    key_t key = KEY + index;
    shmid = shmget(key, sizeof(shData), 0666|IPC_CREAT);
    m_data = (shData*) shmat(shmid, NULL, 0);
    m_data->state = START;
}

void FuzzyControl::terminateFuzzyEngines()
{
    m_data->state = TERMINATE;
    sem_post(&(m_data->sem));
    shmdt(m_data);
}

void FuzzyControl::setSpeedError(float se)
{
    m_data->speedError = se;
}

void FuzzyControl::setSlip(float slip)
{
    m_data->slip = slip;
}

void FuzzyControl::setSpeed(float speed)
{
    m_data->speed = speed;
}

void FuzzyControl::setCarToLimit(float carToLimit)
{
    m_data->carToLimit = carToLimit;
}

void FuzzyControl::setFuel(float fuel)
{
    m_data->fuel = fuel;
}

void FuzzyControl::setFuelLaps(float fuelLaps)
{
    m_data->fuelLaps = fuelLaps > 2 ? 2 : fuelLaps;
}

void FuzzyControl::setDamage(int damage)
{
    m_data->damage = damage;
}

void FuzzyControl::setState(int state)
{
    m_data->state = state;
}

float FuzzyControl::getAccel()
{
    return m_data->accel;
}

float FuzzyControl::getBrake()
{
    return m_data->brake;
}

float FuzzyControl::getOffset()
{
    return m_data->bias;
}

float FuzzyControl::getFuelLaps()
{
    return m_data->fuelLaps;
}

int FuzzyControl::getPitstop()
{
    return m_data->pitstop;
}

void FuzzyControl::processFuzzyData()
{
    if(clock_gettime(CLOCK_REALTIME, &ts) == -1) return;
    ts.tv_nsec += 10 * 1000 * 1000;     // 10 milliseconds to nanoseconds
    sem_post(&(m_data->sem));
    usleep(100);
    sem_timedwait(&(m_data->sem), &ts); 
}