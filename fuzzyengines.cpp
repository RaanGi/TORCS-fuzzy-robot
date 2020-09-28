
#define NBBOTS 10

#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <thread> 
#include <mutex>
#include <condition_variable>
#include  <signal.h>

#include "shm_common.h"
#include "fl/Headers.h"

using namespace fl;

enum
{
    WAIT,
    CLOSING_HELPER,
    CLOSING_ENGINE,
    READY
};

std::mutex helperMutex, engineMutex, mainMutex;
std::condition_variable cv;
int state = WAIT;
int closeFE = 0;
int threadIndex[NBBOTS];
int activeHelpers = 0;
int activeEngines = 0;

void engines(int index, shData* data);
void helper(int index, std::thread* th, shData* data);
void closer(int index, sem_t* sem);
void INThandler(int sig);

void engines(int index, shData* data)
{
    {
        std::lock_guard<std::mutex> lk(engineMutex);
        std::cout << "** Engine thread " << index << " created **\n";
        activeEngines++;
    }

    float accBrakeOutput;

    Engine* abEngine = FllImporter().fromFile("Fuzzyengines/AccelBrake.fll");
    InputVariable* speedError = abEngine->getInputVariable("speedError");
    InputVariable* slip = abEngine->getInputVariable("slip");
    OutputVariable* accelBrake = abEngine->getOutputVariable("accelbrake");

    Engine* biasEngine = FllImporter().fromFile("Fuzzyengines/Bias.fll"); 
    InputVariable* speed = biasEngine->getInputVariable("speed");
    InputVariable* carToLimit = biasEngine->getInputVariable("carToLimit");
    OutputVariable* bias = biasEngine->getOutputVariable("bias");

    Engine* pitstopsEngine = FllImporter().fromFile("Fuzzyengines/Pitstops.fll");
    InputVariable* fuel = pitstopsEngine->getInputVariable("fuel");
    InputVariable* fuelLaps = pitstopsEngine->getInputVariable("fuelLaps");
    InputVariable* damage = pitstopsEngine->getInputVariable("damage");
    OutputVariable* pitstop = pitstopsEngine->getOutputVariable("pitstop");
    
    while(data->state != TERMINATE && !closeFE)
    {
        sem_wait(&(data->sem));
        switch (data->state)
        {
            case ACCEL_BRAKE:
                //std::cout << "Process accel brake\n";
                speedError->setValue(data->speedError);
                slip->setValue(data->slip);
                abEngine->process();
                //data->accel = (float) accel->getValue();
                //data->brake = (float) brake->getValue();
                accBrakeOutput = accelBrake->getValue();
                //std::cout << "ACB: " << accBrakeOutput << "\n";
                if(accBrakeOutput >= 0)
                {
                    data->accel = accBrakeOutput / 0.99;
                    data->brake = 0.0;
                }
                else
                {
                    data->accel = 0.0;
                    data->brake = -accBrakeOutput;
                }
                
                //std::cout << "SPEED-ERROR:" << speedError->getValue() << "\tACCEL: " << data->accel << "\tBRAKE: " << data->brake << "\n";
                break;
                
            case STEER:
                //std::cout << "Process bias\n";
                speed->setValue(data->speed < 0 ? 0 : data->speed);
                carToLimit->setValue(data->carToLimit < 2.0f ? data->carToLimit : 2.0f);
                //std::cout << speed->fuzzyInputValue() << "\n";
                biasEngine->process();
                //std::cout << bias->fuzzyOutputValue() << "\n";
                data->bias = bias->getValue();
                //std::cout << "bias: " << bias->getValue() << "\n";                
                break;

            case PITSTOP:
                //std::cout << "Process PITSTOP\n";
                fuel->setValue(data->fuel);
                fuelLaps->setValue(data->fuelLaps);
                damage->setValue(data->damage);
                pitstopsEngine->process();
                data->pitstop = pitstop->getValue() > 0.5 ? 1 : 0;
                std::cout << "FUEL: " << fuel->getValue() << "\tFUELLAPS: " << fuelLaps->getValue() << "\tDAMAGE: " << damage->getValue() << "\n";
                std::cout << "PITSTOP: " << data->pitstop << "\n";              
                break;

            default:
                break;
        }
        sem_post(&(data->sem));
    }

    int shmid = data->shmid;
    sem_destroy(&(data->sem));
    shmdt(data);
    shmctl(shmid, IPC_RMID, 0);

    {
        std::unique_lock<std::mutex> lk(engineMutex);
        cv.wait(lk, []{return state = READY;});
        state = CLOSING_ENGINE;
        threadIndex[index]++;
        activeEngines--;
        std::cout << "** Engine thread " << index << " closed **\n";
    }
    cv.notify_one();
}

void helper(int index, std::thread* th, shData* data)
{
    {
        std::lock_guard<std::mutex> lk(helperMutex);
        std::cout << "## Helper thread " << index << " created ##\n";
        activeHelpers++;
    }

    key_t key = KEY + index;
    int shmid = shmget(key, sizeof(shData), 0666|IPC_CREAT);
    data = (shData*) shmat(shmid, NULL, 0);
    memset(data, 0, sizeof(shData));
    sem_init(&(data->sem), 1, 0);
    data->shmid = shmid;

    //std::thread(closer, index, &(data->sem)).detach();

    sem_wait(&(data->sem));

    {
        std::unique_lock<std::mutex> lk(helperMutex);
        cv.wait(lk, []{return (state == READY);});

        state = CLOSING_HELPER;
        threadIndex[index]++;
        std::cout << "## Helper thread " << index << " closed ##\n";

        if(!closeFE)
        {
            *th = std::thread(engines, index, data);
        }
        activeHelpers--;
    }
    cv.notify_one();
}

void closer(int index, sem_t* sem)
{
    while(!closeFE) {sleep(1);}
    sem_post(sem);
}

void INThandler(int sig)
{
    closeFE++;
    signal(SIGINT, INThandler);
}

int main()
{
    std::thread helperThreads[NBBOTS];
    std::thread enginesThreads[NBBOTS];
    shData* data[NBBOTS];

    //signal(SIGINT, INThandler);

    for(int i = 0; i < NBBOTS; i++)
    {
        helperThreads[i] = std::thread(helper, i, &(enginesThreads[i]), data[i]);
        threadIndex[i] = 0;
    }
    {
        std::lock_guard<std::mutex> lk(helperMutex);
        state = READY;
    }
    cv.notify_one();

    while(!closeFE || (activeHelpers + activeEngines))
    {
        std::unique_lock<std::mutex> lk(mainMutex);
        cv.wait(lk, []{return (state == CLOSING_ENGINE || state == CLOSING_HELPER);});
        if(state == CLOSING_HELPER)
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            for(int i = 0; i < NBBOTS; i++)
            {
                if(threadIndex[i])
                {
                    helperThreads[i].join();
                    threadIndex[i]--;
                }
            }
            state = READY;
        }
        else if(state == CLOSING_ENGINE)
        {
            for(int i = 0; i < NBBOTS; i++)
            {
                if(threadIndex[i])
                {
                    enginesThreads[i].join();
                    if(!closeFE)
                    {
                        helperThreads[i] = std::thread(helper, i, &(enginesThreads[i]), data[i]);
                    }
                    threadIndex[i]--;
                }
            }
            state = READY;
        }
        cv.notify_all();
    }

    std::cout << "\nFuzzyengines closed\n\n";

    return 0;
}