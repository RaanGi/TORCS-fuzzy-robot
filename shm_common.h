#include <semaphore.h>

#define KEY 0x060398

enum
{
    ACCEL_BRAKE,
    STEER,
    PITSTOP,
    START,
    TERMINATE
};

typedef struct
{
    float speedError = 0.0f;    // [ km/h ]
    float accel = 0.0f;
    float brake = 0.0f;
    float slip = 0.0f;

    float speed = 0.0f;         // [ km/h ]
    float carToLimit = 0.0f;
    float bias = 0.0f;

    float fuel = 0.0f;
    float fuelLaps = 0.0f;      /* Remaining fuel divided by the estimated fuel needed to end the race 
                                    - Greater then 1 -> there's enough fuel
                                    - Smaller than 1 -> there's not enough fuel*/
    int damage = 0;
    int pitstop = 0;

    int state = 0;
    int shmid = 0x0;
    sem_t sem;
} shData;
