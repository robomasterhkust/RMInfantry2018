#ifndef FEEDER
#define FEEDER

#define OUTPUT_MAX 6000

#define FEEDER_STOP 3
#define FEEDER_SINGLE 1
#define FEEDER_LONG 2

#define NORMAL_TURN 0
#define ERROR_TURN  1



//int16_t return_measured(int16_t *);


typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;


void feeder_func(int mode);
void feederInit(void);


#endif
