#include "ch.h"
#include "hal.h"

#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "feeder.h"

#define FEEDER_CAN &CAND1
#define FEEDER_CAN_EID 0x200

#define GEAR_BOX (36.0f)

#define feeder_canUpdate() \
    (can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        set_speed, 0, 0, 0))

#define feeder_canStop() \
    (can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        0, 0, 0, 0))

#define FEEDER_INDEX 0

ChassisEncoder_canStruct*   feeder_encode;
RC_Ctl_t*                   p_dbus;


pid_struct  vel_pid;
pid_struct  pos_pid;

volatile int16_t set_speed;

int error_count = 0;


volatile float speed_sp = -15.0f / 7.0f * GEAR_BOX * 60.0f;   //  (15) / 7 * 36 * 60
float angle_change = -360.0f / 7.0f;
float error_angle_change;


volatile int16_t measured_speed_fuck;
static THD_WORKING_AREA(feeder_control_wa, 512);
static THD_FUNCTION(feeder_control, p){
    (void) p;
    chRegSetThreadName("feeder controller");
    while(!chThdShouldTerminateX()){
        feeder_func(p_dbus->rc.s1);
        measured_speed_fuck = (*feeder_encode).raw_speed;
        chThdSleepMilliseconds(10);
    }
}


volatile int16_t PID_VEL(float target){
    static float last_error;
    static float current_error;

    last_error = current_error;
    current_error = target - (float) (*feeder_encode).raw_speed;
    vel_pid.inte += current_error;
    vel_pid.inte = vel_pid.inte > vel_pid.inte_max?  vel_pid.inte_max:vel_pid.inte;
    vel_pid.inte = vel_pid.inte <-vel_pid.inte_max? -vel_pid.inte_max:vel_pid.inte;

    float output = vel_pid.kp * current_error + vel_pid.ki * vel_pid.inte + vel_pid.kd * (current_error - last_error);
    output = output > 6000?  6000:output;
    output = output <-6000? -6000:output;

    return (int16_t) output;

}

volatile float PID_POS(float target){
    static float last_error;
    static float current_error;

    last_error = current_error;
    current_error = target - (float) (*feeder_encode).total_ecd;
    pos_pid.inte += current_error;
    pos_pid.inte = pos_pid.inte > pos_pid.inte_max?  pos_pid.inte_max:pos_pid.inte;
    pos_pid.inte = pos_pid.inte <-pos_pid.inte_max? -pos_pid.inte_max:pos_pid.inte;

    float output = vel_pid.kp * current_error + vel_pid.ki * vel_pid.inte + vel_pid.kd * (current_error - last_error);
    output = output > 6000?  6000:output;
    output = output <-6000? -6000:output;

    return output;
}

void turn_angle(float angle_sp){
    float temp_speed = PID_POS(angle_sp);
    set_speed = PID_VEL(temp_speed);
}

void feeder_func(int mode){
    switch (mode){
        case FEEDER_STOP:
            set_speed = PID_VEL(0);
            feeder_canUpdate();
            break;
        case FEEDER_SINGLE:{
            int single_start = 1;
            float angle_sp = (*feeder_encode).total_ecd + angle_change / 360.0f * GEAR_BOX * CAN_ENCODER_RANGE;
            systime_t single_start_time = chVTGetSystemTime();
            while(single_start == 1){

                if( ( ST2MS(chVTGetSystemTime())-ST2MS(single_start_time)) > 500){
                    if(angle_change < 0){
                        error_angle_change = 360.0f / 36.0f;
                    }
                    else error_angle_change = -360.0f / 36.0f;
                    float error_angle_sp = feeder_encode->total_ecd + error_angle_change / 360.0f * GEAR_BOX * CAN_ENCODER_RANGE;
                    systime_t error_start_time = chVTGetSystemTime();
                    while ( chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) ){
                        turn_angle(error_angle_sp);
                        feeder_canUpdate();
                        chThdSleepMilliseconds(1);
                    }
                    single_start_time = chVTGetSystemTime();
                }

                if(feeder_encode[FEEDER_INDEX].total_ecd - angle_sp > -10 && feeder_encode[FEEDER_INDEX].total_ecd - angle_sp < 10){
                    chThdSleepMilliseconds(10);
                    if(feeder_encode[FEEDER_INDEX].total_ecd - angle_sp > -10 && feeder_encode[FEEDER_INDEX].total_ecd - angle_sp < 10){
                        single_start = 0;break;
                    }
                }
                if(feeder_encode[FEEDER_INDEX].total_ecd - angle_sp > -36 && feeder_encode[FEEDER_INDEX].total_ecd - angle_sp < 36){
                    chThdSleepMilliseconds(10);
                    if(feeder_encode[FEEDER_INDEX].total_ecd - angle_sp > -36 && feeder_encode[FEEDER_INDEX].total_ecd - angle_sp < 36){
                        single_start = 0;break;
                    }
                }

                turn_angle(angle_sp);
                feeder_canUpdate();
                chThdSleepMilliseconds(1);
            }
            break;
        }
        case FEEDER_LONG:
            if(speed_sp > 0){
                if (speed_sp - feeder_encode[FEEDER_INDEX].raw_speed > speed_sp*0.9f){
                    error_count++;
                    if (error_count > 200){
                        error_angle_change = -360.0f / 36.0f;
                        float error_angle_sp = feeder_encode->total_ecd + error_angle_change / 360.0f * GEAR_BOX * CAN_ENCODER_RANGE;
                        systime_t error_start_time = chVTGetSystemTime();
                        while ( chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) ){
                            turn_angle(error_angle_sp);
                            feeder_canUpdate();
                            chThdSleepMilliseconds(1);
                        }
                        error_count = 0;
                    }
                }
            }
            if(speed_sp < 0){
                if (speed_sp - feeder_encode[FEEDER_INDEX].raw_speed < speed_sp*0.9f){
                    error_count++;
                    if(error_count > 200){
                        error_angle_change = 360.0f / 36.0f;
                        float error_angle_sp = feeder_encode->total_ecd + error_angle_change / 360.0f * GEAR_BOX * CAN_ENCODER_RANGE;
                        systime_t error_start_time = chVTGetSystemTime();
                        while ( chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) ){
                            turn_angle(error_angle_sp);
                            feeder_canUpdate();
                            chThdSleepMilliseconds(1);
                        }
                        error_count = 0;
                    }
                }
            }
            set_speed = PID_VEL(speed_sp);
            feeder_canUpdate();
            break;
        default:
            feeder_canStop();
            break;
    }
}

static const FEEDER_VEL = "FEEDER_VEL";
static const FEEDER_POS = "FEEDER_POS";
static const char subname_feeder_PID[] = "KP KI KD";
void feederInit(void){


    feeder_encode = can_getChassisMotor();
    p_dbus = RC_get();

    params_set(&vel_pid, 14,4,FEEDER_VEL,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&pos_pid, 15,4,FEEDER_POS,subname_feeder_PID,PARAM_PUBLIC);


    chThdCreateStatic(feeder_control_wa, sizeof(feeder_control_wa),
                      NORMALPRIO+1, feeder_control, NULL);
}
