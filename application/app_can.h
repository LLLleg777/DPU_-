#ifndef __APP_CAN
#define __APP_CAN
#include "can.h"
#include "system.h"

//rm motor data
typedef struct
{
    int16_t angle;
    int16_t speed;
    int16_t troque;
    int16_t temperate;
    int16_t offset_angle;
    int16_t last_angle;
    int16_t round_cnt;
    float total_angle;
    float real_angle;
} motor_measure_t;

void can_filter_init(void);
void can1_cmd1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void can1_cmd2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void can2_cmd1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void can2_cmd2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void get_moto_measure(motor_measure_t *ptr, uint8_t Data[]);
void get_moto_offset(motor_measure_t *ptr);
/******************************extern*********************/
extern motor_measure_t m3508[8];
extern motor_measure_t m2006[8];
#endif
