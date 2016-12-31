#ifndef _DEF_H_
#define _DEF_H_

#define CFG_Kp 1.6
#define CFG_Ki 0.02
#define CFG_Kd 0.02

#define FIT_THROTTLE 0

#define MAX_ANGLE 30

#define MAX_ALT 120.00
#define MIN_ALT 6.00

#define MIN_INTG_ERROR -35
#define MAX_INTG_ERROR 35

#define MIN_MOTOR_ERROR -35
#define MAX_MOTOR_ERROR 35

#define YAW_DIRECTION 1

// 定义引脚编号
/*
    3   5
      X
    6   9
*/
#define MOTOR_PIN_0 3
#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 9

#define FL 0  // 左前
#define FR 1  // 右前
#define BL 2  // 左后
#define BR 3  // 右后

#define THROTTLE 180

#endif
