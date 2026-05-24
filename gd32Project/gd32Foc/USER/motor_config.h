/*!
 Created by tofly Choo on 2026/5/21.
*/

#ifndef CLION_KEIL_GD32_MOTOR_CONFIG_H
#define CLION_KEIL_GD32_MOTOR_CONFIG_H

#define MOTOR_B1630  1  // 无刷电机
#define MOTOR_B2245   2  // 无刷电机
#define MOTOR_TYPE_C  3  // 某种新测试电机C

#define CURRENT_MOTOR_SELECT   MOTOR_B2245  // <--- 核心：修改这个数字切换测试电机


// ==================== 根据选择，自动匹配对应的宏参数 ====================

#if (CURRENT_MOTOR_SELECT == MOTOR_B1630)
    #define FOC_UDC       7.0f          // 母线电压 (V)
    #define MAX_CURRENT   1.0f          // 最大工作电流 (A)
    #define MAX_SPEED     5000.0f       // 最大转速 (RPM)
    #define OFFSET_ANGLE  322.0f        // 编码器电角度偏移量 (度)
    #define NP            1             //电机极对数

    // 电流环 PID
    #define MOTOR_PID_ID_KP           0.1f
    #define MOTOR_PID_ID_KI           0.01f
    #define MOTOR_PID_IQ_KP           0.1f
    #define MOTOR_PID_IQ_KI           0.0005f

    // 速度环/位置环 PID
    #define MOTOR_PID_SPEED_KP        0.001f
    #define MOTOR_PID_SPEED_KI        0.0001f
    #define MOTOR_PID_POS_KP          1.0f

#elif (CURRENT_MOTOR_SELECT == MOTOR_B2245)
    #define FOC_UDC       12.0f
    #define MAX_CURRENT   3.0f    //实际最大堵转电流是6A
    #define MAX_SPEED     13000.0f
    #define OFFSET_ANGLE  0
    #define NP            1       //电机极对数

    // 电流环 PID
    #define MOTOR_PID_ID_KP           0
    #define MOTOR_PID_ID_KI           0
    #define MOTOR_PID_IQ_KP           0
    #define MOTOR_PID_IQ_KI           0

    // 速度环/位置环 PID
    #define MOTOR_PID_SPEED_KP        0
    #define MOTOR_PID_SPEED_KI        0
    #define MOTOR_PID_POS_KP          0

#elif (CURRENT_MOTOR_SELECT == MOTOR_TYPE_C)
    // 预留给未来测试的 C 电机...
#endif

#endif //CLION_KEIL_GD32_MOTOR_CONFIG_H
