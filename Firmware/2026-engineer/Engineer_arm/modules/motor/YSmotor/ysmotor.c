/**
 * @file ysmotor.c
 * @author lh
 * @brief  宇树8010电机驱动
 * @version beta
 * @date 2025-2-14
 * @todo 
 * 
 */

#include "ysmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "crc_ccitt.h"

#define SATURATE(_IN, _MIN, _MAX) {\
	if ((_IN) < (_MIN))\
		(_IN) = (_MIN);\
	else if ((_IN) > (_MAX))\
		(_IN) = (_MAX);\
}

static uint8_t idx;
static YSMotorInstance *ys_motor_instance[YS_MOTOR_CNT];
static osThreadId ys_task_handle[YS_MOTOR_CNT];


static void YSMotorLostCallback(void *motor_ptr)
{
    //YSMotorInstance *motor = (YSMotorInstance *)motor_ptr;
    LOGWARNING("[YSMotor] motor lost");
}

/**
 * @brief 将发送给电机的浮点参数转换为标准数据帧
 *
 * @param motor_send_mailbox M8010电机指令发送结构体
 */
static void YSMotorModifyData(YS_Motor_Send_s *motor_send_mailbox)
{
	motor_send_mailbox->motor_send_data.head[0] = 0xFE;
	motor_send_mailbox->motor_send_data.head[1] = 0xEE;

	SATURATE(motor_send_mailbox->id,   0, 			15);
	SATURATE(motor_send_mailbox->mode, 0, 			7);
	SATURATE(motor_send_mailbox->K_P,  0.0f, 		25.599f);
	SATURATE(motor_send_mailbox->K_W,  0.0f, 		25.599f);
	SATURATE(motor_send_mailbox->torque,   -127.99f, 	127.99f);
	SATURATE(motor_send_mailbox->speed,   -804.00f, 	804.00f);
	SATURATE(motor_send_mailbox->angle, -411774.0f, 	411774.0f);

	motor_send_mailbox->motor_send_data.mode.id 		= motor_send_mailbox->id;
	motor_send_mailbox->motor_send_data.mode.mode 	    = motor_send_mailbox->mode;
	motor_send_mailbox->motor_send_data.cmd.k_pos 	    = motor_send_mailbox->K_P / 25.6f * 32768.0f;
	motor_send_mailbox->motor_send_data.cmd.k_spd 	    = motor_send_mailbox->K_W / 25.6f * 32768.0f;
	motor_send_mailbox->motor_send_data.cmd.angle_des 	= motor_send_mailbox->angle / 6.28318f * 32768.0f;
	motor_send_mailbox->motor_send_data.cmd.spd_des 	= motor_send_mailbox->speed / 6.28318f * 256.0f;
	motor_send_mailbox->motor_send_data.cmd.tor_des 	= motor_send_mailbox->torque * 256.0f;
	motor_send_mailbox->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_send_mailbox->motor_send_data, sizeof(motor_send_mailbox->motor_send_data) - 2);
}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的usart_instance
 */
static void YSMotorDecode(USARTInstance *_instance)
{
    YSMotorInstance *motor = (YSMotorInstance *)_instance->id;
    DaemonReload(motor->motor_daemon); // 喂狗
    /*------以上仅为喂狗操作，与数据解包无关------*/

    // 如果此帧数据错误，则读出的id也是错误的。但也无妨，因为后面有数据校验，若校验不通过则不会使用这一帧数据
    uint8_t id = _instance->recv_buff[2] & 0x0F;    // 首先读出电机id
    motor = ys_motor_instance[id];                  // 通过id找到对应的电机实例
    YS_Motor_Measure_s *motor_measure = &(motor->motor_measure);
    memcpy(&motor_measure->motor_recv_data , _instance->recv_buff , sizeof(motor_measure->motor_recv_data));
   
	if (motor_measure->motor_recv_data.head[0] != 0xFD || motor_measure->motor_recv_data.head[1] != 0xEE)
	{
		motor_measure->correct = 0;
		return;
	}
	if(motor_measure->motor_recv_data.CRC16 !=
		crc_ccitt(0, (uint8_t *)&motor_measure->motor_recv_data, sizeof(motor_measure->motor_recv_data) - 2)){
		motor_measure->correct = 0;
		return;
	}
    else
    {
        motor_measure->motor_id     = motor_measure->motor_recv_data.mode.id;
        motor_measure->mode         = motor_measure->motor_recv_data.mode.mode;
        motor_measure->temp         = motor_measure->motor_recv_data.fbk.temp;
        motor_measure->MError       = motor_measure->motor_recv_data.fbk.MError;
        motor_measure->speed        = ((float)motor_measure->motor_recv_data.fbk.speed / 256.0f) * 6.28318f ;
        motor_measure->torque       = ((float)motor_measure->motor_recv_data.fbk.torque) / 256.0f;
        motor_measure->angle        = 6.28318f * ((float)motor_measure->motor_recv_data.fbk.angle) / 32768.0f;
        motor_measure->footForce    = motor_measure->motor_recv_data.fbk.force;
        motor_measure->correct      = 1;
        return;
    }
}

/**
 * @brief M8010电机初始化
 *
 * @param config 电机初始化配置
 * @return YSMotorInstance* 注册电机实例指针
 */
YSMotorInstance *YSMotorInit(RS485_Motor_Init_Config_s *config)
{
    YSMotorInstance *motor = (YSMotorInstance *)malloc(sizeof(YSMotorInstance));
    memset(motor, 0, sizeof(YSMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    motor->motor_send_mailbox.id = config->id;  // 电机id
    // motor controller init 电机控制器初始化
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    // 仅第一个电机初始化时注册usart_instance和daemon，后续电机初始化时不再注册，通过反馈帧id分辨不同电机
    if (!idx)
    {
        config->rs485_init_config.module_callback = YSMotorDecode;
        config->rs485_init_config.recv_buff_size = 16;
        config->rs485_init_config.id = motor;
        motor->motor_rs485_ins = USARTRegister(&config->rs485_init_config);

        Daemon_Init_Config_s conf = {
            .callback = YSMotorLostCallback,
            .owner_id = motor,
            .reload_count = 10,
        };
        motor->motor_daemon = DaemonRegister(&conf);
    }

    ys_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 电机参数配置
 *
 * @param *motor 电机实例
 */
// void YSMotorConfig(YSMotorInstance *motor ,unsigned short _id, unsigned short _mode, float _torque, float _angle, float _K_P, float _K_W)
// {
//     motor->motor_send_mailbox.id = _id;
//     motor->motor_send_mailbox.mode = _mode;
//     motor->motor_send_mailbox.K_P = _K_P;
//     motor->motor_send_mailbox.K_W = _K_W;
//     motor->motor_send_mailbox.torque = _torque;
//     motor->motor_send_mailbox.angle = _angle;
//     //motor->motor_send_mailbox.speed = _speed;
// }

void YSMotorSetTorque(YSMotorInstance *motor, float _torque)
{
    motor->motor_send_mailbox.torque = _torque;
}
void YSMotorSetSpeed(YSMotorInstance *motor, float _speed)
{
    motor->motor_send_mailbox.speed = _speed;
}
void YSMotorSetAngle(YSMotorInstance *motor, float _angle)
{
    motor->motor_send_mailbox.angle = _angle;
}
void YSMotorSetCoefficient(YSMotorInstance *motor, float _K_P, float _K_W)
{
    motor->motor_send_mailbox.K_P = _K_P;
    motor->motor_send_mailbox.K_W = _K_W;
}

void YSMotorSetMode(YSMotorInstance *motor, YS_Motor_Mode_e _mode)
{
    motor->motor_send_mailbox.mode = _mode;
}

void YSMotorStop(YSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void YSMotorEnable(YSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void YSMotorSetRef(YSMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}


/**
 * @brief 电机控制任务,为所有电机实例计算PID,发送控制报文
 * @param argument 传入的电机指针
 */
__attribute__((noreturn)) void YSMotorTask(void const *argument)
{
    uint8_t group = 0;                                      // 电机组号
    int16_t set;                                            // 输出值
    float pid_measure, pid_ref;                             // 电机PID测量值和设定值
    YSMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting;                 // 电机控制参数
    YS_Motor_Measure_s *measure;                            // 电机返回的测量值
    Motor_Controller_s *motor_controller;                   // 电机控制器


    while (1)
    {
        // 一次任务调用只计算发送一个电机的控制数据，然后交替遍历两个电机组
        if (group == 0)
        {
            motor = ys_motor_instance[group];                   // 电机实例
            motor_setting = &motor->motor_settings;             // 电机控制参数
            measure = &motor->motor_measure;                    // 电机返回的测量值
            motor_controller = &motor->motor_controller;        // 电机控制器

            group = 1;
        }
        else
        {
            motor = ys_motor_instance[group];                   // 电机实例
            motor_setting = &motor->motor_settings;             // 电机控制参数
            measure = &motor->motor_measure;                    // 电机返回的测量值
            motor_controller = &motor->motor_controller;        // 电机控制器

            group = 0;
        }

        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
       
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        // if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
        //     pid_ref += *motor_controller->current_feedforward_ptr;
        // if (motor_setting->close_loop_type & CURRENT_LOOP)
        // {
        //     pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        // }

        // @todo: 后续可增加力矩环

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;
        // YSMotorSetSpeed(motor , pid_ref);   // speed由位置环输出值给定
        YSMotorSetTorque(motor, pid_ref);

        // @todo: 不同情况的模式配置

        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将发送置零
            YSMotorSetTorque(motor, 0);
            YSMotorSetSpeed(motor, 0);
            YSMotorSetAngle(motor, measure->angle);
            YSMotorSetCoefficient(motor, 0, 0);
            YSMotorSetMode(motor, M8010_MODE_FOC);

            motor_controller->angle_PID.Iout = 0.0f;
            motor_controller->speed_PID.Iout = 0.0f;
        }

        // 设置发送报文的值
        YSMotorModifyData(&motor->motor_send_mailbox);

        // 仅第一个电机注册了usart_instance，数据发送都调用第一个电机的usart_instance，后续电机通过反馈帧id分辨不同电机
        // 阻塞,等待上一次发送或接收完成
        while(!USARTIsReady(ys_motor_instance[0]->motor_rs485_ins));  
        // 发送报文
        USARTSend(ys_motor_instance[0]->motor_rs485_ins , (uint8_t *)&motor->motor_send_mailbox.motor_send_data , sizeof(motor->motor_send_mailbox.motor_send_data) , USART_TRANSFER_DMA);

        osDelay(1);
    }
}




/***********************************************************************************************************************
 * @brief 电机控制任务初始化
 **********************************************************************************************************************/
void YSMotorControlInit()
{
    if (!idx)
        return;

    osThreadDef(ysmotor, YSMotorTask, osPriorityNormal, 0, 128);
    ys_task_handle[0] = osThreadCreate(osThread(ysmotor), NULL);
}