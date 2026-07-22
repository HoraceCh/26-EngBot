/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
//static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    //RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_line[3]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
static Graph_Data_t UI_pump_state[3];

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    // UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    // UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    // UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    // UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    // UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    // UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // 绘制车辆状态标志指示
    // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    // UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    // UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    // UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    // UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    // UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    // UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    // UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    // UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    // UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 底盘功率显示，静态
    // UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    // UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    // UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    // UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    // 气泵状态初始化
    UICircleDraw(&UI_pump_state[0], "pp0", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1550, 750, 20);
    UICircleDraw(&UI_pump_state[1], "pp1", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1500, 700, 20);
    UICircleDraw(&UI_pump_state[2], "pp2", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1600, 700, 20);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[0]);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_pump_state[1], UI_pump_state[2]);
    // 自定义控制器静态ui
    UICharDraw(&UI_State_sta[0], "cc0", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 750, "ctrl:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_dyn[0], "cc1", UI_Graph_ADD, 2, UI_Color_Yellow, 15, 2, 230, 750, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    // 图传跟随静态ui
    UICharDraw(&UI_State_sta[1], "cc2", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 700, "img:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_dyn[1], "cc3", UI_Graph_ADD, 2, UI_Color_Yellow, 15, 2, 210, 700, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    // 底盘水平线
    UILineDraw(&UI_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 5, 710, 240, 1210, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_line[0]);
    // 车身前进线
    UILineDraw(&UI_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 810, 540, 710, 40);
    UILineDraw(&UI_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1160, 540, 1460, 40);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_line[1], UI_line[2]);

}

void UIReset(Referee_Interactive_info_t *_Interactive_data)
{
        // 气泵状态初始化
        if (_Interactive_data->pump_state & 0b00001)
        {
            UICircleDraw(&UI_pump_state[0], "pp0", UI_Graph_ADD, 1, UI_Color_Purplish_red, 8, 1550, 750, 20);
        }
        else
        {
            UICircleDraw(&UI_pump_state[0], "pp0", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1550, 750, 20);
        }
        if (_Interactive_data->pump_state & 0b00010)
        {
            UICircleDraw(&UI_pump_state[1], "pp1", UI_Graph_ADD, 1, UI_Color_Purplish_red, 8, 1500, 700, 20);
        }
        else
        {
            UICircleDraw(&UI_pump_state[1], "pp1", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1500, 700, 20);
        }
        if (_Interactive_data->pump_state & 0b00100)
        {
            UICircleDraw(&UI_pump_state[2], "pp2", UI_Graph_ADD, 1, UI_Color_Purplish_red, 8, 1600, 700, 20);
        }
        else
        {
            UICircleDraw(&UI_pump_state[2], "pp2", UI_Graph_ADD, 1, UI_Color_Yellow, 8, 1600, 700, 20);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[0]);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_pump_state[1], UI_pump_state[2]);
    

    
        // 自定义控制器静态ui
        if (_Interactive_data->user_state & 0b100)
        {
            UICharDraw(&UI_State_sta[0], "cc0", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 750, "ctrl:");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
            UICharDraw(&UI_State_dyn[0], "cc1", UI_Graph_ADD, 2, UI_Color_Purplish_red, 15, 2, 230, 750, "on ");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        }
        else
        {
            UICharDraw(&UI_State_sta[0], "cc0", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 750, "ctrl:");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
            UICharDraw(&UI_State_dyn[0], "cc1", UI_Graph_ADD, 2, UI_Color_Yellow, 15, 2, 230, 750, "off");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
       }
    
        // 图传跟随静态ui
        if (_Interactive_data->user_state & 0b010)
        {
            UICharDraw(&UI_State_sta[1], "cc2", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 700, "img:");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
            UICharDraw(&UI_State_dyn[1], "cc3", UI_Graph_ADD, 2, UI_Color_Purplish_red, 15, 2, 210, 700, "on ");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        }
        else
        {
            UICharDraw(&UI_State_sta[1], "cc2", UI_Graph_ADD, 2, UI_Color_Green, 15, 2, 150, 700, "img:");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
            UICharDraw(&UI_State_dyn[1], "cc3", UI_Graph_ADD, 2, UI_Color_Yellow, 15, 2, 210, 700, "off");
            UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        }
        
    // 底盘水平线
    UILineDraw(&UI_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 5, 710, 240, 1210, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_line[0]);
    // 车身前进线
    UILineDraw(&UI_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 810, 540, 710, 40);
    UILineDraw(&UI_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1160, 540, 1460, 40);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_line[1], UI_line[2]);       
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
// {
//     count++;
//     if (count >= 50)
//     {
//         count = 0;
//         count1++;
//     }
//     switch (count1 % 4)
//     {
//     case 0:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
//         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
//         _Interactive_data->shoot_mode = SHOOT_ON;
//         _Interactive_data->friction_mode = FRICTION_ON;
//         _Interactive_data->lid_mode = LID_OPEN;
//         _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
//         if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
//             _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
//         break;
//     }
//     case 1:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_ROTATE;
//         _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
//         _Interactive_data->shoot_mode = SHOOT_OFF;
//         _Interactive_data->friction_mode = FRICTION_OFF;
//         _Interactive_data->lid_mode = LID_CLOSE;
//         break;
//     }
//     case 2:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
//         _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
//         _Interactive_data->shoot_mode = SHOOT_ON;
//         _Interactive_data->friction_mode = FRICTION_ON;
//         _Interactive_data->lid_mode = LID_OPEN;
//         break;
//     }
//     case 3:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
//         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
//         _Interactive_data->shoot_mode = SHOOT_OFF;
//         _Interactive_data->friction_mode = FRICTION_OFF;
//         _Interactive_data->lid_mode = LID_CLOSE;
//         break;
//     }
//     default:
//         break;
//     }
// }

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    // if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    // {
    //     switch (_Interactive_data->chassis_mode)
    //     {
        // case CHASSIS_ZERO_FORCE:
        //     UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
        //     break;
        // case CHASSIS_ROTATE:
        //     UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
        //     // 此处注意字数对齐问题，字数相同才能覆盖掉
        //     break;
        // case CHASSIS_NO_FOLLOW:
        //     UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "nofollow ");
        //     break;
        // case CHASSIS_FOLLOW_GIMBAL_YAW:
        //     UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
        //     break;
        // }
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    // }

    /*------ 气泵状态 ------*/
    if ((_Interactive_data->pump_state & 0b00001) && (_Interactive_data->Referee_Interactive_Flag.pump0_flag == 0))
    {
        UICircleDraw(&UI_pump_state[0], "pp0", UI_Graph_Change, 1, UI_Color_Purplish_red, 8, 1550, 750, 20);
        _Interactive_data->Referee_Interactive_Flag.pump0_flag = 1;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[0]);
    }
    else if ((!(_Interactive_data->pump_state & 0b00001)) && (_Interactive_data->Referee_Interactive_Flag.pump0_flag == 1))
    {
        UICircleDraw(&UI_pump_state[0], "pp0", UI_Graph_Change, 1, UI_Color_Yellow, 8, 1550, 750, 20);
        _Interactive_data->Referee_Interactive_Flag.pump0_flag = 0;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[0]);
    }


    if ((_Interactive_data->pump_state & 0b00100) && (_Interactive_data->Referee_Interactive_Flag.pump1_flag == 0))
    {
        UICircleDraw(&UI_pump_state[1], "pp1", UI_Graph_Change, 1, UI_Color_Purplish_red, 8, 1500, 700, 20);
        _Interactive_data->Referee_Interactive_Flag.pump1_flag = 1;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[1]);
    }
    else if ((!(_Interactive_data->pump_state & 0b00100)) && (_Interactive_data->Referee_Interactive_Flag.pump1_flag == 1))
    {
        UICircleDraw(&UI_pump_state[1], "pp1", UI_Graph_Change, 1, UI_Color_Yellow, 8, 1500, 700, 20);
        _Interactive_data->Referee_Interactive_Flag.pump1_flag = 0;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[1]);
    }


    if ((_Interactive_data->pump_state & 0b00010) && (_Interactive_data->Referee_Interactive_Flag.pump2_flag == 0))
    {
        UICircleDraw(&UI_pump_state[2], "pp2", UI_Graph_Change, 1, UI_Color_Purplish_red, 8, 1600, 700, 20);        
        _Interactive_data->Referee_Interactive_Flag.pump2_flag = 1;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[2]);
    }
    else if ((!(_Interactive_data->pump_state & 0b00010)) && (_Interactive_data->Referee_Interactive_Flag.pump2_flag == 1))
    {
        UICircleDraw(&UI_pump_state[2], "pp2", UI_Graph_Change, 1, UI_Color_Yellow, 8, 1600, 700, 20);        
        _Interactive_data->Referee_Interactive_Flag.pump2_flag = 0;
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pump_state[2]);
    }

    /*------ 图传跟随 ------*/
    if ((_Interactive_data->user_state & 0b010) && (_Interactive_data->Referee_Interactive_Flag.img_flag == 0))
    {
        UICharDraw(&UI_State_dyn[1], "cc3", UI_Graph_Change, 2, UI_Color_Purplish_red, 15, 2, 210, 700, "on ");
        _Interactive_data->Referee_Interactive_Flag.img_flag = 1;
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    }
    else if ((!(_Interactive_data->user_state & 0b010)) && (_Interactive_data->Referee_Interactive_Flag.img_flag == 1))
    {
        UICharDraw(&UI_State_dyn[1], "cc3", UI_Graph_Change, 2, UI_Color_Yellow, 15, 2, 210, 700, "off");
        _Interactive_data->Referee_Interactive_Flag.img_flag = 0;
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    }

    /*------ 自定义控制器开关 ------*/
    if ((_Interactive_data->user_state & 0b100) && (_Interactive_data->Referee_Interactive_Flag.ctrl_flag == 0))
    {
        UICharDraw(&UI_State_dyn[0], "cc1", UI_Graph_Change, 2, UI_Color_Purplish_red, 15, 2, 230, 750, "on ");
        _Interactive_data->Referee_Interactive_Flag.ctrl_flag = 1;
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    }
    else if ((!(_Interactive_data->user_state & 0b100)) && (_Interactive_data->Referee_Interactive_Flag.ctrl_flag == 1))
    {
        UICharDraw(&UI_State_dyn[0], "cc1", UI_Graph_Change, 2, UI_Color_Yellow, 15, 2, 230, 750, "off");
        _Interactive_data->Referee_Interactive_Flag.ctrl_flag = 0;
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    //     _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    // }

    // if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    //     _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    // }

    // if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    //     _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    // }

    // if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    //     _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    // }

    // if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
    //     _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    // }

    // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    //     _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    // }

    // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    //     _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    // }

}
