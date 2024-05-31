#include "app_bluetooth.h"
#include "bsp_bluetooth.h"
#include "app_control.h"
#include "bsp_esp32.h"

#define FORWARD_VELOCITY    20.0f
#define ROTATION_VELOCITY   15.0f

uint8_t accelerate = 0;
uint8_t moderate = 0;
uint8_t turn_left = 0;
uint8_t turn_right = 0;

void Bluetooth_Cmd_Pro(void)
{
    if(bt.rx_flag){
        switch(bt.cmd){
            case Forward:
                Velocity.target = 0;    // 归零
                accelerate = 1;         // 加速标志位
                Dir.target = sys.Yaw;
                sys.veloc_sta = BT_CTRL;    // 蓝牙控制速度
                sys.turn_sta = ANGLE_CTRL;  // 角度闭环
                break;
            
            case Backward:
                Velocity.target = 0;        //归零
                moderate = 1;               // 减速标志位
                Dir.target = sys.Yaw;       
                sys.veloc_sta = BT_CTRL;    // 蓝牙控制速度
                sys.turn_sta = ANGLE_CTRL;  // 角度闭环
                break;
            
            case Left_Rot:
                sys.veloc_sta = BT_CTRL;    // 蓝牙控制速度
                Velocity.target = 0;        
                sys.turn_sta = BT_TURN_CTRL;// 蓝牙控制转向
                turn_left = 1;      // 左转标志位
                break;
            
            case Right_Rot:
                sys.veloc_sta = BT_CTRL;    // 蓝牙控制速度
                Velocity.target = 0;        
                sys.turn_sta = BT_TURN_CTRL;// 蓝牙控制转向
                turn_right = 1; // 右转标志位
                
                break;
            
            case Self_Ctrl:
                Velocity.target = 0;            // 自稳
                Dir.target = sys.Yaw;           // 获取当前偏航角
                Location.target = Get_Motor_S();// 获取当前里程
                sys.turn_sta = ANGLE_CTRL;      // 角度闭环
                sys.veloc_sta = LOCATION_CTRL;  // 位置闭环
                BT_State_Clear();
                break;
            
            case Left_Rot_45:
                sys.turn_sta = ANGLE_CTRL;
                Dir.target += 45;
                if(Dir.target>180){
                    Dir.target = Dir.target - 360;
                }
                
            break;
                
            case Right_Rot_45:
                sys.turn_sta = ANGLE_CTRL;
                 Dir.target -= 45;
                if(Dir.target<-180){
                    Dir.target = Dir.target + 360;
                }                      
            break;
                
            case Location_Fix:
                Location.target = Get_Motor_S(); // 获取当前位移
                sys.veloc_sta = LOCATION_CTRL;
            break;
                
            case Angle_Fix:
                Dir.target = sys.Yaw;       // 获取当前偏航角
                sys.turn_sta = ANGLE_CTRL;  // 角度闭环模式
            break;
            
            case Location_YOLO:
                sys.veloc_sta = YOLO_FOLLOW_CTRL;// 位置跟随目标
            break;
                        
            case Angle_YOLO:
                sys.turn_sta = YOLO_TURN_CTRL;  // 方向跟随目标
            break;
            
            default:
                Velocity.target = 0;
                Dir.target = sys.Yaw;   
                BT_State_Clear();            
                break;
        }
        bt.rx_flag = 0;
    }
    
    if(accelerate==1){
        ++Velocity.target;
        Velocity.target = (Velocity.target>FORWARD_VELOCITY) ? (FORWARD_VELOCITY):(Velocity.target);
    }
    else if(moderate==1){
        --Velocity.target;
        Velocity.target = (--Velocity.target<-FORWARD_VELOCITY) ? (-FORWARD_VELOCITY):(Velocity.target);
    }
    else if(turn_left==1){
        sys.turn_sta = BT_TURN_CTRL;
        Turn.target+=0.05f;
        Turn.target = (Turn.target>ROTATION_VELOCITY) ? (ROTATION_VELOCITY):(Turn.target);
    }
    else if(turn_right==1){
        sys.turn_sta = BT_TURN_CTRL;
        Turn.target-=0.05f;
        Turn.target = (Turn.target<-ROTATION_VELOCITY) ? (-ROTATION_VELOCITY):(Turn.target);
    }
}

void BT_State_Clear(void)
{
    accelerate = 0;
    moderate = 0;
    turn_left = 0;
    turn_right = 0;
    Turn.target = 0;
}


