#include "app_bluetooth.h"
#include "bsp_bluetooth.h"
#include "app_control.h"

#define FORWARD_VELOCITY    10.0f
#define ROTATION_VELOCITY   6.0f

uint8_t accelerate = 0;
uint8_t moderate = 0;
uint8_t turn_left = 0;
uint8_t turn_right = 0;

void Bluetooth_Cmd_Pro(void)
{
    if(bt.rx_flag){
        switch(bt.cmd){
            case Forward:
                accelerate = 1;
                break;
            
            case Backward:
                moderate = 1;
                break;
            
            case Left_Rot:
                sys.turn_sta = FREEDOM;
                turn_left = 1;
                break;
            
            case Right_Rot:
                sys.turn_sta = FREEDOM;
                turn_right = 1;
                break;
            
            case Self_Ctrl:
                Velocity.target = 0;
                Dir.target = sys.Yaw;
                BT_State_Clear();
                break;
            
            case Left_Rot_45:
                Dir.target += 45;
                if(Dir.target>180){
                    Dir.target = Dir.target - 360;
                }
            break;
            case Right_Rot_45:
                 Dir.target -= 45;
                if(Dir.target<-180){
                    Dir.target = Dir.target + 360;
                }               
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
        sys.turn_sta = FREEDOM;
        Turn.target+=0.01f;
        Turn.target = (Turn.target>ROTATION_VELOCITY) ? (ROTATION_VELOCITY):(Turn.target);
    }
    else if(turn_right==1){
        sys.turn_sta = FREEDOM;
        Turn.target-=0.01f;
        Turn.target = (Turn.target<-ROTATION_VELOCITY) ? (-ROTATION_VELOCITY):(Turn.target);
    }
}

void BT_State_Clear(void)
{
    accelerate = 0;
    moderate = 0;
    turn_left = 0;
    turn_right = 0;
    Velocity.target = 0;
    Dir.target = sys.Yaw;
    Turn.target = 0;
    sys.turn_sta = ANGLE;
}


