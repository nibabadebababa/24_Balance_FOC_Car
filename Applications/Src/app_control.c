#include "app_control.h"
#include "bsp_esp32.h"
#include "stdio.h"

#define     PICKUP_P_MAX        8       // 检测拿起的角度范围 ±9°
#define     PICKUP_V_MIN        90      // 检测拿起的速度范围 ±    
#define     FALLING_PITCH_P     60      // 检测倒地的角度正边界值
#define     FALLING_PITCH_N     -60     // 检测倒地的角度负边界值
#define     LANDING_PITCH_P     5       // 检测重新启动的Pitch角正边界值
#define     LANDING_PITCH_N     -5      // 检测重新启动的Pitch角负边界值
#define     LANDING_ROLL_P      5       // 检测重新启动的Roll角正边界值
#define     LANDING_ROLL_N      -5      // 检测重新启动的Roll角负边界值
#define     MED_ANGLE           5.85    // 小车的机械中值
#define     MED_PIXEL_POSITION  (9.60)  // Yolov5目标方向像素中值
#define     YOLO_RECT_SIZE      (400)   // YOlov5目标矩阵框大小

PID_TYPE_DEF  Balance ;         // 直立环
PID_TYPE_DEF  Velocity;         // 速度环
PID_TYPE_DEF  Turn    ;         // 转向环
PID_TYPE_DEF  Location;         // 位置环
PID_TYPE_DEF  Dir     ;         // 方向环
PID_TYPE_DEF  FollowLoc  ;      // 跟随位置环
PID_TYPE_DEF  FollowDir  ;      // 跟随方向环

uint8_t     is_first_in = 1; // 记录第一次进入位置环时的位置(标志位)


void PID_Init(void)
{
    /* 直立环参数 */
    Balance.Kp =2.46;        // 1.86f     极性正
    Balance.Ki = 0;
    Balance.Kd = 2.26;       // 2.16f     极性正
    Balance.I  = 0;
    Balance.I_MAX = 0;
    Balance.outMAX = 140.0f;
    Balance.outMIN = -140.0f;
    Balance.target = MED_ANGLE;
    
    /* 速度环参数 */
    Velocity.Kp = 1.25;       // 1.25
    Velocity.Ki = Velocity.Kp/200.0f;
    Velocity.Kd = 0;
    Velocity.I = 0;
    Velocity.I_MAX =  500.0f;
    Velocity.outMAX = 100.0f;
    Velocity.outMIN = -100.0f;
    Velocity.target = 0;

    /* 方向环参数 */
    Dir.Kp = 0.5;     // 0.5      极性正
    Dir.Ki = Dir.Kp/200.0f;
    Dir.Kd = -0.54;   // -0.54    极性负
    Dir.I = 0;
    Dir.I_MAX = 1000.0f;
    Dir.outMAX = 40.0f;   
    Dir.outMIN = -40.0f;
    Dir.target = sys.Yaw;

    /* 转向环参数 */
    Turn.Kp = 5;//0.2
    Turn.Ki = 0;
    Turn.Kd = -0.74;
    Turn.outMAX = 60.0f;
    Turn.outMIN = -60.0f;
    Turn.target = 0;

    /* 位置环参数 */
    Location.Kp = 1.6;  // 2 极性正
    Location.Ki = Location.Kp/200;
    Location.Kd = 10;
    Location.I = 0;
    Location.I_MAX = 1000;
    Location.outMAX = 15;
    Location.outMIN = -15;
    Location.target = 0;
    
    /* 跟随位置环参数 */
    FollowLoc.Kp = 0.15;
    FollowLoc.Ki = 0;
    FollowLoc.Kd = 10;
    FollowLoc.I = 0;
    FollowLoc.I_MAX = 0;
    FollowLoc.out = 0;
    FollowLoc.outMAX = 13;
    FollowLoc.outMIN = -13;
    FollowLoc.target = YOLO_RECT_SIZE;
    
    /* 跟随方向环参数 */
    FollowDir.Kp = 1.3;   // 4
    FollowDir.Ki = 0;
    FollowDir.Kd = -0.1;
    FollowDir.I = 0;
    FollowDir.I_MAX = 0;
    FollowDir.out = 0;
    FollowDir.outMAX = 40;
    FollowDir.outMIN = -40;
    FollowDir.target = MED_PIXEL_POSITION;
}

void PID_Control_Update(void)
{
    /* 1.判断当前的速度控制模式是否处于位置闭环控制模式 */
    if(sys.veloc_sta == LOCATION_CTRL ){
        if(is_first_in){
            Location.target = (sys.S0-sys.S1)/2;
            is_first_in = 0;
        }
        /* 2.计算位置环 */
        Location.out = Location_PID_Calcu(Location.target, (sys.S0-sys.S1)/2, sys.Ax);
        /* 3.位置环控制速度 */
        Velocity.target = Location.out;
    }
    else if(sys.veloc_sta == YOLO_FOLLOW_CTRL){
        if(sys.yolo_flag==1){
            /* 2.计算跟随位置环 */
            FollowLoc.target = YOLO_RECT_SIZE;
            FollowLoc.out = FollowLoc_PID_Calcu(FollowLoc.target, (sys.yolo.height*sys.yolo.width)/100.0f, sys.Ax);
            /* 3.跟随位置环控制速度 */
            Velocity.target = FollowLoc.out;            
        }
        else
            Velocity.target = 0;
    }
    else if(sys.veloc_sta == BT_CTRL){
        /* 2.蓝牙控制速度 */
        is_first_in = 1;
    }
    
    /* 4.计算速度环 */
    Velocity.out = Velocity_PID_Calcu(Velocity.target, sys.V0-sys.V1);

    /* 5.计算直立环 */
    Balance.out = Balance_PID_Calcu(Balance.target, sys.Pitch, sys.Gy);

    /* 6.判断当前是角度闭环，还是自由转向 */
    if(sys.turn_sta == ANGLE_CTRL){
        /* 7.计算方向环 */
        Dir.out = Dir_PID_Calcu(Dir.target, sys.Yaw, sys.Gz);     
        sys.Set_V0 =  Balance.out - Velocity.out + Dir.out;
        sys.Set_V1 =  Balance.out - Velocity.out - Dir.out;        
    }
    else if(sys.turn_sta == BT_TURN_CTRL){
        /* 7.计算转向环 */
        Turn.out = Turn_PID_Calcu(Turn.target, sys.Gz);   
        sys.Set_V0 =  Balance.out - Velocity.out + Turn.out;
        sys.Set_V1 =  Balance.out - Velocity.out - Turn.out;        
    } 
    else if(sys.turn_sta == YOLO_TURN_CTRL){
        /* 7.计算目标跟随转向环 */
        if(sys.yolo_flag){
            FollowDir.target = MED_PIXEL_POSITION;
            FollowDir.out = FollowDir_PID_Calcu(FollowDir.target, sys.yolo.x_offset/100.0f, sys.Gz);           
        } else{
            FollowDir.out = 0;
        }
        sys.Set_V0 =  Balance.out - Velocity.out + FollowDir.out;
        sys.Set_V1 =  Balance.out - Velocity.out - FollowDir.out; 
    }
    if(sys.yolo_flag)
        printf("%.2f,%.2f,%.2f,%.2f\n",FollowDir.out, FollowLoc.out, Velocity.out, (sys.yolo.height*sys.yolo.width)/100.0f);

    //printf("%.2f,%.2f,%.2f,%.2f\n",Location.out, Velocity.out, (sys.S0-sys.S1)/2, Location.Kd*sys.Az); // 位置环Debug输出
    //printf("%.1f,%.1f,%.1f,%.1f,%.1f\n", sys.V0, sys.V1, sys.Set_V0, sys.Set_V1,Velocity.I);
    //printf("%.1f,%.1f,%.1f\n",Velocity.out, Balance.out, sys.Set_V0);
    //printf("%.1f,%.1f,%.1f,%.1f\n",Dir.target,sys.Yaw, sys.Gz, sys.Set_V0); //方向环Debug输出
    //printf("%.1f\n",Turn.out);    // 转向环Debug输出
    
    /* 输出到电机的扭矩 */
    Set_Motor_Torque(MOTOR0, sys.Set_V0);
    Set_Motor_Torque(MOTOR1, sys.Set_V1);
}

float Balance_PID_Calcu(float target_angle, float current_angle, float gyro)
{
    float PID_out = 0;

    PID_out = Balance.Kp*(target_angle - current_angle) + Balance.Kd*(gyro);
    
    /* 直立环限幅 */
    PID_out =   ( PID_out > Balance.outMAX) ? (Balance.outMAX): \
                ((PID_out < Balance.outMIN) ? (Balance.outMIN): PID_out);

    return PID_out;
}

float Velocity_PID_Calcu(float target_v, float current_v)
{
    static float Err_Lowout_cur, Err_Lowout_pre;  // 低通滤波器当前输出与上轮输出
    float Error;
    float PID_out = 0;
    float a = 0.7;

    Error = target_v - current_v; // 期望速度 - 当前速度
    Err_Lowout_cur = (1-a)*Error + a*Err_Lowout_pre; // 低通滤波器，数据更平滑
    Err_Lowout_pre = Err_Lowout_cur;                 // 记录当前低通滤波的输出作为下一次数据

    Velocity.I += Err_Lowout_cur;  // 对误差进行积分

    /* 积分限幅 */
    Velocity.I = (Velocity.I > Velocity.I_MAX) ? (Velocity.I_MAX):  \
      ( (Velocity.I < -Velocity.I_MAX) ? (-Velocity.I_MAX) : (Velocity.I) );

    /* 计算PID输出 */
    PID_out = Velocity.Kp*Err_Lowout_cur + Velocity.Ki*Velocity.I;

    /* 速度环限幅 */
    PID_out =   ( PID_out > Velocity.outMAX) ? (Velocity.outMAX): \
                ((PID_out < Velocity.outMIN) ? (Velocity.outMIN):PID_out);
    
    return PID_out;
}

float Dir_PID_Calcu(float target_yaw, float current_yaw, float gyro_z)
{
    static float Err_Lowout_cur = 0, Err_Lowout_pre = 0;    // 低通滤波后的误差值
    float a = 0.7;      // 低通滤波器系数
    float Error,PID_out = 0;
     
    if(target_yaw <= -90 && current_yaw>=90){
        Error = (target_yaw + 360) - current_yaw;
    }
    else if(target_yaw >= 90 && current_yaw <= -90){
        Error = target_yaw - (360 + current_yaw);
    }
    else{
        Error = target_yaw - current_yaw;
    }
    
    Err_Lowout_cur = (1-a)*Error + a*Err_Lowout_pre;
    Err_Lowout_pre = Err_Lowout_cur;
    
    Dir.I += Err_Lowout_cur;
    
    /* 积分限幅 */
    Dir.I = (Dir.I > Dir.I_MAX) ? (Dir.I_MAX):  \
      ( (Dir.I < -Dir.I_MAX) ? (-Dir.I_MAX) : (Dir.I) ); 
    
    /* 计算PID输出 */
    PID_out = Dir.Kp*Err_Lowout_cur + Dir.Ki*Dir.I + Dir.Kd*gyro_z;
    
    /* 方向环限幅 */
    PID_out =   ( PID_out > Dir.outMAX) ? (Dir.outMAX): \
                ((PID_out < Dir.outMIN) ? (Dir.outMIN):PID_out); 
    
    return PID_out;
}

float Turn_PID_Calcu(float RC, float gyro_Z)
{
	float PID_out = 0;
	
	PID_out= Turn.Kp*RC + Turn.Kd*gyro_Z ;
    
    /* 转向环限幅 */
    Turn.out =  ( Turn.out > Turn.outMAX) ? (Turn.outMAX): \
                ((Turn.out < Turn.outMIN) ? (Turn.outMIN):Turn.out);
    
	return PID_out;
}

float Location_PID_Calcu(float target_s, float current_s, float accel_x)
{
    static float Err_Lowout_cur = 0, Err_Lowout_pre = 0;    // 低通滤波后的误差值
    float a = 0.3;      // 低通滤波器系数
    float Error,PID_out = 0;
     
    Error = target_s - current_s;
    if(Error < 2 && Error > -2){
        /* 在正常允许的误差范围内则抑制误差，减小来回摆动 */
        Error = Error/4;
    }
    else if(Error >50 || Error < -50){
        /* 除去异常值 */
        return 0;
    }
    Err_Lowout_cur = (1-a)*Error + a*Err_Lowout_pre;
    Err_Lowout_pre = Err_Lowout_cur;
    
    Location.I += Err_Lowout_cur;
    
    /* 积分限幅 */
    Location.I = (Location.I > Location.I_MAX) ? (Location.I_MAX):  \
      ( (Location.I < -Location.I_MAX) ? (-Location.I_MAX) : (Location.I) ); 
    
    /* 计算PID输出 */
    PID_out = Location.Kp*Err_Lowout_cur + Location.Ki*Location.I + Location.Kd*accel_x;
    
    /* 输出限幅 */
    PID_out =  ( PID_out > Location.outMAX) ? (Location.outMAX): \
                ((PID_out < Location.outMIN) ? (Location.outMIN):PID_out);
    
    return PID_out;
}

float FollowLoc_PID_Calcu(float target, float current, float accel_x)
{
    float Error,PID_out = 0;    

    Error = target - current;
    /* 抑制零点误差 */
    if(Error >= -30 && Error <= 30){
        Error=Error/4;
    }
    FollowLoc.I += Error;
    
    /* 积分限幅 */
    FollowLoc.I = (FollowLoc.I > FollowLoc.I_MAX) ? (FollowLoc.I_MAX):  \
      ( (FollowLoc.I < -FollowLoc.I_MAX) ? (-FollowLoc.I_MAX) : (FollowLoc.I) );
    
    PID_out = FollowLoc.Kp*Error + FollowLoc.Ki*FollowLoc.I + FollowLoc.Kd*accel_x;

    /* 输出限幅 */
    PID_out =  ( PID_out > FollowLoc.outMAX) ? (FollowLoc.outMAX): \
                ((PID_out < FollowLoc.outMIN) ? (FollowLoc.outMIN):PID_out);
    
    return PID_out;
}

float FollowDir_PID_Calcu(float target, float current, float gyro_z)
{
    float Error,PID_out = 0;    

    Error = target - current;
    /* 抑制零点误差 */
    if(Error < 3 && Error > -3){
        Error = Error/3;
    }
    
    PID_out = FollowDir.Kp*Error + FollowDir.Kd*gyro_z;
    
    /* 输出限幅 */
    PID_out =  ( PID_out > FollowDir.outMAX) ? (FollowDir.outMAX): \
                ((PID_out < FollowDir.outMIN) ? (FollowDir.outMIN):PID_out);
    
    return PID_out;    
}


void Pick_Up_Detect(float velocity, float pitch)
{
    if((velocity > PICKUP_V_MIN || velocity < -PICKUP_V_MIN) && pitch<PICKUP_P_MAX && pitch>-PICKUP_P_MAX ){
        sys.pick_up_flag = 1;
    }
}

void Falling_Detect(float pitch)
{
    static uint16_t cur_cnt = 0, pre_cnt = 0;
    static uint16_t start_cnt_flag = 0;
    uint16_t time = 0;
    if(pitch > FALLING_PITCH_P || pitch < FALLING_PITCH_N)
    {
        if(start_cnt_flag == 0){
            pre_cnt = tim2_100ms_cnt;
            start_cnt_flag = 1;
        }
        if(start_cnt_flag){
            cur_cnt = tim2_100ms_cnt;
            time = (cur_cnt>pre_cnt)?(cur_cnt-pre_cnt):(TIM2_CNT_MAX+cur_cnt-pre_cnt);
            if(time >= 30){
                sys.falling_flag = 1;
            }            
        }

    }
    else{
        start_cnt_flag = 0;
    }
}

void Landing_Detect(void)
{
    static uint16_t cnt_100ms = 0;
    if(sys.pick_up_flag ==1 || sys.falling_flag == 1){
        if(sys.Pitch < 5 && sys.Pitch > -5 && sys.Roll< 5 && sys.Roll > -5){
            cnt_100ms++;
        }
        else 
            cnt_100ms = 0;
    }
    if(cnt_100ms >= 30){
        /* 满足重启条件 */
        sys.S_cur = (sys.S0-sys.S1)/2;  // 重新启动后，记录当前电机转过的弧度
        is_first_in = 1;
        sys.falling_flag = 0;
        sys.pick_up_flag = 0;
        cnt_100ms = 0;
    }
}


