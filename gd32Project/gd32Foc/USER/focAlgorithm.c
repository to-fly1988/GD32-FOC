#include "focAlgorithm.h"
#include "foc_tim_pwm.h"
#include "motor_config.h"
#include "math.h"
#include "encoder_spi.h"

//函数声明
static void ESO_SPEED_LOOP(volatile FocStatus *foc);

/* =====FOC参数初始化=====*/
void FOC_Init(volatile FocStatus *foc){
	
	foc->ADC_FLAG=1;
	foc->ADC_CNT=0;
	foc->ia_sum=0;
	foc->ib_sum=0;
	
	foc->targetAngle=100;
  foc->targetSpeed=100;
	
	foc->target_id=0;
	foc->target_iq=0;
	
	/*PID参数初始化*/
	foc->pid_id.kp=MOTOR_PID_ID_KP;
	foc->pid_id.ki=MOTOR_PID_ID_KI;
	foc->pid_id.integral=0;
	foc->pid_id.out_limit=FOC_UDC*0.577f;		//0.577为根号三分之1	
	foc->pid_id.int_limit = FOC_UDC*0.577f*0.8f;	
	
	foc->pid_iq.kp=MOTOR_PID_IQ_KP;
	foc->pid_iq.ki=MOTOR_PID_IQ_KI;
	foc->pid_iq.integral=0;
	foc->pid_iq.out_limit=FOC_UDC*0.577f;		//0.577为根号三分之1
	foc->pid_iq.int_limit = FOC_UDC*0.577f*0.8f;	
	
	foc->pid_speed.kp = MOTOR_PID_SPEED_KP;
	foc->pid_speed.ki = MOTOR_PID_SPEED_KI;
	foc->pid_speed.integral = 0;
	foc->pid_speed.out_limit = MAX_CURRENT;
	foc->pid_speed.int_limit = MAX_CURRENT*0.8f;	
	
	foc->pid_position.kp=MOTOR_PID_POS_KP;
	foc->pid_position.ki=0;
	foc->pid_position.integral=0;
	foc->pid_position.out_limit=MAX_SPEED/1.3f;
	foc->pid_position.int_limit = 4000.0f;	
	
	/*轨迹规划初始参数*/
	foc->traj.pos_plan=0;
	foc->traj.vel_plan=0;
	foc->traj.vmax=5000;
	foc->traj.amax=800000;
	foc->traj.acc_plan=0;
	foc->TR_FLAG = 1;
	
	/*速度环eso初始参数*/
	foc->eso_speed.z1 = 0.0f;
	foc->eso_speed.z2 = 0.0f;
	foc->eso_speed.b0 = 320000.0f;      // 

	float wo = 250.0f;                // rad/s，先试 100~300
	foc->eso_speed.beta1 = 2.0f * wo;
	foc->eso_speed.beta2 = wo * wo;
	
}

/* =====坐标变换函数=====*/
static void clark_Park(volatile FocStatus *foc){
	//直接由三相静止坐标系转换为同步旋转坐标系，只由ab两相即可计算
	foc->id=foc->ia*cosf(foc->theta_e)+(foc->ia+foc->ib*2)*0.57735f*sinf(foc->theta_e);    //0.57735f为三分之根号三
	foc->iq=-foc->ia*sinf(foc->theta_e)+(foc->ia+foc->ib*2)*0.57735f*cosf(foc->theta_e);

}

static void clark_Conv(volatile FocStatus *foc){
	
	foc->i_alpha=foc->ia;
	foc->i_beta=(foc->ia+2.0f*foc->ib)*0.57735f;
	
}

static void park_Conv(volatile FocStatus *foc){

	foc->id=foc->i_alpha*cosf(foc->theta_e)+foc->i_beta*sinf(foc->theta_e);
	foc->iq=-foc->i_alpha*sinf(foc->theta_e)+foc->i_beta*cosf(foc->theta_e);
	
}

static void anti_Park(volatile FocStatus *foc){
	//通过逆帕克变换将ud,uq转为ualpha,ubeta
	foc->u_alpha=foc->ud*cosf(foc->theta_e)-foc->uq*sinf(foc->theta_e);
	foc->u_beta=foc->ud*sinf(foc->theta_e)+foc->uq*cosf(foc->theta_e);
	
}


/* ======PID计算函数======*/
static float pid_Process(volatile PidStatus *pid,float error){
	
	float out, out_unsat, temp_int;
	
	/*积分项*/
	temp_int = pid->integral + pid->ki*error;
	
	/*积分限幅*/
	if(temp_int > pid->int_limit)
	{temp_int = pid->int_limit;}
	else if(temp_int < -pid->int_limit)
	{temp_int = -pid->int_limit;}
	
	/*比例项*/
	out_unsat = pid->kp*error + temp_int;
	out = out_unsat;
	
	/*输出限幅*/
	if(out > pid->out_limit)
	{out = pid->out_limit;}
	else if(out < -pid->out_limit)
	{out = -pid->out_limit;}
	
	if((out_unsat == out) || (out >= pid->out_limit && error < 0) || (out <= -pid->out_limit && error >0) ){
		pid->integral = temp_int;
	}
	
	
	return out;

}

/* =====SVPWM生成函数=====*/
static void svpwm_Generate(volatile FocStatus *foc){

	//-------扇区判定-------
	float Vref1=foc->u_beta;
	float Vref2=(SQRT3*foc->u_alpha-foc->u_beta)*0.5f;
	float Vref3=(-SQRT3*foc->u_alpha-foc->u_beta)*0.5f;
	uint8_t sector=0;
	
	if(Vref1>0){
	sector+=1;}
	
	if(Vref2>0){
	sector+=2;}
	
	if(Vref3>0){
	sector+=4;}
	
	foc->secn=sector;
	
	float X=SQRT3*foc->u_beta*FOC_PWM_ARR/FOC_UDC;
	float Y=(1.5f*foc->u_alpha+SQRT3*foc->u_beta*0.5f)*FOC_PWM_ARR/FOC_UDC;
	float Z=(-1.5f*foc->u_alpha+SQRT3*foc->u_beta*0.5f)*FOC_PWM_ARR/FOC_UDC;
	
	//-------各扇区作用时间----------
	float T1,T2;
	switch (sector){
		case 3:		//	sector代表实际扇区的顺序是315462（12346）
			T1=-Z;T2=X;
			break;
		
		case 1:
			T1=Z;T2=Y;
			break;
		
		case 5:
			T1=X;T2=-Y;
			break;
		
		case 4:
			T1=-X;T2=Z;
			break;
		
		case 6:
			T1=-Y;T2=-Z;
			break;
		
		case 2:
			T1=Y;T2=-X;
			break;
		
		default: //异常情况
			T1=0.0f;T2=0.0f;
			break;
	}
	
	//------过调制处理--------
	if(T1+T2>FOC_PWM_ARR){
	float total=T1+T2;
	T1=T1/total*FOC_PWM_ARR;
	T2=T2/total*FOC_PWM_ARR;
	}
	
	//------计算PWM比较值CRR---
	float ta,tb,tc,Tcm1,Tcm2,Tcm3;
	ta=(FOC_PWM_ARR-(T1+T2))/4.0f;
	tb=ta+T1*0.5f;
	tc=tb+T2*0.5f;
	
	switch(sector){
		case 1:
			Tcm1=tb;
			Tcm2=ta;
			Tcm3=tc;
		  break;
		case 2:
			Tcm1=ta;
			Tcm2=tc;
			Tcm3=tb;
		  break;
		case 3:
			Tcm1=ta;
			Tcm2=tb;
			Tcm3=tc;  
			break;
		case 4:
			Tcm1=tc;
			Tcm2=tb;
			Tcm3=ta;
			break;
		case 5:
			Tcm1=tc;
			Tcm2=ta;
			Tcm3=tb;
			break;
		case 6:
			Tcm1=tb;
			Tcm2=tc;
			Tcm3=ta;
			break;
		default:
			Tcm1=FOC_PWM_ARR*0.5f;
		  Tcm2=FOC_PWM_ARR*0.5f;
		  Tcm3=FOC_PWM_ARR*0.5f;
		  break;
	}
	foc->pwm_a=(uint16_t)Tcm1;
	foc->pwm_b=(uint16_t)Tcm2;
	foc->pwm_c=(uint16_t)Tcm3;
		
}

/***饱和函数***/
static float SMC_Sat(float s, float delta){
	if(s>delta){
	return 1.0f;}
	else if(s<delta){
	return -1.0f;}
	else
	{return s/delta;}
}

/*************/

	
/***摩擦力补偿***/
static float Friction_COMP(volatile FocStatus *foc){
	  float pos_err=foc->targetAngle-foc->theta_m;
    if(fabsf(pos_err)<2.0f && fabsf(foc->speed)<1.0f && fabsf(pos_err)>0.05f){
		if(pos_err>0){
		return 0.01f;}
		else if(pos_err<0){
		return -0.01f;}
		}
			return 0;
}
/***************/

/*!
   \brief  轨迹规划器
*/
static void Trajectory_Update(volatile TRAJ_T *traj, float target_pos, float Ts, volatile FocStatus *foc  ){
	
	float dist=target_pos-traj->pos_plan;
	
	float max_dv=traj->amax*Ts;

	
	/***终点死区设置***/
//	if(foc->TR_FLAG){
	if (fabsf(dist) < 0.1f && fabsf(traj->vel_plan) <= 0.1f*max_dv && fabsf(foc->speed )<=10.0f ) {
        traj->pos_plan = target_pos; 
        traj->vel_plan = 0.0f;
				
		    traj->acc_plan = 0.0f;
//		    foc->pid_iq.integral *= 0.8f;
		
        return; 
    }
//	}
	
	float last_vel = traj->vel_plan;	
	/***计算安全速度***/
//	float max_safe_vel = sqrtf(fabsf(dist) * traj->amax / 3.0f);
  float max_safe_vel = traj->amax / 6.0f *(sqrtf(36.0f*0.001f*0.001f+12.0f*fabsf(dist)/traj->amax)-6.0f*0.001f );
	
		
	if(max_safe_vel > traj->vmax)
    {
        max_safe_vel = traj->vmax;
    }
		
	if(dist < 0.0f)
    {
        max_safe_vel = -max_safe_vel;
    }
		
		float v_err = max_safe_vel - traj->vel_plan;
		
		if(v_err > max_dv)
    {
        traj->vel_plan += max_dv; 
    }
    else if(v_err < -max_dv)
    {
        traj->vel_plan -= max_dv;
    }
    else
    {
        traj->vel_plan = max_safe_vel; // 平滑着陆刹车曲线
    }
		
	traj->acc_plan = (traj->vel_plan - last_vel)	/0.001f;
	
	/*计算规划位置*/
	traj->pos_plan=traj->pos_plan+traj->vel_plan*Ts*6.0f;    //6.0f是将rpm转换成deg/s
	
	/*判断本周期规划后的位置是否跨越target pos*/
	float dist_next = target_pos - traj->pos_plan;
	if((dist>0.0f && dist_next <=0.0f) || (dist < 0.0f && dist_next >=0.0f)){
	    traj->pos_plan = target_pos;
		  traj->vel_plan = 0.0f;
		
		  traj->acc_plan = 0;
	}
	
}

/**/

//============FOC速度电流环运行程序================
//代码在中断中调用
void FOC_CURRENT_LOOP(volatile FocStatus *foc){
	if(foc->focEnable==0){ //使能为0时，电机停止
		
		foc->pwm_a=(uint16_t)(FOC_PWM_ARR*0.5f);
		foc->pwm_b=(uint16_t)(FOC_PWM_ARR*0.5f);
		foc->pwm_c=(uint16_t)(FOC_PWM_ARR*0.5f);
		return;
	}
	//三相电流转换成dq电流
		//clark_Park(foc); 
	clark_Conv(foc);
	park_Conv(foc);
	
	//电流环PID计算
	float id_error=foc->target_id-foc->id;
	float iq_error=foc->target_iq-foc->iq;
	
	foc->iqerror=iq_error;
	
	foc->ud = pid_Process(&foc->pid_id,id_error);
	foc->uq = pid_Process(&foc->pid_iq,iq_error);
	
	
	//逆帕克变换得到ualpha,ubeta
	anti_Park(foc);
	
	//SVPWM模块调用
	svpwm_Generate(foc);
	pwm_crr_setA(foc->pwm_a);
	pwm_crr_setB(foc->pwm_b);
	pwm_crr_setC(foc->pwm_c);
	
}

/*=======速度环部分=======*/
void FOC_SPEED_LOOP(volatile FocStatus *foc){

	//速度环PID计算
	float speed_error = foc->targetSpeed - foc->speed;
	
	/*积分释放*/
//	if (fabsf(foc->targetSpeed) < 10.0f && fabsf(foc->speed) < 50.0f && fabsf(foc->pos_error) < 3.0f) {
//    foc->pid_speed.integral *= 0.8f;
//}
	
	
//	foc->target_iq = pid_Process(&foc->pid_speed, speed_error);
	foc->target_iq = pid_Process(&foc->pid_speed, speed_error)+0.000001f*foc->traj.acc_plan;
	
//	ESO_SPEED_LOOP(foc);
//	

}

/*======位置环部分=======*/
void FOC_POSITION_LOOP(volatile FocStatus *foc){
	
	Trajectory_Update(&foc->traj,foc->targetAngle, 0.001f,foc);
	
	
//	float position_error=foc->targetAngle-foc->theta_m;
	float position_error = foc->traj.pos_plan - foc->theta_m;
//	foc->pos_error=position_error;
//	if(position_error>180){
//		position_error=position_error-360;}
//	else if(position_error<=-180){
//		position_error=position_error+360;}

//	foc->targetSpeed = pid_Process(&foc->pid_position,position_error);
//	foc->targetSpeed = pid_Process(&foc->pid_position,position_error);
	foc->targetSpeed = pid_Process(&foc->pid_position,position_error) + foc->traj.vel_plan ;
//	foc->targetSpeed = pid_Process(&foc->pid_position,position_error) + foc->traj.vel_plan + 0.1f*(foc->traj.vel_plan-foc->speed);
	
	/*******/
	
}

/*!
   \brief ESO速度环
*/
static void ESO_SPEED_LOOP(volatile FocStatus *foc){

	float u = foc->iq;     // 上一次输出
  float y = foc->speed;
	
	float e_eso = foc->eso_speed.z1 - y;  // e 是 ESO 估计速度和实际速度之间的误差。
	
	foc->eso_speed.z1 += 0.00025f * (	foc->eso_speed.z2+ foc->eso_speed.b0 * u- foc->eso_speed.beta1 * e_eso);
	foc->eso_speed.z2 += 0.00025f * ( - foc->eso_speed.beta2 * e_eso);
	
	float speed_error = foc->targetSpeed - foc->speed;

  float iq_cmd = pid_Process(&foc->pid_speed, speed_error);

	float iq_comp = -foc->eso_speed.z2 / foc->eso_speed.b0;
	iq_cmd += iq_comp;
	
	if (iq_cmd > MAX_CURRENT) {
        iq_cmd = MAX_CURRENT;
    } else if (iq_cmd < -MAX_CURRENT) {
        iq_cmd = -MAX_CURRENT;
    }

    foc->target_iq = iq_cmd;
}

/*=======开环=======*/
void FOC_OPEN_LOOP(volatile FocStatus *foc){

	if(foc->focEnable==0){ //使能为0时，电机停止
		
		foc->pwm_a=(uint16_t)(FOC_PWM_ARR*0.5f);
		foc->pwm_b=(uint16_t)(FOC_PWM_ARR*0.5f);
		foc->pwm_c=(uint16_t)(FOC_PWM_ARR*0.5f);
		return;
	}
	anti_Park(foc);
	svpwm_Generate(foc);
	pwm_crr_setA(foc->pwm_a);
	pwm_crr_setB(foc->pwm_b);
	pwm_crr_setC(foc->pwm_c);
	
	clark_Conv(foc);
	park_Conv(foc);

}
