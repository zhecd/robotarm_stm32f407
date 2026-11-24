/** 链接真实PWM驱动，验证旧角度兼容、无动作绑定、零时刻/回绕与HAL失败。 */
#include "driver/drv_servo.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"servo line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now,starts,stops,writes,preload;
static bool fail_start,fail_stop,channel_busy;
uint32_t HAL_GetTick(void){return now;}
void Test_SetCompare(TIM_HandleTypeDef *t,uint32_t channel,uint32_t pulse)
{CHECK(t && channel==TIM_CHANNEL_2);t->compare=pulse;writes++;preload=writes;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel)
{CHECK(t && channel==TIM_CHANNEL_2 && preload==writes);preload=0;starts++;
 if(fail_start || channel_busy)return HAL_ERROR;
 channel_busy=true;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t,uint32_t channel)
{CHECK(t && channel==TIM_CHANNEL_2);stops++;if(fail_stop)return HAL_ERROR;
 channel_busy=false;return HAL_OK;}
int main(void)
{
 TIM_HandleTypeDef timer={0};ServoDevice_t g={0};
 CHECK(!Drv_Servo_Bind(NULL,&timer,TIM_CHANNEL_2));CHECK(!Drv_Servo_Bind(&g,NULL,TIM_CHANNEL_2));
 CHECK(!Drv_Servo_Bind(&g,&timer,99));CHECK(Drv_Servo_Bind(&g,&timer,TIM_CHANNEL_2));
 CHECK(!starts && !stops && !writes && !g.pwm_active && g.state==SERVO_STATE_UNKNOWN);
 /* 旧版72°关/100°开与500..2500us换算保持一致。正常Init仍主动关闭。 */
 now=0;Drv_Servo_Init(&g,&timer,TIM_CHANNEL_2);
 CHECK(starts==1 && timer.compare==1300 && g.state==SERVO_STATE_CLOSE && g.pwm_active);
 now=599;Drv_Servo_IdleStop(&g);CHECK(stops==0 && g.pwm_active);
 now=600;Drv_Servo_IdleStop(&g);CHECK(stops==1 && !g.pwm_active && g.state==SERVO_STATE_UNKNOWN);
 Drv_Servo_Open(&g);CHECK(timer.compare==1611 && g.state==SERVO_STATE_OPEN && g.cur_angle==100.0f);
 uint32_t count=starts;Drv_Servo_Open(&g);CHECK(starts==count);
 CHECK(!Drv_Servo_TrySetAngle(&g,NAN) && !Drv_Servo_TrySetAngle(&g,INFINITY));
 CHECK(starts==count && writes==starts);CHECK(!Drv_Servo_TrySetAngle(NULL,100));
 ServoDevice_t empty={0};CHECK(!Drv_Servo_TrySetAngle(&empty,100));CHECK(!Drv_Servo_TryStop(&empty));
 CHECK(!Drv_Servo_TryOpen(NULL) && !Drv_Servo_TryClose(NULL));
 CHECK(!Drv_Servo_TryOpen(&empty) && !Drv_Servo_TryClose(&empty));
 CHECK(Drv_Servo_TrySetAngle(&g,-1));CHECK(timer.compare==500 && g.cur_angle==0);
 CHECK(Drv_Servo_TrySetAngle(&g,181));CHECK(timer.compare==2500 && g.cur_angle==180);
 CHECK(Drv_Servo_TryStop(&g));
 /* uint32毫秒回绕仍按600ms停止。 */
 now=UINT32_MAX-100U;CHECK(Drv_Servo_TrySetAngle(&g,90));count=stops;
 now=498;Drv_Servo_IdleStop(&g);CHECK(stops==count && g.pwm_active);
 now=499;Drv_Servo_IdleStop(&g);CHECK(stops==count+1U && !g.pwm_active);
 /* PWM启动失败不能被Open/Close覆盖为成功目标状态。 */
 fail_start=true;Drv_Servo_Open(&g);CHECK(g.state==SERVO_STATE_UNKNOWN && !g.pwm_active);
 Drv_Servo_Close(&g);CHECK(g.state==SERVO_STATE_UNKNOWN && !g.pwm_active);
 fail_stop=true;CHECK(!Drv_Servo_TrySetAngle(&g,90));CHECK(g.pwm_active && g.state==SERVO_STATE_UNKNOWN);
 fail_start=false;fail_stop=false;now+=600;Drv_Servo_IdleStop(&g);CHECK(!g.pwm_active);
 CHECK(Drv_Servo_TrySetAngle(&g,100));fail_stop=true;CHECK(!Drv_Servo_TryStop(&g));CHECK(g.pwm_active);
 fail_stop=false;CHECK(Drv_Servo_TryStop(&g));CHECK(!g.pwm_active);
 Drv_Servo_SetAngle(&g,72);CHECK(g.pwm_active && timer.compare==1300);Drv_Servo_Stop(&g);
 CHECK(!g.pwm_active);Drv_Servo_IdleStop(NULL);Drv_Servo_Stop(NULL);
 CHECK(Drv_Servo_GetHandle()==Drv_Servo_GetHandle());
 puts("servo_driver: original angles; bind without PWM; compare-before-start; tick0/wrap; numeric and HAL failure handling passed");
 return 0;
}
