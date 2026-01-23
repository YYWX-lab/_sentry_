/* 包含头文件 ----------------------------------------------------------------*/
#include "clinet_ui_app.h"
#include "client_ui_base.h"
#include "chassis/chassis_app.h"
#include "infantry_console.h"
#include "gimbal/gimbal_app.h"
#include "user_protocol.h"
// #include "super_power.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
int16_t flag = 1;
int16_t init_flag = 0;
UiGraphicData_t line;
UiStringData_t str;
UiStringData_t shoot;
UiStringData_t autoo;
UiStringData_t spin;
UiNumberData_t num;
UiNumberData_t fnum;
UiNumberData_t fnum2;
UiNumberData_t none;
UiGraphicData_t Rectangle;
extern float cap_v;
extern GimbalHandle_t gimbal_handle;
extern UI_Update_flag UI_update_flag;

extern ChassisHandle_t chassis_handle;
extern Console_t console;

static float flag_on = 0;
static float flag_off = 0;
static float a = 500;
float end_x;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

void ui_init()
{
   

   if (init_flag == 0)
   {
      
      if (flag < 10)
      {
         flag += 1;
         ClientUI_Delete(UI_DELETE_ALL_OPERATE, 0);

         
         
         
         
      //    ClientUI_DrawIntNumber(&fnum2, "fnc", 0, 0, 0, 1200, 50, 10, UI_COLOR_YELLOW);
      //    ClientUI_DrawString(&str, "str", 0, 200, 300, "ciallo", 50, 10, UI_COLOR_YELLOW);
      //    ClientUI_DrawIntNumber(&num, "num", 0, -300, -300, 200, 50, 10, UI_COLOR_YELLOW);
      //    ClientUI_DrawFloatNumber(&fnum, "fn", 0, 500, 0, 12.34f, 50, 10, UI_COLOR_YELLOW);
      //    ClientUI_DrawLine(&line, "lin", 0, 0, 0, 960, 340, 10, UI_COLOR_YELLOW);
      }
      else if (flag < 11)
      {    
         
         ClientUI_DrawString(&str, "cia", 0, 200, 200, "what can i say", 50, 8, UI_COLOR_YELLOW);
         flag += 1;
      }
      else if (flag < 12)
      {
         
         //   ClientUI_DrawIntNumber(&num, "num", 0, -300, -300, 200, 50, 10, UI_COLOR_YELLOW);
         ClientUI_DrawString(&spin,"spi",0,-900,0,"follow",40,7,UI_COLOR_PINK);//自旋模式
         flag += 1;
      }
      else if (flag < 13)
      {
         
         ClientUI_DrawFloatNumber(&fnum, "d", 0, 500, 0, 0.0f, 50, 10, UI_COLOR_YELLOW);//自瞄 目标的距离
         flag += 1;
      }
      else if (flag < 14)
      {
         
         ClientUI_DrawRectangle(&Rectangle,"jx",0,-300,-450,500,-500,3,UI_COLOR_YELLOW);//画超电矩形
         flag += 1;
      }
      else if (flag < 15)
      {
         
         ClientUI_DrawString(&shoot, "str", 0, 600, -130, "off", 50, 10, UI_COLOR_PURPLISH_RED);//射击模式是否打开
         flag += 1;
      }
      else if (flag < 18)
      {
         
         ClientUI_DrawString(&str, "sho", 0, 500, 0, "shoot", 75, 5, UI_COLOR_CYAN);//是否进入射击模式
         flag += 1;
      }
      else if (flag < 19)
      {
         
         ClientUI_DrawLine(&line, "lin", 0, -290, -475, 490, -475, 30, UI_COLOR_GREEN);//超电进度条
         flag += 1;
      }
      else if (flag < 20)
      {
         
         ClientUI_DrawString(&str, "aut", 0, -900, 200, "auto aim", 60, 5, UI_COLOR_CYAN);//自瞄
         flag += 1;
      }
      else if (flag < 25)
      {
         
         ClientUI_DrawString(&autoo, "aim", 0, -725, 100, "off", 50, 8, UI_COLOR_PURPLISH_RED); //自瞄是否打开
         flag +=1;
            
      }
      else 
      {
         init_flag =1;
      }
      
   }
   
}



int32_t ClientUI_DrawLoop(void *argc)
{
   uint16_t robot_id = RefereeSystem_GetRobotID();
   ClientUI_SetHeaderSenderID(robot_id);
   ClientUI_SetHeaderReceiverID(0x100 + robot_id);
   ui_init();
   Comm_GimbalInfo_t* gimbal_info = GimbalInfo_Pointer();
   if (init_flag == 1)
   {

       if (UI_update_flag.shoot_mode == 1)
       {   
         ClientUI_SetColor(&shoot, UI_COLOR_GREEN);
         ClientUI_UpdateString(&shoot,"on");
            
         UI_update_flag.shoot_mode = 2;    
       }
       else if(UI_update_flag.shoot_mode == 0)
       {
         ClientUI_SetColor(&shoot, UI_COLOR_PURPLISH_RED); 
         ClientUI_UpdateString(&shoot,"off");
         
         UI_update_flag.shoot_mode = 2;    
       }//射击模式是否打开


       if (UI_update_flag.vision_mode == 1)
       {
         ClientUI_SetColor(&autoo, UI_COLOR_GREEN);
         ClientUI_UpdateString(&autoo,"on");
         
         UI_update_flag.vision_mode = 2;
       }
       else if (UI_update_flag.vision_mode == 0)
       {
         ClientUI_SetColor(&autoo, UI_COLOR_PURPLISH_RED);
         ClientUI_UpdateString(&autoo,"off");
         
         UI_update_flag.vision_mode = 2;
       }//自瞄是否打开

      
      // if (console.rc->kb.bit.F)
      // {
      //    ClientUI_UpdateString(&spin,"nomal");
      //    ClientUI_SetColor(&spin, UI_COLOR_CYAN);
      // }
      // else if (console.rc->kb.bit.C)
      // {
      //    ClientUI_UpdateString(&spin,"spin");
      //   ClientUI_SetColor(&spin, UI_COLOR_CYAN);
      // }

       if (UI_update_flag.chassis_mode == 1)
       {
         ClientUI_SetColor(&autoo, UI_COLOR_CYAN);
         ClientUI_UpdateString(&spin,"spin");
         
         UI_update_flag.chassis_mode = 2;
       }
       else if (UI_update_flag.chassis_mode == 0)
       {
         ClientUI_SetColor(&autoo, UI_COLOR_CYAN);
         ClientUI_UpdateString(&spin,"follow");
         
         UI_update_flag.chassis_mode = 2;
       }//自旋模式是否开启


       end_x = ((cap_v-1000) /1600)*780-290;//没电的时候时10v，满电的时候是26v，所以差值是16v，由于超电传回来的数值乘了100，所以除以1600
       if ((cap_v-1000) /1600 < 0.3)
       {
          ClientUI_SetColor(&line, UI_COLOR_PURPLISH_RED);
       }
       else
       {
          ClientUI_SetColor(&line, UI_COLOR_GREEN);
       }
       if ((cap_v-1000) /1600 < 0)
       {
          end_x = -290;
       }

       ClientUI_SetFloatNumber(&fnum, (gimbal_info->vision_distance)/1000);//自瞄距离更新
       ClientUI_SetPosition(&line, -290, -475 , end_x,-475);//超电电量进度条更新     
       ClientUI_Update(&line, &fnum, &none, &none, &none, &none,&none);//所有信息一并更新
   }

   return 0;
}



