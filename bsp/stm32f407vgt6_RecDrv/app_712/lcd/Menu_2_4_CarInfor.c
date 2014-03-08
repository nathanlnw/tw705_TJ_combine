#include  <string.h>
#include "Menu_Include.h"
#include "Lcd.h"

static unsigned char vech_num[16]={"���ƺ�:        "};
static unsigned char vech_type[25]={"��������:            "};
static unsigned char  vech_ID[19]={"����ID:            "}; 
static unsigned char  vech_VIN[20]={"VIN                 "}; 
static unsigned char  device_ID[19]={"�豸ID:            "};
static unsigned char  device_ID_7[19]={"�ն�ID:       "};

static unsigned char updown_flag=0;

//��ʻԱ����
void Display_driver(u8 drivercar)
{
    u8 color_disp[4];
switch(drivercar)
	{
	case 1:
	
		//���ƺ�JT808Conf_struct.Vechicle_Info.Vech_Num
		if(License_Not_SetEnable==1)
			memcpy(vech_num+7,"������  ",8); 
		else
		memcpy(vech_num+7,Vechicle_Info.Vech_Num,8);  
            //������ɫ
               memset(color_disp,0,sizeof(color_disp));
               switch(Vechicle_Info.Dev_Color)
               	{
               	    case 1: memcpy(color_disp,"��ɫ",4);break;
                         case 2:memcpy(color_disp,"��ɫ",4);break;
			    case 3:memcpy(color_disp,"��ɫ",4);break;
			    case 4:memcpy(color_disp,"��ɫ",4);break;
			    case 9:memcpy(color_disp,"����",4);break;
			    default:memcpy(color_disp,"��ɫ",4);break;	
               	}
	      lcd_fill(0);
	      lcd_text12(10,3,(char *)vech_num,15,LCD_MODE_SET);	   
		lcd_text12(10,19,"������ɫ:",9,LCD_MODE_SET);

      //====  ���ƺ�δ����=====
        if(License_Not_SetEnable==1)
			 lcd_text12(64,19,(char *)"0",1,LCD_MODE_SET); 
	    else
		     lcd_text12(64,19,(char*)color_disp,4,LCD_MODE_SET); 
		lcd_update_all();
		break;

	case 2://����ID Vechicle_Info.DevicePhone
		lcd_fill(0);
              memcpy(vech_type+9,Vechicle_Info.Vech_Type,6);
		lcd_text12(0,3,(char *)vech_type,19,LCD_MODE_SET);		
		//��ȡ�豸�ٶ�ȡ����GPS�ٶȻ����ٶ����ٶ�
		//��ȡ�豸�ٶ�ȡ����GPS�ٶȻ����ٶ����ٶ�
		if(JT808Conf_struct.DF_K_adjustState)
			lcd_text12(0,18,"�豸�ٶ�:�������ٶ�",19,LCD_MODE_SET);
		else
			lcd_text12(0,18,"�豸�ٶ�:GPS�ٶ�",16,LCD_MODE_SET);
		lcd_update_all();
		break;
	case 3:  //  ����ID
        lcd_fill(0);
        memcpy(vech_ID+7,SimID_12D,12); 
        lcd_text12(0,3,(char *)vech_ID,19,LCD_MODE_SET);
        memcpy(vech_VIN+3,Vechicle_Info.Vech_VIN,17); //����VIN
        lcd_text12(0,19,(char *)vech_VIN,20,LCD_MODE_SET);
        lcd_update_all();   
		 break;
	case 4:  //  �豸ID
        lcd_fill(0);
        memcpy(device_ID+7,SimID_12D,12);    
        lcd_text12(0,3,(char *)device_ID,19,LCD_MODE_SET);
		memcpy(device_ID_7+7,SimID_12D+5,7); 
        lcd_text12(0,18,(char *)device_ID_7,14,LCD_MODE_SET);
        lcd_update_all();   
		 break;
	default:
		break;
	}

}

static void msg( void *p)
{
}
static void show(void)
{
	lcd_fill(0);
	lcd_text12(24, 3,"������Ϣ�鿴",12,LCD_MODE_SET);
	lcd_text12(24,19,"�鿴�밴ȷ��",12,LCD_MODE_SET);
	lcd_update_all();

}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			pMenuItem=&Menu_2_InforCheck;
			pMenuItem->show();
			CounterBack=0;

			updown_flag=0;
			break;
		case KeyValueOk:
			updown_flag=1;
			Display_driver(1);
			break;
		case KeyValueUP:
			if(updown_flag==1)
			  {
			  Display_driver(4);
			  updown_flag=4;
			  }
			else if(updown_flag==2)
			  {	
			  Display_driver(1);
			  updown_flag=1;
			  }	
			else if(updown_flag==3)
			  {	
			  Display_driver(2);
			  updown_flag=2;
			  }
			else if(updown_flag==4)
			  {	
			  Display_driver(3);
			  updown_flag=3;
			  }
			break;
		case KeyValueDown:
			if(updown_flag==1)
			  {	
			  Display_driver(2);
			  updown_flag=2;
			  }
			else if(updown_flag==2)
			  {	
			  Display_driver(3);
			  updown_flag=3;
			  }	
			else if(updown_flag==3)
			  {	
			  Display_driver(4);
			  updown_flag=4;
			  }
			else if(updown_flag==4)
			  {	
			  Display_driver(1);
			  updown_flag=1;
			  }
			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{
	CounterBack++;
	if(CounterBack!=MaxBankIdleTime)
		return;
	CounterBack=0;
	pMenuItem=&Menu_1_Idle;
	pMenuItem->show();
}


ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_2_4_CarInfor=
{
	"������Ϣ�鿴",
	12,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};

