#include  <string.h>
#include "Menu_Include.h"
#include "Lcd.h"

static u8 type_selc_screen=0;
static u8 type_selc_flag=0;
static void msg( void *p)
{

}
static void show(void)
{
lcd_fill(0);
lcd_text12(0, 3,"两客一危",8,LCD_MODE_INVERT);
lcd_text12(0,18,"货运",4,LCD_MODE_SET);
lcd_update_all();
type_selc_flag=1;
type_selc_screen=1;
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			break;
		case KeyValueOk:
			if(type_selc_screen==1)
				{
				type_selc_screen=2;
				if(type_selc_flag==1)
					{
						lcd_fill(0);
						lcd_text12(9,3,"车辆类型:两客一危",17,LCD_MODE_SET);
						lcd_text12(6,18,"按确认键设置下一项",18,LCD_MODE_SET);
						lcd_update_all();
						Vechicle_Info.Vech_Type_Mark=1;
					}
				else
					{   
						lcd_fill(0);
						lcd_text12(15,3,"车辆类型:货运车",15,LCD_MODE_SET);
						lcd_text12(6,18,"按确认键设置下一项",18,LCD_MODE_SET);
						lcd_update_all();
						Vechicle_Info.Vech_Type_Mark=2;
					}
				}
			else if(type_selc_screen==2)
				{
				if(MENU_set_carinfor_flag==1)
					{
					 //--------------------------------------------------------------------------------
                     //  select mode
					 if(Vechicle_Info.Vech_Type_Mark==1)   //两客一危
					 	{
					 	    Vechicle_Info.Link_Frist_Mode=1;
	                        Socket_main_Set("60.28.50.210:9131"); 
							 //--------    清除鉴权码 -------------------
					        idip("clear");		
							

					 	}
					 else
					 if(Vechicle_Info.Vech_Type_Mark==2)  //  公共货运平台
					 	{
	                         Vechicle_Info.Link_Frist_Mode=0; 
							 port_main("7008");
							  //--------    清除鉴权码 ------------------- 
					         idip("clear");		
					 	}
					 //------------------------------------------------------------------------------------					
					DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
					}
				type_selc_screen=0;
				type_selc_flag=0;
				
				CarSet_0_counter=1;
				
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				}
			break;
		case KeyValueUP:
			if(type_selc_screen==1)
				{
				type_selc_flag=1;
				lcd_fill(0);
				lcd_text12(0, 3,"两客一危",8,LCD_MODE_INVERT); 
				lcd_text12(0,18,"货运",4,LCD_MODE_SET);
				lcd_update_all();
				}
			break;
		case KeyValueDown:
			if(type_selc_screen==1)
				{
				type_selc_flag=2;
				lcd_fill(0);
				lcd_text12(0, 3,"两客一危",8,LCD_MODE_SET);
				lcd_text12(0,18,"货运",4,LCD_MODE_INVERT);
				lcd_update_all();
				}
			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_0_0_type=
{
	"类型",
	6,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};




