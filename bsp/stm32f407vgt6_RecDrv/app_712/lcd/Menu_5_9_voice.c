#include "Menu_Include.h"

///*

//*/
u8 voice_sel=0;//��������
u8 voice_sel_flag=0;//���������޸Ĳ���

void voice_dis(u8 par)
{
	lcd_fill(0);
	if(Menu_voice_value==0)
		lcd_text12(0, 2,"TTS ����:    С",15,LCD_MODE_SET);
	else if(Menu_voice_value==1)
		lcd_text12(0, 2,"TTS ����:    ��",15,LCD_MODE_SET);
	else if(Menu_voice_value==2)
		lcd_text12(0, 2,"TTS ����:    ��",15,LCD_MODE_SET);
	else
		lcd_text12(0, 2,"TTS ����:    ��",15,LCD_MODE_SET);
	
	lcd_text12(0,20,"��������: С �� ��",18,LCD_MODE_SET);
	if(par==0)
		lcd_text12(60,20,"С",2,LCD_MODE_INVERT);
	else if(par==1)
		lcd_text12(78,20,"��",2,LCD_MODE_INVERT);
	else if(par==2)
		lcd_text12(96,20,"��",2,LCD_MODE_INVERT);
	else
		lcd_text12(78,20,"��",2,LCD_MODE_INVERT);
	lcd_update_all();

}

static void msg( void *p)
{
/*u8 voice=0;


*/
}
static void show(void)
{
if(voice_sel_flag==0)
	{
	voice_sel_flag=1;
	
	//DF_ReadFlash(DF_Voice_offset,0,&Menu_voice_value,1); 
	voice_sel=0;
	voice_dis(voice_sel);
	}
}


static void keypress(unsigned int key)
{

	switch(KeyValue)
		{
		case KeyValueMenu:
			pMenuItem=&Menu_5_other;
			pMenuItem->show();
			
			CounterBack=0;
			voice_sel_flag=0;
			voice_sel=0;
			break;
			
		case KeyValueOk:
			if(voice_sel_flag==1)
				{
				
				lcd_fill(0);
				lcd_text12(12,10,"����ѡ�����:",13,LCD_MODE_SET);
				if(voice_sel==0)
					lcd_text12(90,10,"С",2,LCD_MODE_INVERT);
				else if(voice_sel==1)
					lcd_text12(90,10,"��",2,LCD_MODE_INVERT);
				else if(voice_sel==2)
					lcd_text12(90,10,"��",2,LCD_MODE_INVERT);
				lcd_update_all();
				
			
				Menu_voice_value=voice_sel;
				//------ �����޸����� ---
			    delay_ms(100);
				switch(Menu_voice_value)
				{
                    case  0:
						      rt_hw_gsm_output("AT+VGR=1\r\n");  
							  break;						
					case  1:
						      rt_hw_gsm_output("AT+VGR=3\r\n");  
							  break;	
						
					case  2:	
                              rt_hw_gsm_output("AT+VGR=6\r\n");    
							  break;						
				}
				delay_ms(100);			
				voice_sel_flag=0;
				voice_sel=0;
				} 

			break;
			
		case KeyValueUP:
			if(voice_sel_flag==1)
				{
				if(voice_sel==0)
					voice_sel=2;
				else
					voice_sel--;
				voice_dis(voice_sel);
				}
	
			break;
			
		case KeyValueDown:
        	if(voice_sel_flag==1)
				{
				if(voice_sel==2)
					voice_sel=0;
				else
					voice_sel++;
				voice_dis(voice_sel);
				}
			break;	
		}
 KeyValue=0;
}


static void timetick(unsigned int systick)
{
	CounterBack++;
	if(CounterBack!=MaxBankIdleTime*5)
		return;
	CounterBack=0;
	pMenuItem=&Menu_1_Idle;
	pMenuItem->show();

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_5_9_voice=
{
"TTS ��������",
	12,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};

