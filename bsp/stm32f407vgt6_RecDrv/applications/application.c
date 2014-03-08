/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>
#include "App_moduleConfig.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

void rt_init_thread_entry(void* parameter)
{
    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();
        /* re-init device driver */
        rt_device_init_all();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif

//FS

//GUI
}



ALIGN(RT_ALIGN_SIZE)
static char thread_led_stack[1024];
struct rt_thread thread_led;



static void rt_thread_entry_led(void* parameter)
{
    u8 LowPowerCounter=0,CutPowerCounter=0,Battery_Flag=0;;	

    CAN_App_Init();   // CAN��ʼ��   
    AD_PowerInit(); 

  while (1)  
  {
          //-------  CAN query ---------------                                              
	       /*                       
			    if(CAN_initOver==1)
			    {
						   TestRx=(TestStatus)CANRXStr();    
						  if (TestRx == PASSED) 
						   	   rt_kprintf("\r\n CAN1-RxData\r\n");	 
			     }
		     */
		 //---------------------------------------------
          				//----------------------
				//------------- ��Դ��ѹAD��ʾ ----------------------- 
				 Power_AD.ADC_ConvertedValue=ADC_ConValue[0];               
				 Power_AD.AD_Volte=((Power_AD.ADC_ConvertedValue*543)>>12);   
					//rt_kprintf ("\r\n  ��ȡ���ĵ��AD��ֵΪ:	%d	 AD��ѹΪ: %d V  ��Դ��ѹ: %d V\r\n",ADC_ConvertedValue,a,a+11);	 
					 //  ---��ԴǷѹ����---- 
				 Power_AD.AD_Volte=Power_AD.AD_Volte+11;   

				 	 //  -----  ����2 ·  AD �Ĳɼ���ѹֵת��
                             // 1 .through  1  Voltage Value
                              AD_2through[0]=(((ADC_ConValue[1]-70)*543)>>12);   
				  AD_2through[0]= AD_2through[0]+11+10;    
				   AD_2through[0]= AD_2through[0]*100;// mV
                             // 2 .through  2  Voltage Value
				  AD_2through[1]=(((ADC_ConValue[2]-70)*543)>>12);    
				  AD_2through[1]= AD_2through[1]+11+10;     
				  AD_2through[1]= AD_2through[1]*100;   	


                 //------------�ⲿ�ϵ�---------------------
                 if(Power_AD.ADC_ConvertedValue<500)  //  С��500 ��Ϊ���ⲿ�ϵ�
				{
				      CutPowerCounter++;
				       if(CutPowerCounter>15)
				       	{
                                                  CutPowerCounter=0;
							 LowPowerCounter=0;					  
    
							   //------ ��������  Ϊ���� ������ 
							    if(Battery_Flag==0)
							    	{
				                       rt_kprintf("\r\n   ����Դ����! \r\n");   
									   Battery_Flag=1;	
									   MainPower_cut_process();
									   PositionSD_Enable();
									   Current_UDP_sd=1;	 
							    	}
						     //--------------------------------					   
				       	}
 
                          	}
			     else
				{      //    ��Դ���������
				       CutPowerCounter=0;
					  if(Battery_Flag==1)
			    	         { 
			    	                Powercut_Status=0x01;
			    	            MainPower_Recover_process();
                                rt_kprintf("\r\n   ����Դ����! \r\n");   
					       Battery_Flag=1;	  
					       PositionSD_Enable();
						Current_UDP_sd=1;    
			               }	
				         Battery_Flag=0;
                      //------------�ж�Ƿѹ������-----
                      // ���ݲɼ����ĵ�ѹ���ֵ�ƿ���ͣ��޸�Ƿѹ������ֵ
                       if(Power_AD.AD_Volte<=160)     // 16V   
                             Power_AD.LowWarn_Limit_Value=100;   // С��16V  ��Ϊ��С�� ��Ƿѹ������10V
                       else
					   	     Power_AD.LowWarn_Limit_Value=170;   // ����16V  ��Ϊ�Ǵ� ��Ƿѹ������17V  


					  
					 if(Power_AD.AD_Volte< Power_AD.LowWarn_Limit_Value)	   
					  {
							if((Warn_Status[3]&0x80)==0x00)     
							{
								  LowPowerCounter++;
								  if(LowPowerCounter>15) 
								  {
								    LowPowerCounter=0;						
									Warn_Status[3]|=0x80;  //Ƿѹ����
								     PositionSD_Enable();
								     Current_UDP_sd=1;	 
								     rt_kprintf("\r\n Ƿѹ����! \r\n");
								  }	
							}  				   
					  }
					 else
					 {
						    if((Warn_Status[3]&0x80)==0x80) 
						   	 rt_kprintf("\r\n ��Ƿѹ�л�ԭ����! \r\n");  
						   
						     LowPowerCounter=0;	
							 Warn_Status[3]&=~0x80; //ȡ��Ƿѹ���� 
					 }

					//---------------------------------------			   
			     	}	           
       
	   //----------------------------------    
	   // 3. GPS_ANTENNA_status 	 
		  GPS_ANTENNA_status(); 	
		  GPS_short_judge_timer(); 

		//-----------------------------------------------
		rt_thread_delay(RT_TICK_PER_SECOND/5);

	
  }
}


int rt_application_init()
{
    rt_thread_t init_thread;


     Device_CAN2_regist();    //  Device CAN2 Init 

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   256, 8, 20);  // thread null 
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,   
                                   2048, 80, 20); 
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    //------- init led1 thread
    rt_thread_init(&thread_led,
                   "led1", 
                   rt_thread_entry_led,
                   RT_NULL,
                   &thread_led_stack[0],
                   sizeof(thread_led_stack),Prio_Demo,5); 
    rt_thread_startup(&thread_led);
	
    return 0; 

}
/*@}*/
