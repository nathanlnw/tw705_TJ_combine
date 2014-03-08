/*
   vdr Vichle Driver Record ������ʻ��¼
 */

#include <rtthread.h>
#include <finsh.h>
#include "stm32f4xx.h"

#include <rtdevice.h>
#include <dfs_posix.h>

#include <time.h>

#include "SST25VF.h"
#include "Vdr.h"
#include "App_moduleConfig.h"


//  =================  �г���¼�� ��� ============================
/*
    StartPage :    6320          Start Address offset :   0x316000       

    Area Size :
                          213  Sector       = 1704 pages
                           ----------------------
                           
				����               
				1                                     00-07H
				90                                    08H               
				85                                    09H
				7                                      10H
				2                                      11H 
				2                                      12H
				1                                      13H
				1                                      14H  
				1                                      15H   

          ----------  ֻ������������---  ע�� ����������� Vdr.C  				
*/

#define       SECTORSIZE                  4096
#define       VDR_START_ADDRESS           0x316000
#define       MarkByte                    0x0F          // ��ʾд��������Ϣ

//-------------------------- Sectors-----------  total   195 Sectors --------------
#define  VDR_00_07_SIZE      1
#define  VDR_08_SIZE         90    //   128 rec_size               1sector=32 recrods    2880/32=90
#define  VDR_09_SIZE         72    //  666=>>768 rec_size      1sector=5 recrods     360/5=72
#define  VDR_10_SIZE         7     //   234=>> 256 rec_size    1sector=16              100/16 =7
#define  VDR_11_SIZE         2     //   50 =>> 64                  1 sector=64             100/64 =2 
#define  VDR_12_SIZE         2     //    25 =>> 32                 1 sector=128            200/128=2
#define  VDR_13_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_14_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_15_SIZE         1     //  133==>256                  1 sector =16               10/16 =1

 //   �洢�����������
#define  VDR_08_MAXindex         2880
#define  VDR_09_MAXindex         360   
#define  VDR_10_MAXindex         100
#define  VDR_11_MAXindex         10
#define  VDR_12_MAXindex         200
#define  VDR_13_MAXindex         100
#define  VDR_14_MAXindex         100
#define  VDR_15_MAXindex         10   



//----------------------absolutely  address   --------------  
#define VDR_00_to_07H    VDR_START_ADDRESS 

#define VDR_08H_START	 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE
#define VDR_08H_END		 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE-1

#define VDR_09H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE)*SECTORSIZE
#define VDR_09H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE)*SECTORSIZE-1


#define VDR_10H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE
#define VDR_10H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE-1

#define VDR_11H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE
#define VDR_11H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE-1

#define VDR_12H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE
#define VDR_12H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE-1

#define VDR_13H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE
#define VDR_13H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE-1

#define VDR_14H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE
#define VDR_14H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE-1

#define VDR_15H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE
#define VDR_15H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE-1 






//  -   no  use  actually----
#define VDR_08H_09H_START	 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE
#define VDR_08H_09H_END		 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE-1




/*
   ʵ��ÿ����¼128�ֽڣ����ڶ�λ��Ҫ��һ���Ĵ���û��ʹ��delta����

   ��¼��ʽ

   000--003  yy-mm-dd hh:mm   ���԰�ʱ��ת��UTC��ʽ,���ڱȽϲ���
      �ֿ��Խ�ʡһ���ֽڣ���Ч��ʱ���ʽFFFFFFFFFF
   004--056  ���ٶȼ�¼ �ٶ�Ҫ��+/- 1kmh
          ԭ��60*8=480bit ������7bit������Ҫ 60*7=420bit=53byte
          7bit�� 0b1111111 ��ʾ�ٶ���Ч(gpsδ��λ)
   057--116  ״̬��Ϣ
   117--126  ��λ����λ��,�μ��г���¼�� GBT19065 �׸���Чλ��

 */


/*
   4MB serial flash 0x400000
 */

/*ת��hex��bcd�ı���*/
#define HEX_TO_BCD( A ) ( ( ( ( A ) / 10 ) << 4 ) | ( ( A ) % 10 ) )



/*����Сʱ��ʱ���,��Ҫ��Ϊ�˱Ƚϴ�Сʹ��
   byte0 year
   byte1 month
   byte2 day
   byte3 hour
 */
typedef unsigned int YMDH_TIME;

typedef struct
{
	uint8_t cmd;

	uint32_t	ymdh_start;
	uint8_t		minute_start;

	uint32_t	ymdh_end;
	uint8_t		minute_end;

	uint32_t	ymdh_curr;
	uint8_t		minute_curr;
	uint32_t	addr;

	uint16_t	blocks;         /*����ÿ���ϴ����ٸ����ݿ�*/
	uint16_t	blocks_remain;  /*��ǰ��֯�ϴ����ǻ���Ҫ�ĵ�blocks*/
}VDR_CMD;

VDR_CMD		vdr_cmd;

uint8_t		vdr_tx_info[1024];
uint16_t	vtr_tx_len = 0;


/*����д���ļ�����Ϣ
   0...3  д��SerialFlash�ĵ�ַ
   4...31 �ļ���
 */

//=====================================================
VDR_INDEX         Vdr_Wr_Rd_Offset;      //  ��¼��д���� λ��
VDR_DATA          VdrData;               //  ��ʻ��¼�ǵ�databuf
VDR_TRIG_STATUS   VDR_TrigStatus;









//======================================================
/*
      ������ȡ��ǰ��ʻ��¼�Ĳ���λ��

      type :    0  �����������      08  09  10  11 12 13  14 
      u8    :     0   :��������  
      
*/
u8  Vdr_PowerOn_getWriteIndex(u8 type)   
{
   
       u16    i;
	   u8	  c;  
      
	  //	 1.  get  address	


     switch(type)
     	{
		   case 0x08:
		   	           // 1.1  �ȶ�ȡ��һ���ַ����ж�
		   	           //rt_kprintf("\r\n 08H write=0x%X  \r\n",VDR_08H_START); 
		   	           for(i=0;i<VDR_08_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_08H_START+i*128);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_08H_Write=i; 
							  rt_kprintf("\r\n 08H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_08H_Write); 
                              break;
						    }
		   	           	}
					   // 1.2. û���ҵ�2880 ��ô ������һ��  ��0 ��ʼ
					   if(i==VDR_08_MAXindex)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_08H_START );  
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_08H_Write=0;
					   	}							   	
		   	          
		   	           break;
		   case 0x09:
		   	           //  2.1 
		   	           // rt_kprintf("\r\n 09H write=0x%X  \r\n",VDR_09H_START); 
		   	          for(i=0;i<360;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_09H_START+i*768);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_09H_Write=i; 
							  rt_kprintf("\r\n 09H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_09H_Write); 
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�360 ��ô ������һ��  ��0 ��ʼ
					   if(i==360)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_09H_START );   
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_09H_Write=0;  
					   	} 
					   break;
		   
		   case 0x10:
		   	          //  rt_kprintf("\r\n 10H write=0x%X  \r\n",VDR_10H_START);  
		   	           for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_10H_START+i*256);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_10H_Write=i; 
							  rt_kprintf("\r\n 10H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_10H_Write);  
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_10H_START );    
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_10H_Write=0;   
					   	}  
					   break;
		   
		   case 0x11:
		   	            for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_11H_START+i*64);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							  rt_kprintf("\r\n 11H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_11H_Write);   
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_11H_START );    
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_11H_Write=0;       
					   	}  
					   break;

		   case 0x12: 
		   	            for(i=0;i<200;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_12H_START+i*32);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_12H_Write=i; 
							  rt_kprintf("\r\n 12H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_12H_Write);   
                              break;
						    }
		   	           	}
					    // 2. û���ҵ�200 ��ô ������һ��  ��0 ��ʼ 
					    if(i==200)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_12H_START );   
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_12H_Write=0;           
					   	}  
						break;

		   case 0x13:   for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_13H_START+i*8); 
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							  rt_kprintf("\r\n 13H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_13H_Write);   
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_13H_START );     
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_13H_Write=0;       
					   	}  
						break;

		   case 0x14:   
		   	             for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_14H_START+i*8); 
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_14H_Write=i; 
							  rt_kprintf("\r\n 14H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_14H_Write);    
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_14H_START );        
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_14H_Write=0;       
					   	}  
						break;

		   case 0x15:
		   	            for(i=0;i<10;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_15H_START+i*256);  
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_15H_Write=i; 
							  rt_kprintf("\r\n 15H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_15H_Write);    
                              break;
						    }
		   	           	}
					   // 2. û���ҵ�10 ��ô ������һ��  ��0 ��ʼ
					   if(i==10)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_15H_START );     
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_15H_Write=0;          
					   	}  
						break;

     	}
  return true;
}


//    ���������г���¼д�����ʼ��ַ   from  08H
void total_ergotic(void)    
{
    Vdr_PowerOn_getWriteIndex(0x08);
    Vdr_PowerOn_getWriteIndex(0x09);
	Vdr_PowerOn_getWriteIndex(0x10);
    Vdr_PowerOn_getWriteIndex(0x11);
	Vdr_PowerOn_getWriteIndex(0x12);
    Vdr_PowerOn_getWriteIndex(0x13);
	Vdr_PowerOn_getWriteIndex(0x14);
	Vdr_PowerOn_getWriteIndex(0x15);     

	Vdr_Wr_Rd_Offset.V_07H_Read=0;
	Vdr_Wr_Rd_Offset.V_08H_Read=0;
	Vdr_Wr_Rd_Offset.V_09H_Read=0;
	Vdr_Wr_Rd_Offset.V_10H_Read=0;
	Vdr_Wr_Rd_Offset.V_11H_Read=0;
	Vdr_Wr_Rd_Offset.V_12H_Read=0;
	Vdr_Wr_Rd_Offset.V_13H_Read=0;
	Vdr_Wr_Rd_Offset.V_14H_Read=0;
	Vdr_Wr_Rd_Offset.V_15H_Read=0; 
}
//FINSH_FUNCTION_EXPORT( total_ergotic, total_ergotic );
//   ֻ���ڳ�ʼ�����õ�
void vdr_erase(void) 
{
   u16  i=0;
    // start address        0x316000        start sector :790th        total: 195 sector 

    //  1. erase  to  839          840-790=50 
// if(rt_mutex_take(DF_lock_mutex,150)==RT_EOK) 
{
   DF_LOCK=1;
	//  32K
	 rt_kprintf("\r\n   <------------  vdr  area  erase  0 ------------------>\r\n");   
   for(i=0;i<10;i++)
   {
	 WatchDog_Feed();
	 SST25V_SectorErase_4KByte(VDR_START_ADDRESS+i*SECTORSIZE);  	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+i*SECTORSIZE);   
     delay_ms(150);     
	 
   }	 
   
   rt_kprintf("\r\n   <------------  vdr  area  erase  1 ------------------>\r\n");   
   
   #if 1
   // 195-50=145         145-64*2=17             0x320000
   for(i=0;i<22;i++) 
   {
	 WatchDog_Feed();
	// SST25V_BlockErase_64KByte(VDR_START_ADDRESS+0xA000+i*0x10000);   // 32k  <=> 0x8000   
      
	 SST25V_BlockErase_32KByte(VDR_START_ADDRESS+0xA000+i*0x8000);   // 32k  <=> 0x8000   8sector
     DF_delay_ms(700); 
	 WatchDog_Feed();
	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+0xA000+i*0x8000);   
	//  rt_kprintf("\r\n   <------------ erase 32K : %d\r\n",i+1);    
   }
  #endif

  
   DF_LOCK=0;
 //  rt_mutex_release(DF_lock_mutex);  
 }   

   rt_kprintf("\r\n   <------------  vdr  area  erase   over  ------------------>\r\n");  
}
//FINSH_FUNCTION_EXPORT( vdr_erase, vdr_erase ); 




/*
         ������������ͣ�����ʼʱ����ҵ�������������ʼ��ַ������
*/
void query_vdr(u8 type,u8 *intime, u16 current_start,u16 num)
{
  u16 i=0;
	u8  c;

   switch(type)
   {
		  case 0x08:
					  // 1.1  �ȶ�ȡ��ǰд�����ʱ��
                      if(Vdr_Wr_Rd_Offset.V_08H_Write==0)
                      	{
                             current_start=0;
							 num=0;
							 break;
                      	}
                      
                      //  1.2    ��ȡ��ǰ����ļ�¼��ʱ��
					  
					  for(i=0;i<VDR_08_MAXindex;i++)
					   {
						   c=SST25V_ByteRead(VDR_08H_START+i*128);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_08H_Write=i; 
							 rt_kprintf("\r\n 08H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_08H_Write); 
							 break;
						   }
					   }
					  // 1.2. û���ҵ�2880 ��ô ������һ��	��0 ��ʼ
					  if(i==VDR_08_MAXindex) 
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_08H_START );  
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_08H_Write=0;
					   }							   
					 
					  break;
		  case 0x09:
					  //  2.1 
					 for(i=0;i<360;i++)
					   {
						   c=SST25V_ByteRead(VDR_09H_START+i*768);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_09H_Write=i; 
							 rt_kprintf("\r\n 09H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_09H_Write); 
							 break;
						   }
					   }
					  // 2. û���ҵ�360 ��ô ������һ��  ��0 ��ʼ
					  if(i==360)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_09H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_09H_Write=0;  
					   } 
					  break;
		  
		  case 0x10:
					  for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_10H_START+i*256);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_10H_Write=i; 
							 rt_kprintf("\r\n 10H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_10H_Write);  
							 break;
						   }
					   }
					  // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_10H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_10H_Write=0;	
					   }  
					  break;
		  
		  case 0x11:
					   for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_11H_START+i*64);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							 rt_kprintf("\r\n 11H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_11H_Write);   
							 break;
						   }
					   }
					  // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_11H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_11H_Write=0;		
					   }  
					  break;
   
		  case 0x12: 
					   for(i=0;i<200;i++)
					   {
						   c=SST25V_ByteRead(VDR_12H_START+i*32);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_12H_Write=i; 
							 rt_kprintf("\r\n 12H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_12H_Write);   
							 break;
						   }
					   }
					   // 2. û���ҵ�200 ��ô ������һ��  ��0 ��ʼ 
					   if(i==200)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_12H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_12H_Write=0;			
					   }  
					   break;
   
		  case 0x13:   for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_13H_START+i*8); 
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							 rt_kprintf("\r\n 13H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_13H_Write);   
							 break;
						   }
					   }
					  // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_13H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_13H_Write=0;		
					   }  
					   break;
   
		  case 0x14:   
						for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_14H_START+i*8); 
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_14H_Write=i; 
							 rt_kprintf("\r\n 14H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_14H_Write);	
							 break;
						   }
					   }
					  // 2. û���ҵ�100 ��ô ������һ��  ��0 ��ʼ
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_14H_START );		 
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_14H_Write=0;		
					   }  
					   break;
   
		  case 0x15:
					   for(i=0;i<10;i++)
					   {
						   c=SST25V_ByteRead(VDR_15H_START+i*256);	
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_15H_Write=i; 
							 rt_kprintf("\r\n 15H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_15H_Write);	
							 break;
						   }
					   }
					  // 2. û���ҵ�10 ��ô ������һ��	��0 ��ʼ
					  if(i==10)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_15H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_15H_Write=0;		   
					   }  
					   break;
   
	   }

 
}

//    ������ʻ��¼������  

void  VDR_product_08H_09H_10H(void)
{
  
  u16 j=0,cur=0;	 
  
  //  ��ʻ��¼������ݲ��� ����,  �Ҷ�λ�������   
  if((VDR_TrigStatus.Running_state_enable==1)&&(UDP_dataPacket_flag==0X02)) 	  
  { 	//	  ��������ʻ״̬��
			
			//-------------------  08H	 --------------------------------------------------
			if((Temp_Gps_Gprs.Time[2]==0)&&(VDR_TrigStatus.Running_state_enable==1)) 
			{  // ��Ϊ0 
				// 0.  �ж��Ƿ��ǵ�һ���ϵ�
				if((VdrData.H_08[0]==0x00)&&(VdrData.H_08[1]==0x00)) 
				{
					  rt_kprintf("\r\n first  get  08H	  \r\n");
				}
				else
				{ 
					  memcpy(VdrData.H_08_BAK,VdrData.H_08,126);
					  VdrData.H_08_saveFlag=1;	//	 ����  ��ǰ���ӵ����ݼ�¼					 
				}	 
			   //  1.  ������ ʱ����	 ����0
				 VdrData.H_08[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_08[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_08[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_08[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_08[4]=((Temp_Gps_Gprs.Time[1]/10)<<4)+(Temp_Gps_Gprs.Time[1]%10);
				 VdrData.H_08[5]=0; 	
  
			   // 2.  initial 
			   memset(VdrData.H_08+6,0xFF,120);    // Ĭ���� 0xFF
			}  
			   // 3.   save 	Minute		   1-59   s 		
			   VdrData.H_08[6+2*Temp_Gps_Gprs.Time[2]]=Speed_gps/10; //km/h
			   VdrData.H_08[7+2*Temp_Gps_Gprs.Time[2]]=Vehicle_sensor;	
  
		  //------------------------09H ---------  min=0;	sec=0	----------------------------
		   if((Temp_Gps_Gprs.Time[1]==0)&&(Temp_Gps_Gprs.Time[2]==0)) 
		   {
			   
				// 1.  �ж��Ƿ�����Ҫ�洢
					VdrData.H_09_saveFlag=1;  //   ����  ��ǰ���ӵ����ݼ�¼ 
			   //  2.  ������ ʱ����	 |	 ��  ����0
				 VdrData.H_09[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_09[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_09[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_09[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_09[4]=0;
				 VdrData.H_09[5]=0;
  
				 //  initial  0xFF
				 memset(VdrData.H_09+6,0xFF,660);	 // Ĭ���� 0xFF
  
		   }   
		   else
			// 3.  �ж��Ƿ��ǵ�һ���ϵ� ,  �������ô������ֵ
		   if((VdrData.H_09[0]==0x00)&&(VdrData.H_09[1]==0x00))  // ����Ϊ 0
		   {
				 VdrData.H_09[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_09[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_09[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_09[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_09[4]=0; 
				 VdrData.H_09[5]=0;
		   }   
  
		   //  4 .	��д����λ��	1-59 min
		   if(Temp_Gps_Gprs.Time[2]==59)  // 59 ��ʱ�������������0 ��ô���Ӿͱ���
		   {  
			  //   �� A.20		  (��ǰ���ӵ�λ��  +	��ǰ���ӵ�ƽ���ٶ�) 
			  memcpy( VdrData.H_09+6+Temp_Gps_Gprs.Time[1]*11,VdrData.Longi,4);  // ����
			  memcpy( VdrData.H_09+6+Temp_Gps_Gprs.Time[1]*11+4,VdrData.Lati,4); //γ��
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+8]=(GPS_Hight>>8);
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+9]=GPS_Hight; 		   
			  //  ��ǰ���ӵ�ƽ���ٶ�	 ��AvrgSpd_MintProcess()  ���õı���
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+10]= PerMinSpdTotal/AspdCounter;			  
		   }
		 //-------------------- 10H    �¹��ɵ�����  ---------------------------------
		 if((VdrData.H_10_saveFlag==0)&&(XinhaoStatus[11]=='1')&&(XinhaoStatusBAK[11]=='0')) 
		 {	//	ɲ��״̬�����仯
			  //  1.  ��ʻ����ʱ��
			 time_now=Get_RTC();	 //  RTC  ��� 
			 Time2BCD(VdrData.H_10); 
			 //   2.   ��������ʻ֤����
			 memcpy(VdrData.H_10+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
			 //   3.  �ٶȺ�״̬��Ϣ   
					  //-----------------------  Status Register   --------------------------------
					cur=save_sensorCounter;  //20s���¹��ɵ�
					for(j=0;j<100;j++)
					{
						VdrData.H_10[24+j*2]=Sensor_buf[cur].DOUBTspeed;	//�ٶ�
						VdrData.H_10[24+j*2+1]=Sensor_buf[cur].DOUBTstatus;   //״̬	
						
						if(cur>0)	  // �ӵ�ǰ��ǰ��ȡ
						   cur--;
						else
						   cur=100; 						   
					}  
			 // 	4.	λ����Ϣ
			  memcpy( VdrData.H_10+224,VdrData.Longi,4);  // ����
			  memcpy( VdrData.H_10+224+4,VdrData.Lati,4); //γ��
			  VdrData.H_10[224+8]=(GPS_Hight>>8);
					VdrData.H_10[224+9]=GPS_Hight;	
  
			 //--------------------------------------------------------------------------			  
			 VdrData.H_10_saveFlag=1;
		 }		
		 memcpy(XinhaoStatusBAK,XinhaoStatus,20);
		 // Vehicle_sensor_BAK=Vehicle_sensor;
  
   }
  
  
}


void VDR_product_11H_Start(void)
{
           //    11 H   ������ʻ��ʼʱ��   ����ʼλ��     ��60s ��ʼ���ʻ��ʼ
                   if(VDR_TrigStatus.Run_baseon_spd_10s_couter==60)
                   	{ 
                   	    //  1.   ��������ʻ֤��
                   	    memcpy(VdrData.H_11,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
					    //   2.   ��ʼʱ��
						time_now=Get_RTC();     //  RTC  ��� 
			            Time2BCD(VdrData.H_11+18); 
                        //   3.  ��ʼλ��                        
						memcpy( VdrData.H_11+30,VdrData.Longi,4);  // ����
						memcpy( VdrData.H_11+30+4,VdrData.Lati,4); //γ��
						VdrData.H_11[30+8]=(GPS_Hight>>8);
						VdrData.H_11[30+9]=GPS_Hight;	

                   	}
}

void VDR_product_11H_End(void)
{
         if(TiredConf_struct.TiredDoor.Door_DrvKeepingSec<=0) 
		 	 return;
            //         11 H     ���
        if(VDR_TrigStatus.Run_baseon_spd_10s_couter>TiredConf_struct.TiredDoor.Door_DrvKeepingSec)
        	{
                   
			
                    //   2.   ����ʱ��
						time_now=Get_RTC();     //  RTC  ��� 
			       Time2BCD((VdrData.H_11+25)); 
                        //   3.  ��ʼλ��                        
						memcpy( VdrData.H_11+40,VdrData.Longi,4);  // ����
						memcpy( VdrData.H_11+40+4,VdrData.Lati,4); //γ��
						VdrData.H_11[40+8]=(GPS_Hight>>8);
						VdrData.H_11[40+9]=GPS_Hight;	
                    //    4.   save 
                    VdrData.H_11_saveFlag=1; 
        	}
}




void VDR_product_12H(u8  value)    // ��ο���¼
{
     //   1. ʱ�䷢����ʱ��
		time_now=Get_RTC();     //  RTC  ��� 
        Time2BCD(VdrData.H_12); 

     //  2.   ��������ʻ֤��
         memcpy(VdrData.H_12+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
     //  3.  ��� 
         VdrData.H_12[24]=value;

	 // 4.  save 
	     VdrData.H_12_saveFlag=1; 
}


//  Note :  �����˵���û���ã������������£��ϵ����SPI DF  Σ����ܴ�
void VDR_product_13H(u8  value)    //  �ⲿ�����¼
{
       //   1. ʱ�䷢����ʱ��
		time_now=Get_RTC();     //  RTC  ��� 
        Time2BCD(VdrData.H_13); 
       //  2.  value
         VdrData.H_13[6]=value;
       //  3. save
       VdrData.H_13_saveFlag=1; 
}


void VDR_product_14H(u8 cmdID)    // ��¼�ǲ����޸ļ�¼
{

    //   1. ʱ�䷢����ʱ��
		time_now=Get_RTC();     //  RTC  ��� 
        Time2BCD(VdrData.H_14); 
       //  2.  value
         VdrData.H_14[6]=cmdID;
       //  3. save
       VdrData.H_14_saveFlag=1;
}


 
// note  :    ��Ҫ��������   08 H �����ݼ�¼ �����ﲻ��Ҫ��䡣
void VDR_product_15H(u8  value)    //  �ɼ�ָ���ٶȼ�¼ 
{
    




}

void  VDR_get_15H_StartEnd_Time(u8  *Start, u8 *End)  
{
    u8  i=0;

	for(i=0;i<6;i++)
	{
      VdrData.H_15_Set_StartDateTime[i]=Start[i];
	  VdrData.H_15_Set_EndDateTime[i]=End[i];
	}
	
}

//------- stuff     user data       ---------------
u16  stuff_drvData(u8 type,u16 Start_recNum,u16 REC_nums,u8 *dest)
{
    u16  res_len=0,get_len=0;
    u16  i=0;

	for(i=0;i<REC_nums;i++)
	{
	  switch(type)
	  	{
	  	   case 0x08:
                     get_len=get_08h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x09:
                     get_len=get_09h(Start_recNum+i,dest+res_len);    
					 break;
		   case 0x10:
                     get_len=get_10h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x11:
                     get_len=get_11h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x12:
                     get_len=get_12h(Start_recNum+i,dest+res_len);
					 break;	
		   case 0x13:
                     get_len=get_13h(Start_recNum+i,dest+res_len);
					 break;		
		   case 0x14:
                     get_len=get_14h(Start_recNum+i,dest+res_len);
					 break;		
		   case 0x15:
                     get_len=get_15h(Start_recNum+i,dest+res_len); 
					 break;	
		   default:
		   	         return 0;

	  	}
	   
	   res_len+=get_len;
	}    
    return   res_len;
}









// 00H  �ɼ��г���¼��ִ�б�׼�汾
u16   get_00h(u8  *p_in ,u16 inLen, u8 *p_out)   
{

 return 0;
}

// 01H  �ɼ���ǰ��ʻԱ����Ϣ
u16   get_01h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

//  02H   �ɼ���¼�ǵ�ʵʱʱ��
u16   get_02h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

// 03H  �ɼ��ۼ���ʻ���
u16   get_03h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_04h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_05h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_06h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_07h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

/*
   ��ʻ�ٶȼ�¼
   ���������Լ������ݣ��Լ���
   ����126�ֽ�ÿ����
   48*60=2880��

    ÿһ��   128 �ֽ�

 */

u16 get_08h( u16 indexnum,u8 *p)    
{
	int			i;
	uint8_t		buf[128];
    u32  addr=0;
	u8  FCS=0;


    
	//	  1.  get  address	 
	 addr=VDR_08H_START+indexnum*128;

   //    2. read   
      SST25V_BufferRead(buf, addr, 128 );
      DF_delay_ms(60);
       OutPrint_HEX("08H content",buf,128); // debug   
   //     3.  FCS  check
           FCS=0;
       for(i=0;i<127;i++)
	 	   FCS^=buf[i];
	   if(buf[127]!=FCS)
	   	{
	   	     rt_kprintf("\r\n  08H read fcs error  save:%X  cacu: %X \r\n",buf[127],FCS);
             return  0;
	   	}
     if(buf[0]==MarkByte)
      {  
         memcpy(p,buf+1,126);
	     return  126;
      }	
	 else 
	 	 return 0; 
}



//FINSH_FUNCTION_EXPORT( get_08h, get_08 );


/*
   λ����Ϣ
   360Сʱ   ��ÿСʱ666�ֽ�
 */

u16  get_09h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[669];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_09H_START+indexnum*768;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 669 ); //2+content 
	   DF_delay_ms(60);
	   OutPrint_HEX("09H content",buf,669); // debug 
	//	   3.  FCS	check
	          FCS=0;
		for(i=0;i<667;i++)
			FCS^=buf[i];
		if(buf[667]!=FCS)
		 {
			  rt_kprintf("\r\n	09H read fcs error	save:%X  cacu: %X \r\n",buf[667],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,666); 
		  return  666;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_09h, get_09 );


/*
   �¹��ɵ�
   234Byte
   ��100��
 */
u16 get_10h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[237];
	 u32  addr=0;
	 u8   FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_10H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 236 );	
	   DF_delay_ms(60); 
	   OutPrint_HEX("10H content",buf,236); // debug 
	//	   3.  FCS	check
	         FCS=0;
		for(i=0;i<235;i++)
			FCS^=buf[i];
		if(buf[235]!=FCS)
		 {
			  rt_kprintf("\r\n	10H read fcs error	save:%X  cacu: %X \r\n",buf[235],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,234); 
		  return  234;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_10h, get_10 );


/*��ʱ��ʻ��¼
   50Bytes   100��

      ÿ����¼   64   ���ֽ�
 */

u16 get_11h( u16 indexnum,u8 *p)   
{
	
	 int		 i;
	 uint8_t	 buf[60];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_11H_START+indexnum*64;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 52 );	
	   DF_delay_ms(10);
	   OutPrint_HEX("11H content",buf,52); // debug 
	//	   3.  FCS	check
	        FCS=0;
		for(i=0;i<51;i++)
			FCS^=buf[i];
		if(buf[51]!=FCS)
		 {
			  rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[51],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,50); 
		  return  50;
	   } 
	  else 
		  return 0; 

	
}

//FINSH_FUNCTION_EXPORT( get_11h, get_11 );


/*��ʻԱ��ݵ�¼
   25Bytes   200��
 */

u16 get_12h( u16 indexnum,u8 *p)  
{
     int		 i;
	 uint8_t	 buf[30];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_12H_START+indexnum*32;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 27 ); 
	   DF_delay_ms(10);
	   OutPrint_HEX("12H content",buf,27); // debug 
	//	   3.  FCS	check
	       FCS=0;
		for(i=0;i<26;i++)
			FCS^=buf[i];
		if(buf[26]!=FCS)
		 {
			  rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[26],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,25); 
		  return  25;
	   } 
	  else 
		  return 0; 
}

//FINSH_FUNCTION_EXPORT( get_12h, get_12 );


/*�ⲿ�����¼
   7�ֽ� 100��**/
u16 get_13h( u16 indexnum,u8 *p)  
{
    // int		 i;
	 uint8_t	 buf[10];
	 u32  addr=0;
	// u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_13H_START+indexnum*8;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 8 );	
	   OutPrint_HEX("13H content",buf,8); // debug 
	//	   3.  FCS	check
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_13h, get_13 );


/*��¼�ǲ���
   7�ֽ� 100��    8 */
u16 get_14h( u16 indexnum,u8 *p)  
{
  //   int		 i;
	 uint8_t	 buf[10];
	 u32  addr=0;
	// u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_14H_START+indexnum*8;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 8 );	
	   OutPrint_HEX("14H content",buf,8); // debug 
	//	   3.  FCS	check
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_14h, get_14 );


/*
   �ٶ�״̬��־
   133Byte    256
   ��10��
 */
u16 get_15h( u16 indexnum,u8 *p)  
{
	
     int		 i;
	 uint8_t	 buf[150];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_15H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 135 );
	   OutPrint_HEX("15H content",buf,135); // debug 
	//	   3.  FCS	check
		for(i=0;i<134;i++)
			FCS^=buf[i];
		if(buf[134]!=FCS)
		 {
			  rt_kprintf("\r\n	15H read fcs error	save:%X  cacu: %X \r\n",buf[134],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,133); 
		  return  133;
	   } 
	  else 
		  return 0;

}

//FINSH_FUNCTION_EXPORT( get_15h, get_15 );








//==========  write   ------------------------------------------------------------------------------
u16  vdr_creat_08h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[128];
	u8  FCS=0,i=0;
	u32 inaddress=0;//  д���ַ

    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>VDR_08_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_08H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   //  126 bytes
   //   3.   caculate  fcs
     for(i=0;i<127;i++)
	 	FCS^=inbuf[i];
	 inbuf[127]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_08H_START+indexnum*128;

	//  �жϲ����������
	if( ( inaddress & 0x0FFF ) == 0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );
				delay_ms(100);
			}
   //     5 .  write
     WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 128 );  
    delay_ms(50);

	return  indexnum;
}


//	666=>>768 rec_size   
u16  vdr_creat_09h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[669];     
	u8  FCS=0;
	u16 i=0;
	u32 inaddress=0;//  д���ַ
   
    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>VDR_09_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_09H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   // 666 bytes   
   //   3.   caculate  fcs
       for(i=0;i<667;i++)
	    	FCS^=inbuf[i];
	   
	   inbuf[667]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_09H_START+indexnum*768;

	//  �жϲ����������
	if( ( inaddress&0x0FFF)==0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );   
				delay_ms(100);
			}
   //     5 .  write
    WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 668 ); // 2+content
    delay_ms(80); 
   
     return  indexnum; 

} 

/*
   �¹��ɵ� Accident Point    234
 */
 
u16  vdr_creat_10h( u16 indexnum,u8 *p, u16 inLen) 
{
   
      u8  inbuf[256];
	  u8  FCS=0,i=0;
	  u32 inaddress=0;//  д���ַ
	 
	  //  1.  Stuff  Head 
	  inbuf[0]=MarkByte	;	   //
	  //  2.   Index  get  address	
	  if(indexnum>VDR_10_MAXindex)
		  {
			   Vdr_Wr_Rd_Offset.V_10H_Write=0; 
			   indexnum=0;
		  }
	  memcpy(inbuf+1,p,inLen);	 // 666 bytes
	 //   3.   caculate  fcs
	   for(i=0;i<235;i++)
		  FCS^=inbuf[i];
	   inbuf[235]=FCS;	 
	 //    4.  get	address   
	  inaddress=VDR_10H_START+indexnum*256;
   
	  //  �жϲ����������
	  if( ( inaddress & 0x0FFF ) == 0 )
			  {
				  WatchDog_Feed();
				  SST25V_SectorErase_4KByte( inaddress );
				  delay_ms(100);
			  }
	 // 	5 .  write
	  WatchDog_Feed(); 
	  SST25V_BufferWrite(inbuf, inaddress, 236 ); // 2+content 
	  delay_ms(10);
   
	   return  indexnum;

}

/***********************************************************
* Function:
* Description:       recordsize 5==>> 64 size 
* Input:             100   ��
* Input:
* Output:
* Return:
* Others:
***********************************************************/

u16  vdr_creat_11h( u16 indexnum,u8 *p, u16 inLen) 
{

   
	u8	inbuf[60];
	u8	FCS=0,i=0;
	u32 inaddress=0;//	д���ַ
   
	//	1.	Stuff  Head 
	inbuf[0]=MarkByte;		 //
	//	2.	 Index	get  address  
	if(indexnum>VDR_11_MAXindex)
		{
			 Vdr_Wr_Rd_Offset.V_11H_Write=0; 
			 indexnum=0;
		}
	memcpy(inbuf+1,p,inLen);   // 666 bytes
   //	3.	 caculate  fcs
	 for(i=0;i<51;i++)
		FCS^=inbuf[i];
	 inbuf[51]=FCS;   
   //	 4.  get  address	
	inaddress=VDR_11H_START+indexnum*64;
   
   //	  5 .  write   
	SST25V_OneSector_Write(inbuf, inaddress, 52 ); // 2+content     ����д  
     WatchDog_Feed(); 
	 return  indexnum;

}
/*
   �ⲿ�����¼
   ������4k�ļ�¼�1�ζ���

      25 =>>   32
 */

u16  vdr_creat_12h( u16 indexnum,u8 *p, u16 inLen) 
{
  
   u8  inbuf[30];
   u8  FCS=0,i=0;
   u32 inaddress=0;//  д���ַ
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_12_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_12H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
  //   3.	caculate  fcs
	for(i=0;i<26;i++)
	   FCS^=inbuf[i];
	inbuf[26]=FCS;	 
  //	4.	get  address   
   inaddress=VDR_12H_START+indexnum*32; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 27 ); // 2+content	   ����д   
    WatchDog_Feed(); 
   return	indexnum;

}


/*������4k�ļ�¼�1�ζ���
     7 bytes    >> 8       record size     

*/
u16  vdr_creat_13h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
  // u8  FCS=0,i=0;
   u32 inaddress=0;//  д���ַ
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_13_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_13H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
/*  //   3.	caculate  fcs
	for(i=0;i<8;i++)
	   FCS^=inbuf[i];
	inbuf[8]=FCS;	 */
  //	4.	get  address   
   inaddress=VDR_13H_START+indexnum*8; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   ����д   
   WatchDog_Feed();   
   return	indexnum;
    

}

/*������4k�ļ�¼�1�ζ���
     7 bytes    >> 8 	  record size 

*/
u16  vdr_creat_14h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
   //u8  FCS=0,i=0;
   u32 inaddress=0;//  д���ַ
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_14_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_14H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
/*  //   3.	caculate  fcs
	for(i=0;i<8;i++)
	   FCS^=inbuf[i];
	inbuf[8]=FCS;	 */
  //	4.	get  address   
   inaddress=VDR_14H_START+indexnum*8; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   ����д   
   WatchDog_Feed(); 
   return	indexnum;
  
}

/*������4k�ļ�¼�1�ζ���
       133   >>  256 
*/
u16  vdr_creat_15h( u16 indexnum,u8 *p, u16 inLen) 
{
	
	 u8  inbuf[260];
	 u8  FCS=0,i=0;
	 u32 inaddress=0;//  д���ַ
	
	 //  1.  Stuff	Head 
	 inbuf[0]=MarkByte; 	  //
	 //  2.   Index  get  address  
	 if(indexnum>VDR_15_MAXindex)
		 {
			  Vdr_Wr_Rd_Offset.V_15H_Write=0; 
			  indexnum=0;
		 }
	 memcpy(inbuf+1,p,inLen);	// 666 bytes
	//	 3.   caculate	fcs
	  for(i=0;i<134;i++)
		 FCS^=inbuf[i];
	  inbuf[134]=FCS;   
	//	  4.  get  address	 
	 inaddress=VDR_15H_START+indexnum*256; 
	
	//	   5 .	write	
	 SST25V_OneSector_Write(inbuf, inaddress, 135); // 2+content	 ����д   
	 WatchDog_Feed(); 
	 return   indexnum;

}

//---------------------------------------------------------------------------------

/************************************** The End Of File **************************************/
