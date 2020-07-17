//无线型号:NRF24L01
//单片机型号：STC89C52RC  
//单片机晶振：11.0592MHZ
//验证单位：沈阳维恩电子科技有限公司 
//最新验证时间2016-3-28
//欢迎关注：WWW.RFINCHINA.COM  无线中国 有你 有我 愿与您一起迈向成功
//基本功能:2.4GHZ无线双向通信功能测试

//********************************************************************************************
#include <reg52.h>
#include <intrins.h>

typedef unsigned char uchar;
typedef unsigned char uint;
unsigned int t=0;
unsigned char zkb1=0 ;    //**左边电机的占空比**//
unsigned char zkb2=0 ;    //**右边电机的占空比**//
sbit IN1=P2^0;
sbit IN2=P2^1;
sbit IN3=P2^2;
sbit IN4=P2^3;
sbit ENA=P2^4;
sbit ENB=P2^5;


//*********************************NRF24L01/NRF24L01+ IO端口定义******************************

sbit 	MISO	=P1^1;
sbit 	MOSI	=P1^2;
sbit	SCK	    =P1^3;
sbit	CE	    =P1^5;
sbit	CSN		=P1^4;
sbit	IRQ		=P1^0;

//**********************************收发数据缓冲区*********************************************

uchar 	TxBuf[32];	 				// 预设发送缓冲区	
uchar 	RxBuf[32];	 				// 预设接收缓冲区

//**********************************按键IO端口定义*********************************************

sbit	KEY1=P2^0;
sbit	KEY2=P3^7;

//**********************************LED指示灯**************************************************

sbit    LED1=P0^3;                  //指示灯
sbit    LED2=P0^4;				    //指示灯

//*********************************************************************************************

uint 	bdata sta;  				//NRF24L01/NRF24L01+状态标志
sbit	RX_DR	=sta^6;
sbit	TX_DS	=sta^5;
sbit	MAX_RT	=sta^4;

//**********************************NRF24L01地址长度设置***************************************

#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width

uint const  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//本地地址
uint const  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//接收地址

//**********************************NRF24L01收发数据长度设置***********************************

#define TX_PLOAD_WIDTH  32  	// 32 uints TX payload
#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload

//**********************************串口通信相关变量*******************************************

uchar 	flag1,flag2;

//***************************************NRF24L01寄存器指令************************************

#define READ_REG        0x00  	// 读寄存器指令
#define WRITE_REG       0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留

//*************************************SPI(nRF24L01)寄存器地址*********************************

#define CONFIG          0x00    // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01    // 自动应答功能设置
#define EN_RXADDR       0x02    // 可用信道设置
#define SETUP_AW        0x03    // 收发地址宽度设置
#define SETUP_RETR      0x04    // 自动重发功能设置
#define RF_CH           0x05    // 工作频率设置
#define RF_SETUP        0x06    // 发射速率、功耗功能设置
#define STATUS          0x07    // 状态寄存器
#define OBSERVE_TX      0x08    // 发送监测功能
#define CD              0x09    // 地址检测           
#define RX_ADDR_P0      0x0A    // 频道0接收数据地址
#define RX_ADDR_P1      0x0B    // 频道1接收数据地址
#define RX_ADDR_P2      0x0C    // 频道2接收数据地址
#define RX_ADDR_P3      0x0D    // 频道3接收数据地址
#define RX_ADDR_P4      0x0E    // 频道4接收数据地址
#define RX_ADDR_P5      0x0F    // 频道5接收数据地址
#define TX_ADDR         0x10    // 发送地址寄存器
#define RX_PW_P0        0x11    // 接收频道0接收数据长度
#define RX_PW_P1        0x12    // 接收频道0接收数据长度
#define RX_PW_P2        0x13    // 接收频道0接收数据长度
#define RX_PW_P3        0x14    // 接收频道0接收数据长度
#define RX_PW_P4        0x15    // 接收频道0接收数据长度
#define RX_PW_P5        0x16    // 接收频道0接收数据长度
#define FIFO_STATUS     0x17    // FIFO栈入栈出状态寄存器设置

//**********************************子函数申明*********************************************

void Delay(unsigned int s);
void inerDelay_us(unsigned char n);
void init_NRF24L01(void);
uint SPI_RW(uint uchar);
uchar SPI_Read(uchar reg);
void SetRX_Mode(void);
uint SPI_RW_Reg(uchar reg, uchar value);
uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars);
uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);
void nRF24L01_TxPacket(unsigned char * tx_buf);

/******************************************************************************************
/*长延时函数
/******************************************************************************************/

void Delay(unsigned int s)
{
	unsigned int i;
	for(i=0; i<s; i++);
	for(i=0; i<s; i++);
}

/******************************************************************************************
/*短延时函数
/******************************************************************************************/
void inerDelay_us(unsigned char n)
{
	for(;n>0;n--)
		_nop_();
}

//**********************************子函数申明*********************************************
// 函数名称： UART_init()串口初始化函数
// 函数功能： 在系统时钟为11.059MHZ时，设定串口波特率为9600bit/s
//            串口接收中断允许，发送中断禁止
//**********************************子函数申明*********************************************

void UART_init()
{
                     //初始化串行口和波特率发生器 
	SCON =0x58;          //选择串口工作方式，打开接收允许
	TMOD =0x21;          //定时器1工作在方式2，定时器0工作在方式1
	TH1 =0xfd;           //实现波特率9600（系统时钟11.0592MHZ）
	TL1 =0xfd;
	TR1 =1;              //启动定时器T1
	ET1 =0; 
	ES=1;                //允许串行口中断
	PS=1;                //设计串行口中断优先级
	EA =1;               //单片机中断允许
}

//**********************************子函数申明*********************************************
// 函数名称： com_interrup()串口接收中断处理函数
// 函数功能： 接收包括起始位'S'在内的十位数据到数据缓冲区
//**********************************子函数申明*********************************************

com_interrupt(void) interrupt 4 using 3
{
  if(RI)                                //处理接收中断
	  {
	 	TxBuf[flag1]=SBUF; 
		RI=0;                                //清除中断标志位
		flag1++;
		if(flag1==32)       				       
		  {
			flag1=0;
			flag2=1;
		  }
	  }
}

//**********************************子函数申明*********************************************
//函 数:	 R_S_Byte(uchar R_Byte)
//功 能:	 数据发送	 
//**********************************子函数申明*********************************************

void R_S_Byte(uchar R_Byte)
{	
	 SBUF = R_Byte;  
     while( TI == 0 );				//查询法
  	 TI = 0;
}

//****************************************************************************************
/*NRF24L01初始化
//***************************************************************************************/

void init_NRF24L01(void)
{
    inerDelay_us(100);
 	CE=0;    // StandBy I模式	
 	CSN=1;   // SPI 片选禁止
 	SCK=0;   
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      //  频道0自动	ACK应答允许	
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        //   设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   		//设置发射速率为1Mbps，发射功率为最大值0dB	

}

/********************************************************************************************
/*函数：uint SPI_RW(uint uchar)
/*功能：NRF24L01的SPI写时序
/********************************************************************************************/

uint SPI_RW(uint uchar)
{
	uint bit_ctr;
   	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // output 8-bit
   	{
		MOSI = (uchar & 0x80);         // output 'uchar', MSB to MOSI
		uchar = (uchar << 1);           // shift next bit into MSB..
		SCK = 1;                      // Set SCK high..
		uchar |= MISO;       		  // capture current MISO bit
		SCK = 0;            		  // ..then set SCK low again
   	}
    return(uchar);           		  // return read uchar
}

/********************************************************************************************
/*函数：uchar SPI_Read(uchar reg)
/*功能：NRF24L01的SPI时序
/********************************************************************************************/

uchar SPI_Read(uchar reg)
{
	uchar reg_val;
	CSN = 0;                // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	CSN = 1;                // CSN high, terminate SPI communication
	return(reg_val);        // return register value
}

/********************************************************************************************/
/*功能：NRF24L01读写寄存器函数
/********************************************************************************************/

uint SPI_RW_Reg(uchar reg, uchar value)
{
	uint status;
	CSN = 0;                   // CSN low, init SPI transaction
	status = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	CSN = 1;                   // CSN high again
	return(status);            // return nRF24L01 status uchar
}

/*********************************************************************************************/
/*函数：uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
/*功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
/*********************************************************************************************/

uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uint status,uchar_ctr;
	CSN = 0;                    		// Set CSN low, init SPI tranaction
	status = SPI_RW(reg);       		// Select register to write to and read status uchar
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)
		pBuf[uchar_ctr] = SPI_RW(0);    // 
	CSN = 1;                           
	return(status);                    // return nRF24L01 status uchar
}

/**********************************************************************************************
/*函数：uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
/*功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
/**********************************************************************************************/

uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uint status,uchar_ctr;
	CSN = 0;           										 //SPI使能       
	status = SPI_RW(reg);   
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++) 
	SPI_RW(*pBuf++);
	CSN = 1;          										 //关闭SPI
	return(status);    
}

/***********************************************************************************************/
/*函数：void SetRX_Mode(void)
/*功能：数据接收配置 
/***********************************************************************************************/

void SetRX_Mode(void)
{
	CE=0;
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   				// IRQ收发完成中断响应，16位CRC	，主接收
	CE = 1; 
	inerDelay_us(130);
}

/************************************************************************************************/
/*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
/*功能：数据读取后放如rx_buf接收缓冲区中
/************************************************************************************************/
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;
	sta=SPI_Read(STATUS);									// 读取状态寄存其来判断数据接收状况
	if(RX_DR)												// 判断是否接收到数据
	{
	    CE = 0; 											//
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);	// read receive payload from RX_FIFO buffer
		revale =1;											//读取数据完成标志
	}
	SPI_RW_Reg(WRITE_REG+STATUS,sta);  						//接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
	return revale;
}

/*************************************************************************************************
/*函数：void nRF24L01_TxPacket(unsigned char * tx_buf)
/*功能：发送 tx_buf中数据
/*************************************************************************************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE=0;																//StandBy I模式	
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 	// 装载数据	
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);   					 		// IRQ收发完成中断响应，16位CRC，主发送
	CE=1;		 														//置高CE，激发数据发送
	inerDelay_us(10);
}

void mit()

{  TMOD=0x01;
   TH0=(65535-50)/256;
   TL0=(65535-50)%256;
   EA=1;
   ET0=1;
   TR0=1;
 }
//***********中断函数+脉宽调制***********//
void timer0() interrupt 1
{ 
	if(t<zkb1)
			 ENA=1;
		else 
			 ENA=0;
		if(t<zkb2)
				ENB=1;
		else 
				ENB=0;
				t++;
		if(t>=100)
				{t=0;}
			TH0=(65535-50)/256;
		 TL0=(65535-50)%256;
}


tz()
{IN1=1;
	IN2=1;
	IN3=1;
	IN4=1;
	zkb1=0;
	zkb2=0;
}
ht()
{IN1=0;
	IN2=1;
	IN3=0;
	IN4=1;
	zkb1=65;
	zkb2=65;
}


qianjin()
{IN1=1;
	IN2=0;
	IN3=1;
	IN4=0;
	zkb1=70;
	zkb2=70;
}
turnleft1()
{IN1=1;
	IN2=0;
	IN3=1;
	IN4=0;
	zkb1=38;
	zkb2=65;
	
}
turnright1()
{IN1=1;
	IN2=0;
	IN3=1;
	IN4=0;
	zkb1=65;
	zkb2=38;
	
}



//************************************主函数******************************************************
void main(void)
{
	uchar i =0,tf=0;	

    init_NRF24L01() ;				//初始化 NRF24L01/NRF24L01+	

	CSN = 0;    
	SPI_RW(FLUSH_RX);
	SPI_RW(FLUSH_TX);
	CSN = 1;    
	Delay(600);	
	 mit();					
	UART_init();					//初始化 UART

//*************************************************************************************************
	while(1)
	{

		SetRX_Mode();//接收数据

		if(nRF24L01_RxPacket(RxBuf))
		{

				LED1=0;			LED2=0;			//LED闪烁，表示接收进行中

			for (i = 0;i< 1;i++)
				{ 
					R_S_Byte(RxBuf[i]);			//将收到的数据串口输出，可以通过下载线和串口助手观测数据
				}	


	   		   CSN = 0;    
			   SPI_RW(FLUSH_RX);
			   CSN = 1; 
			switch(RxBuf[0])

		{

				case 0xfe:
									
				         qianjin();//收到指令1，小车前进
									break;                                           
  //*********************************************************************
				 case 0xf7:                                         
									
				          turnright1();//收到指令2，小车右转
									 break;
   //*********************************************************************
				 case 0xfd:                                   
									 
				           ht();//收到指令3，小车后退
									 break;
   //*********************************************************************
				 case 0xfb:                                         

										
                    turnleft1(); 	//收到指令4，小车左转			 
										break;
   //*********************************************************************
				 case 0xef:  
                  tz();                        
									
									break;//收到指令5，小车停止
   //*********************************************************************
       case 0xdf:  
                  zkb1=100;//收到指令7，小车加速
	                zkb2=100;
									
									break; 
   //*********************************************************************
				 case 0xbf:  
                  zkb1=30;//收到指令8，小车减速
	                zkb2=30;
									
									break; 

			 default:   tz();                
										

		}


		}
//***********************检测串口数据，当串口有数据时发送******************************************
//	  if(flag2==1)										//当收到串口数据后发送
//			{
//
//               nRF24L01_TxPacket(TxBuf);				// 发送数据
//			   sta=SPI_Read(STATUS);	
//			   SPI_RW_Reg(WRITE_REG+STATUS,sta);
// 
//			   flag2=0;	
//			   LED1=0;		 LED2=0;				  //LED闪烁，表示发送进行中
//			   inerDelay_us(1000);
//
//			}	
//***********************检测串口数据，当串口有数据时发送********************************************
	
//	   if(KEY1 ==0 ) 				//当KEY1（P2.0)和地短接时
//		  	{
//			   	LED1=0;	LED2=1;	TxBuf[0] = 0x20 ;tf = 1 ; 
//		    }

//*************************************************************************************************	
//	   if(KEY2 ==0)			       //当KEY2(P3.7)和地短接时
//		   {
//			    LED1=1;	LED2=0;	TxBuf[0] = 0x37 ;tf = 1 ; 
//		   }
//*************************************************************************************************
//	   if (tf==1)
//	       {	
//				nRF24L01_TxPacket(TxBuf);	 		    //发送数据
//			    sta=SPI_Read(STATUS);	
//			    SPI_RW_Reg(WRITE_REG+STATUS,sta);
//
//				tf=0; 
//			    inerDelay_us(1000);  
//
//		   }
//				LED1=1;	LED2=1;	
	}
	
}
