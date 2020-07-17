/***************   writer:shopping.w   ******************/
#include <reg52.h>
#include <intrins.h>
#define uint unsigned int
#define uchar unsigned char
sbit K1   = P1^0;
sbit K2   = P1^1;
sbit K3   = P1^2;

sbit K4   = P1^3;
sbit K5   = P1^4;
sbit LED1 = P0^0;
sbit LED2 = P0^1;
sbit LED3 = P0^2;
sbit MA   = P2^0;
sbit MB   = P2^1;
sbit MC   = P2^2;
sbit MD   = P2^3;
void main(void)
{
	LED1 = 1;
	LED2 = 1;
	LED3 = 0;
	while(1)
	{
	 	if(K1 == 0)
		{
		 	while(K1 == 0);
			LED1 = 0;
			LED2 = 1;
			LED3 = 1;
			MA   = 0;
			MB   = 1;
			MC   = 0;
			MD   = 1;
		}
		if(K2 == 0)
		{
		 	while(K1 == 0);
			LED1 = 1;
			LED2 = 0;
			LED3 = 1;
			MA   = 1;
			MB   = 0;
			MC   = 1;
			MD   = 0;
		}
		if(K3 == 0)
		{
		 	while(K1 == 0);
			LED1 = 1;
			LED2 = 1;
			LED3 = 0;
			MA   = 0;
			MB   = 0;
		    MC   = 0;
			MD   = 0;
		}
	   	if(K4 == 0)
		{
		 	while(K1 == 0);
			
			MA   = 0;
			MB   = 1;
		    MC   = 0;
			MD   = 0;
		}
	   	if(K5 == 0)
		{
		 	while(K1 == 0);
		
			MA   = 0;
			MB   = 0;
		    MC   = 0;
			MD   = 1;
		}


	}
}
						