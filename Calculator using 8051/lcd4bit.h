#include<reg51.h>
#define lcd P1
sbit rs=P1^3;
sbit en=P1^2;
void delay(unsigned int k)
{
	int i;
	for(i=0;i<250*k;i++);
}
void write_to_lcd(unsigned char s)
{
	lcd=s&(0xf0);
	rs=1;
	en=1;
	delay(1);
	en=0;
	lcd=s<<4;
	lcd=lcd&(0xf0);
	rs=1;
	en=1;
	delay(1);
	en=0;
}
void cmd_lcd(unsigned char s)
{
	lcd=s&(0xf0);
	rs=0;
	en=1;
	delay(1);
	en=0;
	lcd=s<<4;
	lcd=lcd&(0xf0);
	rs=0;
	en=1;
	delay(1);
	en=0;
}
void display_int(long int k)
{
	//first the number is reversed
	//and then the digits are collected digit wise by using %operator
	//+48 is done to convert the digit into a displayable charecter
	int m=0,count=0;
	if(k==0)
	{
		write_to_lcd('0');
		return;
	}
	while(k)
	{
		m=m*10+k%10;
		if(k%10==0)
			count++;
		k=k/10;
	}
	while(m)
	{
		write_to_lcd(m%10+48);
		m=m/10;
	}
	for(k=0;k<count;k++)
	write_to_lcd('0');
	return;
		
}
void write_str_to_lcd(unsigned char str[])
{
	int i;
	for(i=0;str[i]!='\0';i++)
		write_to_lcd(str[i]);
}
void init(void)
{
	cmd_lcd(0x28);
	cmd_lcd(0x02);
	cmd_lcd(0X01);//CLEAR
	cmd_lcd(0X06);// ENTRY MODE AUTO INCREMENT
	cmd_lcd(0X0F);//DISPLAY ON CURSOR BLINKNG
}