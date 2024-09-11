#include<reg51.h>
#include "keypad.h"
#include "lcd4bit.h"
int n1,n2,mflag=0,sign=0;
long int res;
void main(void)
{
	char s;
	init();
	cmd_lcd(0x83);
	write_str_to_lcd("WELCOME TO");
	cmd_lcd(0xc3);
	write_str_to_lcd("CALCULATOR");
	delay(100);
	//taking the first operand
	while(1)
	{
		mflag=0;//clearing the invalid_data flag
		sign=0;//clearing sign flag
		cmd_lcd(0x01);
		write_str_to_lcd("Enter first no:");
		cmd_lcd(0xc0);
		n1=0;
		s=0;
		while(s!='=')
		{
			if(s!=0)
				write_to_lcd(s);
			if(s>=48&&s<=57&&s!='k')//48 and 57 are teh ascii values of 0 and 9
				n1=n1*10+s-48;//if we subtract 48 from charecter thwn we get integer conversion of that charecter
			else
			{
				if(s!=0)
				{
					cmd_lcd(0xc0);
					write_str_to_lcd("Invalid data!");//someother input other than numbers is treated as invalid 
					mflag=1;//raising the flag to restart the system if invalid_data is given as input
					break;
				}
			}
			delay(10);
			//This collects the first charecter for next number
			while(1)
			{
				s=get_ans();
				if(s=='k')
					continue;
				else
					break;
			}
			delay(10);
		}
		if(mflag)//if invalid_data is occured we reinitialize the calculator
			continue;
		delay(100);
		cmd_lcd(0x01);
		write_str_to_lcd("Enter second no:");
		cmd_lcd(0xc0);
		n2=0;
		s=0;
		while(s!='='&&mflag==0)//we have to enter a = after entering every input
		{
			if(s!=0)
				write_to_lcd(s);
			if(s>=48&&s<=57&&s!='k')
				n2=n2*10+s-48;
			else
			{
				if(s!=0)
				{
					cmd_lcd(0xc0);
					write_str_to_lcd("Invalid data!");
					mflag=1;
					break;
				}
			}
			delay(10);
			while(1)
			{
				s=get_ans();
				if(s=='k')
					continue;
				else
					break;
			}
			delay(10);
		}
		if(mflag)
			continue;
		delay(100);
		cmd_lcd(0x01);
		write_str_to_lcd("Enter the oper'n:");//collecting the operation
		cmd_lcd(0xc0);
		while(1)
		{
			s=get_ans();
			if(s=='k')
				continue;
			else
				break;
		}
		cmd_lcd(0x01);
		res=-1;
		switch(s)
		{
			case '+':
				res=n1+n2;
				break;
			case '-':
				//if the resultant is a negative number then a sign flag is raised
				if(n1>n2)
					res=n1-n2;
				else
				{
					res=n2-n1;//abs value is calculated ie making sure the output is positive
					sign=1;
				}
				break;
			case '*':
				res=n1*n2;
				break;
			case '/':
				if(n2==0)
				{
					write_str_to_lcd("Can't divide");
					cmd_lcd(0xc0);
					write_str_to_lcd("by zero!");
				}
				else
					res=n1/n2;
					break;
			default:
				write_str_to_lcd("Invalid operation!");
				break;
		}
		if(res!=-1)
		{
			display_int(n1);
			write_to_lcd(s);
			display_int(n2);
			write_to_lcd('=');
			cmd_lcd(0xc0);
			if(sign)//if we have a sign flag then we display - intentionally
			write_to_lcd('-');
			display_int(res);
		}
		delay(150);
	}
}