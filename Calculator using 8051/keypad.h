#include<reg51.h>
/*O stands for on and off
R stands for memory recall
C stands for change of sign
D stands for m-(decrement)
I stands for m+(increment)
S stands for squareroot*/
char keypad[4][4]={{'7','8','9','/'},{'4','5','6','*'},{'1','2','3','-'},{'O','0','=','+'}};
char ans;
sbit col0=P2^0;
sbit col1=P2^1;
sbit col2=P2^2;
sbit col3=P2^3;
sbit row0=P2^4;
sbit row1=P2^5;
sbit row2=P2^6;
sbit row3=P2^7;
char get_ans()
{
	int i,j;
	for(i=0;i<4;i++)
	{
		switch(i)
		{
			case 0:
				row0=0;
				row1=1;
				row2=1;
				row3=1;
				break;
			case 1:
				row0=1;
				row1=0;
				row2=1;
				row3=1;
				break;
			case 2:
				row0=1;
				row1=1;
				row2=0;
				row3=1;
				break;
			case 3:
				row0=1;
				row1=1;
				row2=1;
				row3=0;
				break;
		}	
		for(j=0;j<4;j++)
		{
			switch(j)
			{
				case 0:
					if(col0==0)
						return keypad[i][j];
					break;
				case 1:
					if(col1==0)
							return keypad[i][j];
					break;
				case 2:
					if(col2==0)
						return keypad[i][j];
					break;
				case 3:
					if(col3==0)
							return keypad[i][j];
					break;;
			}
		}
	}
	return 'k';
}
