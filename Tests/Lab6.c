#include <stdio.h>
#include <math.h>
#define MAXHEXDIGITS 20

/*Hex to decimal converter.We assume that the Hexadecimal number is positive integer without a sign in front of it*/

int HextoInt(char s[]);
void inttoBinary(int n,char s[]);

main()
{
	
	char HexString[MAXHEXDIGITS];
	char BinaryString[30];
	long int m;
	
	printf("\nPlease enter the hexadecimal number :");
	scanf("%s",&HexString);
	printf("\nHex Number = %s",HexString);
	m=HextoInt(HexString);
	printf("\nHexadecimal number %s is equal to %d as decimal number.",HexString,m);
	inttoBinary(m,BinaryString);
	printf("\nDecimal number %d is equal to %s as binary number.",m,BinaryString);
			
}
int HextoInt(char s[])
{
	long int n;
	int i;
	i=n=0;
	
	while (s[i] != '\0')
	{
		
		if (s[i] >= '0' && s[i] <= '9'){
			
			n=16*n+(s[i]-'0');
		}
		else if (s[i] >= 'A'&& s[i] <= 'F'){
			
			n=16*n+(s[i]-'A'+10);
		}
	   else if (s[i] >= 'a' && s[i] <= 'f'){
			
			n=16*n+(s[i]-'a'+10);
		}
		else{
			
			printf("\nError : %c is an invalid character.Hexadeciaml number mus have digits from 0 to 9,characters A to Z or a to z !");
			return 0;
		}
		
		i=i+1;		
	}
	
	return n;
		
}

void inttoBinary(int n,char s[])
{
	
	int i=0;
	int b;
	int numberofbinarydigits;
	
	numberofbinarydigits=floor(log(n)/log(2))+1;

	while (n >= 2)
	{
		b=(n % 2);
		s[numberofbinarydigits-i-1]=b+'0';
	    n /= 2;
		i++;
	}
		
	s[numberofbinarydigits-i-1]=n+'0';
	i++;
	s[i]='\0';
	
	printf("\ns=%s",s);
		
}













