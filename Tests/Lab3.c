#include <stdio.h>


int power(int base,int exponent);
int Sum(int base,int exponent);

main()
{
	
	int base,exponent,result,sumresult;
	
	printf("\nPlease enter the base and the exponent separated by space :",&base,&exponent);
	scanf("%d %d",&base,&exponent);
	
	result=power(base,exponent);
	printf("\nThe result of (%d,%d) is %d",base,exponent,result);
	
	sumresult=Sum(base,exponent);
	printf("\nThe result of (%d,%d) is %d",base,exponent,sumresult);
		
}

int power(int base,int exponent)
{
	
	int i=1;
	int prod=1;
	
	while (i<=exponent){
		
		prod *= base;
		i++;
	}
	
	return prod;
		
}

int Sum(int base,int exponent)
{
	
	int total=0;
	int i;
		
	for (i=1;i<=exponent;i++)
		total += power(base,i);
		
    return total;
		
}







