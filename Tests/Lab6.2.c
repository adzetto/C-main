#include <stdio.h>
#include <math.h>

#define MAXITERATION 20
#define PI 3.14156
#define MAXANGLEDEGREES 360

float cosine(float x,float precision);
float sine(float x,float precision);
float last_difference=0;

main()
{
		
    float x,delta_x,precision,cosinesum,sinesum,degree,delta_degrees;
    delta_degrees=30;
    
	printf("\nPlease enter the precision :");
	scanf("%f",&precision);
			        			
	degree=0;
	
	while (degree <= MAXANGLEDEGREES)
    {
	
	    x=(degree/180)*PI;
	    
	    printf("\nDegree  : %3f\tRadian : %2.2f\tCos(x) : %.6f\tLast Difference : %.6f\tPrecision : %.6f",degree,x,cosine(x,precision),last_difference,precision);	
	    printf("\nDegree  : %3f\tRadian : %2.2f\tSin(x) : %.6f\tLast Difference : %.6f\tPrecision : %.6f",degree,x,sine(x,precision),last_difference,precision);		
		
		degree += delta_degrees;
	}
	
}

float cosine(float x,float precision)
{
   
    float term=1.0;
    float sum=1.0;
    float previoussum=0.0;
    int i=1;
    
    while (i<=MAXITERATION)
    {
    	
		term = -(term*x*x)/(2*i*(2*i-1));
		sum += term;
    	i++; 
		
		if (fabs(sum-previoussum) <= precision) 
			break;
		
		previoussum=sum;	
    	
	}
    
    last_difference = fabs(sum-previoussum);
    
   	return sum;
	
}


float sine(float x,float precision)
{
   
    float term=1.0;
    float sum=x;
    float previoussum=0.0;
    int i=1;
    
    while (i<=MAXITERATION)
    {
    	
		term = -(term*x*x)/((2*i+1)*(2*i));
		sum += term;
    	i++; 
		
		if (fabs(sum-previoussum) <= precision) 
			break;
		
		previoussum=sum;	
    	
	}
        
    last_difference = fabs(sum-previoussum);    
        
	return sum;
	
}



