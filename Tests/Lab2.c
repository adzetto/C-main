#include <stdio.h>
#include <math.h>

main()
{
	
	float a,b,c,discriminant,x1,x2,x1re,x1im,x2re,x2im;
	
	printf("\nPlease enter the coefficients a (non-zero), b and c separated by spaces :");
	scanf("%f %f %f",&a,&b,&c);
	
	if ( a == 0 ){
		
		printf("\nError : a cannot be zero,please enter a non-zero number for a !:");
		scanf("%f",&a);		
		
	}
	
	discriminant=b*b-4*a*c;
	
	if (discriminant == 0)
	{
				
		x1=x2=-b/(2*a);
		printf("\nDiscriminant is %f ",discriminant);
		printf("\nThere are two identical roots ");
		printf("\nx1 = x2 = %6.3f",x1);
		
	}
	else if (discriminant > 0)
	{
		x1 = (-b+sqrt(discriminant))/(2*a);
		x2 = (-b-sqrt(discriminant))/(2*a);
		
	    printf("\nDiscriminant is %f ",discriminant);
		printf("\nThere are two real roots ");
		printf("\nx1 = %6.3f and x2 = %6.3f",x1,x2);
		
	}
	else if (discriminant < 0)
	{
		
		x1re=x2re=-b/(2*a);
		x1im = sqrt(-discriminant)/(2*a);
		x2im = -sqrt(-discriminant)/(2*a);
		
	    printf("\nDiscriminant is %f ",discriminant);
		printf("\nThere are no real roots ");
		printf("\nx1re = %6.3f and x1im = %6.3f and ",x1re,x1im);		
		printf("\nx2re = %6.3f and x2im = %6.3f",x2re,x2im);		
		
	}
		
	return 0;
	
}
