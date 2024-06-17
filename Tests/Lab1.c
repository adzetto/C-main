#include <stdio.h>

main()
{
	
	float x,y,k,l,m,n,sum;
	printf("\nPlease enter x and y separated by spaces :");
	scanf("%f\t%f",&x,&y);
	
	sum=x+y;
	
	if (sum > 0)
		k=x-y;
    else if (sum <= 0)
    	k=x*x+y;
    	
   if(sum <= 0)
       l=x+1.0/3.0;
   else if (sum > 0 && sum <= 10 )
       l=sum/(x*y+1);
   else if (sum > 10)
       l=y+1.0/3.0;
       
   if (x == y)
       m=7*x+5;
   else
       m=3*y+6;
       
   n=l;
   
   if (k > l)
      n=k;
      
   if (m > n)
      n=m;
      
   printf("\nk=%.6f",k);
   printf("\nl=%.6f",l);
   printf("\nm=%.6f",m);
   printf("\nn=%.6f",n);   
    
   return 0;	
	
}
