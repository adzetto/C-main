#include <stdio.h>
#include <math.h>

float find_closest(float series[],float arraylength,float p);

main()
{
	
	int n0,n,arraysize;
	float r,p,mean,var,closestvalue;
	int i;
	
	printf("\nPlease enter n0,n,r and p seperated by a space : ");
	scanf("%d %d %f %f",&n0,&n,&r,&p);
	
	arraysize=n-n0+1;
	float a[arraysize];
	
	
	i=0;
	mean = 0.0;
	
	while (i < arraysize){
		
		a[i] = pow(i+n0,2)*pow(r,i+n0);
		printf("\na[%d]=%f",i+n0,a[i]);
		
	    mean += a[i];
		i++;
		
		
	}
	
	printf("\n");
	printf("\nSum = %f ",mean);
	mean = mean/arraysize;
	printf("\nMean = %f ",mean);
	
	
	i=0;
	var = 0.0;
	
	while (i<arraysize){
		
	    var += pow((a[i]-mean),2);
		i++;
	}
	
	var = var/arraysize;
	printf("\nVariance = %f ",var);
	
	closestvalue=find_closest(a,arraysize,p);
	
	printf("\nClosest = %f ",closestvalue);
		
}

float find_closest(float series[],float arraylength,float p)
{
	
	int i=0;
	float minimumdistance,closest;
	
	minimumdistance=abs(series[i]-p);
	
	for (i=1; i <arraylength; i++)
		if (abs(series[i]-p) < minimumdistance){
			minimumdistance=abs(series[i]-p);
			closest=series[i];
		}
				
	return closest;
		
}


