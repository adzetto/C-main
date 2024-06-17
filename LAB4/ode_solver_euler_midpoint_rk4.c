#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double fy(double x){	return (exp(-(pow(x,5)/5) + (pow(x,3)/3) - pow(x,2)/2));    }

double fdy(double x, double y){		return (y*((-1*pow(x,4))+pow(x,2)-x));		}

void euler(double (*fYdx1)(double, double), double xFirst, double yFirst, double xLast, double stepSize, double *eulerResult){
	
	*(eulerResult) = yFirst;
	
	for( int i=1 ;xFirst <= xLast ; xFirst += stepSize, i++ ){
		yFirst += stepSize*fYdx1(xFirst,yFirst);
//		printf("%lf\n",yFirst);
		*(eulerResult+i) = yFirst;
	}	
}

void midpoint(double (*fYdx2)(double, double), double xFirst, double yFirst, double xLast, double stepSize, double *midpointResult){
	
	*(midpointResult) = yFirst;
	
	for( int i=1 ;xFirst <= xLast ; xFirst += stepSize, i++ ){
		yFirst += stepSize*fYdx2(xFirst + stepSize/2, yFirst + (stepSize*fYdx2(xFirst,yFirst))/2);
		*(midpointResult+i) = yFirst;
	}	
}

int main() {
   
    double x0, y0, xlast, stepSize, iter, *eulerResult, *midpointResult, *rk4Result;
   
    printf("Please enter x0, y0, xlast, stepSize : ");
    scanf("%lf %lf %lf %lf",&x0,&y0,&xlast,&stepSize);
    
    iter= (xlast-x0)/stepSize +1; 
    eulerResult = (double *)malloc(iter*sizeof(double));
	midpointResult = (double *)malloc(iter*sizeof(double));
	rk4Result = (double *)malloc(iter*sizeof(double));
	
	euler( fdy,  x0, y0,  xlast,  stepSize,  eulerResult);
	midpoint(fdy, x0, y0, xlast, stepSize, midpointResult);
	
	printf("\t \t \t Euler \t\t Midpoint    \t RK4 \n");
	for(int i=0; i<iter; i++){
		
		printf("Step %d y(%lf) \t %lf \t %lf \t %lf \n",i, x0+i*stepSize, *(eulerResult+i), *(midpointResult+i), fy(x0+i*stepSize) );
	}
	
	
	
	free(eulerResult);
	free(midpointResult);
	free(rk4Result);
					  
    return 0;
}
