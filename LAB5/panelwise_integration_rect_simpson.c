#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int *panelNum(int *ptr){
	
	for(int i=0;i<10;i++){
		*(ptr+i) = 10*(i+1);
	}
	return ptr;
}

double func_power(double x){	return (50*exp((-1)*25*x));		}

double left_rect(double (*func)(double), double start, double end, int n){
	
	double result, xn;
	result = 0;
	xn = (end-start)/n;
	
	for (int i=0;i<n;i++){
		result = result + func(start + i*xn);
	}
	
	return (xn*result);	
}

double right_rect(double (*func)(double), double start, double end, int n){
	
	double result, xn;
	result = 0;
	xn = (end-start)/n;
	
	for (int i=1;i<=n;i++){
		result = result + func(start + i*xn);
	}
	
	return (xn*result);	
}

double simpson(double (*func)(double), double start, double end, int n){
    double h,area=0.0;
	h=(end-start)/n;
	for(;start<end;start=start+h){
		area=area+h*(func(start)+4*func(start+h/2)+func(start+h))/6;
	}
	return(area);
}

int main() {
   
    int *ptr = (int*)malloc(10*sizeof(int));	
	double a,b;
		
	ptr = panelNum(ptr);
	printf("\n\t\tINTEGRALS\n\n ");
	printf("\nEnter a: ");
	scanf("%lf",&a);
	printf("\nEnter b: ");
	scanf("%lf",&b);		
	printf("\t\t\t   rect f1 \t   trap f1 \t    simpson f1 ");	
	
	double valrect1;
	double valrect2;
	double valsimp;	
	for(int i=0;i<10;i++){
		
		valrect1 = left_rect(func_power,a,b,*(ptr+i));
		valrect2 = right_rect(func_power,a,b,*(ptr+i));
		valsimp = simpson(func_power,a,b,*(ptr+i));
		printf("\nNumber of panels: %4d     %.10lf    %.10lf    %.10lf", *(ptr+i), valrect1/1000 , (valrect1+valrect2)/2000, valsimp/1000 ); 	
	}
	
	free(ptr);				  
    return 0;
}
