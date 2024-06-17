#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define e 2.7182818284 

typedef double (*Function)(double);

int *panelNum(int *ptr){
	
	for(int i=0;i<20;i++){
		*(ptr+i) = 50*(i+1);
	}
	return ptr;
}

double f1(double x){
	return pow(e,(-0.1)*x*x);
}

double f2(double x){
	return 2*pow(e,(-0.2)*x);
}

double rect(double start, double end, int n, Function func){
	
	double result, xn;

	result = 0;
	xn = (end-start)/n;
	
	for (int i=0;i<n;i++){
		result = result + func(start + i*xn);
	}
	
	return (xn*result);	
}

double trap(double start, double end, int n, Function func){
	
	double result, xn;

	result = func(start)/2 - func(end)/2;
	xn = (end-start)/n;
	
	for (int i=1;i<=n;i++){
		result = result + func(start + i*xn);
	}
	
	return (xn*result);
}

int main() {
   
    Function func ;
    int *ptr = (int*)malloc(20*sizeof(int));	
		
	ptr = panelNum(ptr);
	printf("\n\t\tINTEGRALS\n\n ");
	printf("\t\t\t   rect f1     trap f1     rect f2     trap f2     rect area   trap area ");	
		
	for(int i=0;i<20;i++){
		func = f1;
		double valrect1 = rect(0.0,30.0,*(ptr+i),func);
		double valtrap1 = trap(0.0,30.0,*(ptr+i),func);
		func = f2;
		double valrect2 = rect(0.0,30.0,*(ptr+i),func);	
		double valtrap2 = trap(0.0,30.0,*(ptr+i),func);
		
		printf("\nNumber of panels: %4d     %lf    %lf    %lf    %lf    %lf    %lf", *(ptr+i), valrect1 ,valtrap1 , valrect2, valtrap2, valrect2 - valrect1, valtrap2 - valtrap1); 	
	}
	
	printf("\n\nRef. value f1 = 2.8025  \nRef. value f2 = 9.97521  \nRef. value area = 7.17272  ");
	free(ptr);				  
    return 0;
}
