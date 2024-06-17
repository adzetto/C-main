#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define pi 3.142857 
#define start 0
#define end 10

double phit(double t){
	double result;
	result=sin(4*pi*t);
    return(result);
}
double phitdiff(double t,double h){
	double result;
	result=(phit(t-h)-phit(t))/h;
return(result);
}
double phidiff(double t){
	double result;
	result=-4*pi*cos(4*pi*t);
return(result);
}
double xt(double t,double h){
	double result;
	result=0.5*cos(20*pi*t+phitdiff(t,h))+1;
return(result);
}
double simpson(double (*f)(double,double),int n){
    double h,area=0.0,x,xlast;
    x=start;
    xlast=end;
	h=(xlast-x)/n;
	for(;x<end;x=x+h){
		area=area+h*(f(x,h)+4*f(x+h/2,h)+f(x+h,h))/6;
	}
	return(area);
}
float adaptivesipmson(float(*f)(float,float),int n,float tolerance){
	float check=tolerance+1.0;
    float lowval,val;
	
	lowval=simpson(xt,n);
//	printf("%f ",lowval);
	while(check>tolerance){
		n=n*2;
		val=simpson(xt,n);
//		printf("%f ",val);
		check=fabs((val-lowval)/val);
		lowval=val;
	}
	return(val);
}
int main() {
	
//printf("%f",phit(5));
float a,b;	
//a=simpson(xt,160);
//b=adaptivesipmson(xt,10,pow(10,-4));
//printf("%f",a);

int i,panel;

for(i=1;i<21;i++){
	panel=i*10;
	printf("panel :%d simpson:   %lf adaptive simpson:    %lf\n",panel,simpson(xt,panel),adaptivesipmson(xt,panel,pow(10,-4)));
}
		
}
