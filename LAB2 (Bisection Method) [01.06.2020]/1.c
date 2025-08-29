#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#define ln(x) log(x)

double func(double R)
{
	float T;
	T= (1.1292/1000)+(2.3410*ln(R)/10000)+8.7754*pow(ln(R),3)/100000000;
	T=1/T;
	T=T-273.15;
	return T;
}
double func2(double t2,double R1)
{
	float result;
	t2=t2+273.15;
	result= (1.1292/1000)+(2.3410*ln(R1)/10000)+8.7754*pow(ln(R1),3)/100000000;
	result=result*t2;
	result=result-1;
	return result;
	
}

double bisect(double temp,double epsilon,double resistor1,double resistor2){

	int sayac=0;
double y;
for(y=(resistor1+resistor2)/2.0; fabs(resistor1-y)>epsilon; y=(resistor1+resistor2)/2.0){
sayac++;
if(func2(temp,resistor1)*func2(temp,y)<=0.0){

	resistor2=y;
}
else
resistor1=y; 
}
printf("With %d iteration,",sayac);
return(y);


}

int main ()
{	
	float temp[4];
	printf("Enter the Temperature values in Celcius\n");
	int i;
	for(i=0; i<4; i++){
		scanf("%f",&temp[i]);
	}
	float r1,r2;
		float startpoint,endpoint,tolerance;
	
	r1=5000;
	r2=25000;
	double con_tol;
	printf("For R_test_1: Temperature is calculated %f\n",func(r1));
	printf("For R_test_2: Temperature is calculated %f\n",func(r2));
	printf("Enter the start point: ");
	scanf("%f",&startpoint);
	printf("Enter end point: ");
	scanf("%f",&endpoint);
	printf("Enter temperature tolerance: ");
	scanf("%f",&tolerance);
	int sayac;
	double root;
		for(i=0; i<4; i++){
		printf("For Temperature=%f , Accurate Root calculated is %f\n",temp[i],bisect(temp[i],tolerance,startpoint,endpoint)); 
	}

	return 0;
}

