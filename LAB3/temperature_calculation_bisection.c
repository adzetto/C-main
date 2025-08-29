#include <stdio.h>
#include <math.h>
#define MAXSIZE 4

double temFun(double R){
	
	double Tem,TK,TC,Ln;
	Ln = log(R);
	Tem= (1.1292/1000)+(2.3410*Ln/10000)+8.7754*pow(Ln,3)/100000000;
	TK=1/Tem;
	TC=TK-273.15;
	return TC;
}

double func2(double t2,double R1)
{
	float result;
	t2=t2+273.15;
	result= (1.1292/1000)+(2.3410*log(R1)/10000)+8.7754*pow(log(R1),3)/100000000;
	result=result*t2-1;
	return result;
	
}

double bisect(double temp,double epsilon,double resistor1,double resistor2,int *it){

double y,count=0;

for(y=(resistor1+resistor2)/2.0; fabs(resistor1-y)>epsilon; y=(resistor1+resistor2)/2.0){
	count++;
	if(func2(temp,resistor1)*func2(temp,y)<=0.0){
	
		resistor2=y;
	}
	else
	resistor1=y; 
}
	(*it)=count;
	return(y);
}

int main() {
   
    double tem[MAXSIZE], x1, x2, tol,r1,r2;
	int iter=0;
	
    printf("Enter 4 tempreture : ");
    for(int i=0;i<MAXSIZE;i++){
    	scanf("%lf",&tem[i]);
	}
    
    r1 = temFun(5000);
    printf("\nTest result for R1=5000 is : %lf",r1);
    r2 = temFun(25000);
    printf("\nTest result for R1=25000 is : %lf",r2);
    
	printf(" \n\nPlease enter start value , end value and tolerance : ");
	scanf("%lf %lf %lf",&x1,&x2,&tol);
	
	for(int i=0;i<MAXSIZE;i++){
		printf("\n\nFor temperature= %lf ,Accurate Root calculated is = %lf with ",tem[i],bisect(tem[i],tol,x1,x2,&iter));
		printf("%d iteration",iter);
	}  
	  
    return 0;
}
