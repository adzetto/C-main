#include <stdio.h>
#include <math.h>
#define IS 1e-12
#define N 1
#define VT 0.026
#define start -1
#define end 1
#define tol 0.00001

double funcVolt(double I,double VD)
{
	double result, Val;
	Val=VD/(N*VT);
	result= -I + IS*(exp(Val) -1);
	
	return result;	
}

double bisect(double current,double epsilon,double min,double max,int *it){

	double y,count=0;
	y=(min+max)/2.0;

for(y=(min+max)/2.0; fabs(min-y)>epsilon; y=(min+max)/2.0){
	count++;
	if(funcVolt(current,min)*funcVolt(current,y)<=0.0){
	
		max=y;
	}
	else
	min=y; 
}
	y=(min+max)/2.0;
	(*it)= ++count;
	return(y);
}


int main() {
   
    double I,V;
	int iter=0;
	//bu aþamadan sonra bekliyorum
	// bir dosyayý brave tarayýcýsýnda açacaðým
		printf("\n\t\tBISECTION METHOT \n");
	    V = funcVolt(0, 0.6);
	    printf("\nTest current for VD=0.6 is : %lf",V);
		
		printf(" \n\nPlease enter diode current : ");
		scanf("%lf",&I);
		//printf("\n\nFor Current = %lf ,Accurate Root calculated is = %lf \n ",I,bisect(I,tol,start,end,&iter));
		//printf("Total iteration : %d",iter); 	
	
		  
    return 0;
}
