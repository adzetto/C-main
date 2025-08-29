#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define d 0.5
#define L 1.4
#define g 3.6
#define a 2.28
#define PI 3.14159265359 

void tolVal(double *ptr){
	
	for(int i=0;i<6;i++){
		*(ptr+i) = 0.1/pow(10,i);
	}
}

double DtoR(double x)
{
	return  (x*PI)/180;	
}

double Kval()
{
	double K = tan((2.0*PI*(d+L))/g)/(2.0*PI*(d/g));
	return K;	
}

double fun(double x)
{
	return  tan(x) - Kval()*x;	
}

double funprime(double x)
{
	return (1/cos(x))*(1/cos(x)) - Kval();	
}

double er(double x){

	return (pow((a/d),2)*pow((x/PI),2) + 1)/(pow(((2*a)/g),2) + 1);	
}

double newton(double (*fun)(double),double (*funprime)(double),double dmin,double x,double tolval, int *error, int *it){

	double deltax;
	deltax= 2.0*tolval;
	*error= 0;
	
	while( !(*error) && fabs(deltax) > tolval){
		
		if(fabs(funprime(x)) > dmin){
			deltax = fun(x)/funprime(x);
			x = x - deltax;
			(*it)++;
		}
		else
			*error = 1;
	}

	return(x);
}



int main() {
   
    double *ptr, dmin, tol, x=0;
	int error = 0,iter=0;
	ptr=&tol;
//	while(1){	
		while(!(x<270 && x>90)){
			
			printf("Please enter initial value between 90 and 270 : ");
			scanf("%lf",&x);
		}
		
		printf("\n\t\tNEWTON  METHOT ");
		x = DtoR(x);
		ptr = (double*)malloc(6*sizeof(double));
		tolVal(ptr);
			
		for(int i=5;i>=0;i--){
			
			if (error)
				printf("Error found in Newton’s method\n");
			else{
				iter=0; 
				double root = newton(fun, funprime, 10e-30, x, *(ptr+i), &error, &iter);
				printf("\n\nTolerance : %lf", *(ptr + i));	
				printf("\nFor initial value = %lf ,Accurate Root calculated is = %lf",x, root);
				printf("\nDielectric constant is : %lf", er(root));
				printf("\nTotal iteration : %d",iter); 	
			}
		}
//	}
	
	free(ptr);	  
    return 0;
}
