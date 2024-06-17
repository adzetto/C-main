/**
*       Author      :   Erman OLCAY
*       Date        :   26.03.2016
*       Discription :   Homework 4
*       Parameters  :   Finds root of function " x*x -n "
*                       argument values are n,start,stop,tolerance,iteration
*                       execute program from command line _example.\>bisect 9 0 5 0.1 15
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
double f(double x);
double bisect(double (*f)(double),double xstart,double xend,double epsilon,int n,int iteration)
{
	double midval;
	int it;
    it=iteration;
    midval=(xstart+xend)/2;
        if(fabs(n-f(midval))>epsilon && it!=0)
            if( (f(xstart)-n)*(f(midval)-n) <=0 ){
                midval=bisect(f,xstart,midval,epsilon,n,it-1);
            }
            else{
                midval=bisect(f,midval,xend,epsilon,n,it-1);
            }
return midval;
}

int main(int argc, char **argv)
{
	int iteration,n,i;
	double x1,x2,epsilon;
	double root;

	n=atof(argv[1]);
	x1=atof(argv[2]);
	x2=atof(argv[3]);
	epsilon=atof(argv[4]);
	iteration=atof(argv[5]);

	root=bisect(f,x1,x2,epsilon,n,iteration);
    printf("\nFound roots are %lf and %lf\n",root,root*(-1));
}

double f(double x)
{
	return ( (x*x) );
}


