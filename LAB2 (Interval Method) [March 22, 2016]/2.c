#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
Define the function for which you want to find the roots. In this case, the function is pow(x,5) -11.08*pow(x,4) + 25.9605*pow(x,3) + 18.4986*pow(x,2) + 41.2429*x + 63.4687.

Define the find_interval function which takes the function f, the start and end of the interval xstart and xend, the step size stepsize, an output array output, and a root array root as parameters.

Inside find_interval, initialize count to -1, xleft to xstart, and xright to xleft + stepsize.

Start a while loop that continues as long as xleft is less than xend.

Inside the loop, if the function values at xleft and xright have opposite signs (i.e., their product is less than or equal to 0), increment count, store xleft and xright in the output array, and calculate the root using the bisect function, storing it in the root array.

Update xleft to xright and xright to xright + stepsize. If xright exceeds xend, set xright to xend.

After the loop, return count.

Define the bisect function which takes the function f, the start and end of the interval x1 and x2, and the tolerance epsilon as parameters.

Inside bisect, initialize y to the midpoint of x1 and x2.

Start a loop that continues as long as the absolute difference between x1 and y is greater than epsilon.

Inside the loop, if the function values at x1 and y have opposite signs, update x2 to y. Otherwise, update x1 to y. Update y to the new midpoint of x1 and x2.

After the loop, return y.

In the main function, call find_interval with the function, start and end of the interval, step size, the result array, and the root array as parameters.

If index (the return value of find_interval) is not -1, print all the intervals and roots found. Otherwise, print "Interval not found".
*/

double function(double);
int find_interval(double (*f)(double) , double, double ,double ,double [][2],double []);
double bisect(double (*f)(double), double, double, double );
double result[5][2];
double epsilon1 = 0.0001;
double root[10];

int main(){
		int index,i;
		double xstart = -10;
		double xend = 10;
		double stepsize = 0.01;
								//		## Call function
		index = find_interval(function,xstart,xend,stepsize,result,root);
								//		## write intervals if there is
		if(index != -1){
			for(i=0; i<=index; i++){
					printf("Interval%d --> [%lf, %lf]\n",i+1,result[i][0],result[i][1]);
			}
			for(i=0; i<=index; i++){
				printf("root [%d] :  %lf \n",i+1,root[i]);
			}

		}else{
			printf("Interval not found");
		}

		printf("%lf",function(6.0));
		return 0;
}
								//		## function to find root intervals
int find_interval(double (*f)(double), double xstart, double xend, double stepsize,double output[][2],double root[])
{
	int count = -1;
	double xleft;
	double xright;

	xleft = xstart;
	xright = xleft + stepsize;

	while(xleft < xend)
	{
		if( f(xleft) * f(xright) <= 0.0){
			count++;
			output[count][0] 	= xleft;
			output[count][1] 	= xright;
			root[count] = bisect(function, xleft, xright, epsilon1);
		}
		xleft = xright;
		xright = xright + stepsize;

		if(xright > xend){	xright = xend;	}
	}
	return count;
}
								//		## function that is given
double function(double x)
{
	return (pow(x,5) -11.08*pow(x,4) + 25.9605*pow(x,3) + 18.4986*pow(x,2) + 41.2429*x + 63.4687);
	//return ((x*x*x*x) + ( (7/2)*x*x*x) - (3*x*x) - ((7/2)*x) + 2 );
}
								//		## bisect function
double bisect(double (*f)(double), double x1, double x2, double epsilon){
	double y;

	for(y=(x1+x2)/2.0; fabs(x1-y) >epsilon; y=(x1+x2)/2.0){

		if(f(x1)*f(y) <= 0.0 ){
			x2 = y;
		}else{
			x1 = y;
		}
	}

	return (y);
}
