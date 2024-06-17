#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/*
1. Define the function for which you want to find the root intervals. In this case, the function is x*x - x - 20.

2. Define the find_interval function which takes the function f,
the start and end of the interval xstart and xend, the step size stepsize,
and an output array output as parameters.

3. Initialize count to -1. This will be used to keep track of the number of intervals found.

4. Initialize xleft to xstart and xright to xleft + stepsize.

5. Start a while loop that continues as long as xleft is less than xend.

6. Inside the loop, check if the function values at xleft and xright have opposite signs
(i.e., their product is less than or equal to 0). If they do, increment count and store xleft and xright in the output array.

7. Update xleft to xright and xright to xright + stepsize.

8. If xright exceeds xend, set xright to xend.

9. After the loop, return count. This is the index of the last interval found.

10. In the main function, call find_interval with the function, start and end of the interval, step size, and the result array as parameters.

11. If index (the return value of find_interval) is not -1, print all the intervals found. Otherwise, print "Interval not found".

*/


double function(double);
int find_interval(double (*f)(double) , double, double ,double ,double [][2]);
double result[5][2];

int main(){
		int index,i;
		double xstart = -10;
		double xend = 10;
		double stepsize = 0.002;
								//		## Call function
		index = find_interval(function,xstart,xend,stepsize,result);
								//		## write intervals if there is
		if(index != -1){
			for(i=0; i<=index; i++){
					printf("Interval%d --> [%lf, %lf]\n",i+1,result[i][0],result[i][1]);
			}
		}else{
			printf("Interval not found");
		}
		return 0;
}
								//		## function to find root intervals
int find_interval(double (*f)(double), double xstart, double xend, double stepsize,double output[][2])
{
	int count = -1;
	double xleft;
	double xright;

	xleft = xstart;
	xright = xleft + stepsize;

	while(xleft < xend)
	{
		if( f(xleft) * f(xright) <= 0){
			count++;
			output[count][0] 	= xleft;
			output[count][1] 	= xright;
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
	//return (pow(x,5) -11.08*pow(x,4) + 25.9605*pow(x,3) + 18.4986*pow(x,2) + 41.2429*x + 63.4687);
	//return ((x*x*x*x) + ( (7/2)*x*x*x) - (3*x*x) - ((7/2)*x) + 2 );
	return(x*x-x-20);
}

double bisect(){

}
