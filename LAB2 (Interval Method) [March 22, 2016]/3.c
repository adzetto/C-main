#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>


/*
1. Initialize global variables:
    - `result` as a 5x2 array of doubles
    - `epsilon1` as 0.0001
    - `root` as an array of 10 doubles

2. Define `main` function:
    1. Initialize local variables:
        - `index` as an integer
        - `i` as an integer
        - `xstart` as -6
        - `xend` as 6
        - `stepsize` as 0.03
    2. Call `find_interval` function with `function`, `xstart`, `xend`, `stepsize`, `result`, `root`, and `index` as arguments, and store the result in `found`
    3. If `found` is true:
        1. Loop from `i=0` to `index` (inclusive):
            1. Print the interval number and its corresponding range from `result`
        2. Loop from `i=0` to `index` (inclusive):
            1. Print the root number and its corresponding value from `root`
    4. Else, print "Interval not found"
    5. Return 0

3. Define `find_interval` function with parameters `f` (a function), `xstart`, `xend`, `stepsize`, `output` (a 2D array), `root` (an array), and `count` (a pointer to an integer):
    1. Set `*count` to -1
    2. Initialize `xleft` as `xstart` and `xright` as `xleft + stepsize`
    3. While `xleft` is less than `xend`:
        1. If `f(xleft) * f(xright)` is less than or equal to 0:
            1. Increment `*count`
            2. Set `output[*count][0]` to `xleft` and `output[*count][1]` to `xright`
            3. Set `root[*count]` to the result of `bisect` function with `function`, `xleft`, `xright`, and `epsilon1` as arguments
        2. Set `xleft` to `xright` and `xright` to `xright + stepsize`
        3. If `xright` is greater than `xend`, set `xright` to `xend`
    4. Return whether `*count` is not equal to -1

4. Define `function` function with `x` as a parameter:
    1. Return `x^4 + (7/2)*x^3 - 3*x^2 - (7/2)*x + 2`

5. Define `bisect` function with parameters `f` (a function), `x1`, `x2`, and `epsilon`:
    1. Initialize `y` as `(x1+x2)/2`
    2. While the absolute difference between `x1` and `y` is greater than `epsilon`:
        1. If `f(x1)*f(y)` is less than or equal to 0, set `x2` to `y`
        2. Else, set `x1` to `y`
        3. Update `y` as `(x1+x2)/2`
    3. Return `y`
*/

double function(double);
bool find_interval(double (*f)(double), double, double, double, double[][2], double[], int*);
double bisect(double (*f)(double), double, double, double);
double result[5][2];
double epsilon1 = 0.0001;
double root[10];

int main(){
    int index, i;
    double xstart = -6;
    double xend = 6;
    double stepsize = 0.03;

    // Call function
    bool found = find_interval(function, xstart, xend, stepsize, result, root, &index);

    // Write intervals if there is
    if(found){
        for(i=0; i<=index; i++){
            printf("Interval%d --> [%lf, %lf]\n", i+1, result[i][0], result[i][1]);
        }
        for(i=0; i<=index; i++){
            printf("root [%d] :  %lf \n", i+1, root[i]);
        }
    } else {
        printf("Interval not found");
    }

    return 0;
}

// Function to find root intervals
bool find_interval(double (*f)(double), double xstart, double xend, double stepsize, double output[][2], double root[], int *count){
    *count = -1;
    double xleft;
    double xright;

    xleft = xstart;
    xright = xleft + stepsize;

    while(xleft < xend){
        if(f(xleft) * f(xright) <= 0.0){
            (*count)++;
            output[*count][0] = xleft;
            output[*count][1] = xright;
            root[*count] = bisect(function, xleft, xright, epsilon1);
        }
        xleft = xright;
        xright = xright + stepsize;

        if(xright > xend){
            xright = xend;
        }
    }
    return *count != -1;
}

// Function that is given
double function(double x){
    return pow(x, 4) + (7.0/2.0)*pow(x, 3) - 3*pow(x, 2) - (7.0/2.0)*x + 2;
}

// Bisect function
double bisect(double (*f)(double), double x1, double x2, double epsilon){
    double y;

    for(y=(x1+x2)/2.0; fabs(x1-y) > epsilon; y=(x1+x2)/2.0){
        if(f(x1)*f(y) <= 0.0){
            x2 = y;
        } else {
            x1 = y;
        }
    }

    return y;
}
