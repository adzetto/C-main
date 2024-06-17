#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double n;

double f(double x) {
    return x*x - n;
}

// Function: bisection
// The bisection method is a root-finding method that applies to any continuous function for which one knows two values with opposite signs.
// The method consists of repeatedly bisecting the interval defined by these values and then selecting the subinterval in which the function changes sign,
// and therefore must contain a root.

double bisection(double start, double end, double tolerance, int max_iter) {
    // Declare a variable to hold the midpoint of the interval
    double midpoint;

    // Start a loop that will run for a maximum of max_iter iterations
    for (int i = 0; i < max_iter; i++) {
        // Calculate the midpoint of the interval
        midpoint = (start + end) / 2;

        // If the absolute value of f(midpoint) is less than the tolerance,
        // it means we have found a root, so we return the midpoint
        if (fabs(f(midpoint)) < tolerance) {
            return midpoint;
        }

        // If f(start) and f(midpoint) have opposite signs, it means the root
        // is in the left subinterval, so we update end to be the midpoint
        if (f(start) * f(midpoint) < 0) {
            end = midpoint;
        }
        // If f(start) and f(midpoint) have the same sign, it means the root
        // is in the right subinterval, so we update start to be the midpoint
        else {
            start = midpoint;
        }
    }

    // If we have reached the maximum number of iterations without finding a root,
    // we return the last calculated midpoint
    return midpoint;
}


int main(int argc, char *argv[]) {
    // atof() is a function that converts a string to a double. Here, it is used to convert the command line arguments to the appropriate types.
    // argv[1] is the second command line argument (the first argument, argv[0], is the name of the program itself), which represents the value of n in the function f(x) = x^2 - n.
    n = atof(argv[1]);

    // argv[2] is the start of the interval to search for the root.
    double start = atof(argv[2]);

    // argv[3] is the end of the interval to search for the root.
    double end = atof(argv[3]);

    // argv[4] is the convergence tolerance between the input value and the square value of the found roots.
    double tolerance = atof(argv[4]);

    // argv[5] is the maximum number of iterations to perform in the search.
    // atoi() is a function that converts a string to an integer.
    int max_iter = atoi(argv[5]);

    // Call the bisection function with the parsed arguments to find the root.
    double root = bisection(start, end, tolerance, max_iter);

    // Print the found roots. Since the function is f(x) = x^2 - n, both root and -root are valid roots.
    printf("Found roots are %f and %f\n", root, -root);

    // Return 0 to indicate that the program has finished successfully.
    return 0;
}
