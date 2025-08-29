#include <stdio.h>
#include <math.h>


/*
This C program finds the roots of a polynomial using the Interval method and the Bisection method.

Here's how it works:

1. The `polynomial` function calculates the value of a polynomial for a given `x` using the coefficients provided in the `coeffs` array.

2. The `intervalMethod` function finds a root of the polynomial within a given interval. It starts at the `start` value and increments by `step` until it reaches the `end` value. If it finds an interval where the polynomial changes sign, it assumes a root is present and returns the start of the interval.

3. The `bisectionMethod` function also finds a root of the polynomial within a given interval. It starts by finding the midpoint of the interval. If the polynomial at the midpoint is 0, or the size of the interval is less than a small tolerance (1e-6), it assumes a root is present and returns the midpoint. Otherwise, it updates the interval to be the half that contains the root and repeats the process.

4. The `main` function asks the user to input the order of the polynomial and its coefficients. It then asks for the start and end of the interval, the step size for the Interval method, and the maximum number of iterations for the Bisection method. It then calls the `intervalMethod` and `bisectionMethod` functions and prints their results.

Note: The program assumes that the polynomial has at least one real root within the given interval. If this is not the case, the Interval method will return `NAN` and the Bisection method will return an approximate root after reaching the maximum number of iterations.
*/

// Function to calculate the polynomial
double polynomial(double x, double coeffs[], int order) {
    double result = 0.0;
    for (int i = order; i >= 0; i--) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Interval method implementation
double intervalMethod(double start, double end, double step, double coeffs[], int order) {
    int iterations = 0;
    double current = start;

    while (current <= end) {
        double prevValue = polynomial(current, coeffs, order);
        double currentValue = polynomial(current + step, coeffs, order);

        if (prevValue * currentValue <= 0) { // Possible root found
            printf("Interval Method: Root found in interval: [%lf, %lf]\n", current, current + step);
            printf("Iterations: %d\n", iterations);
            return current;
        }

        printf("Iteration of interval method on %d: f(%.6lf)=%.6lf\n", iterations, current, prevValue);

        current += step;
        iterations++;
    }

    printf("Interval Method: No root found within the given interval.\n");
    return NAN; // Return NaN if no root found
}

// Bisection method implementation
double bisectionMethod(double start, double end, int maxIterations, double coeffs[], int order) {
    int iterations = 0;

    while (iterations < maxIterations) {
        double mid = (start + end) / 2;
        double midValue = polynomial(mid, coeffs, order);

        if (midValue == 0 || (end - start) / 2 < 1e-6) { // Tolerance check
            printf("Bisection Method: Root found at approximately: %lf\n", mid);
            printf("Iterations: %d\n", iterations);
            return mid;
        }

        printf("Iteration of bisection method on %d: f(%.6lf)=%.6lf\n", iterations, mid, midValue);

        if (polynomial(start, coeffs, order) * midValue < 0) {
            end = mid; // Root lies in the left half
        } else {
            start = mid; // Root lies in the right half
        }

        iterations++;
    }

    printf("Bisection Method: Maximum iterations reached. Approximate root: %lf\n", (start + end) / 2);
    return (start + end) / 2; // Return approximate root if max iterations reached
}

int main() {
    int order;
    printf("Enter the order of the polynomial: ");
    scanf("%d", &order);

    double coeffs[order + 1];
    printf("Enter %d coefficients: ", order + 1);
    for (int i = order; i >= 0; i--) {
        scanf("%lf", &coeffs[i]);
    }

     printf("Given polynomial is: ");
    for (int i = order; i >= 0; i--) {
        if (i != order) printf("%+g*x^%d", coeffs[i], i);
        else printf("%.2g*x^%d", coeffs[i], i);
    }
    printf(" = 0\n");

    double start, end, stepSize;
    int maxIterations;
    printf("Input interval values (x1,x2), step_size and maximum iterations: ");
    scanf("%lf %lf %lf %d", &start, &end, &stepSize, &maxIterations);

    double rootInterval = intervalMethod(start, end, stepSize, coeffs, order);
    printf("\n");
    double rootBisection = bisectionMethod(start, end, maxIterations, coeffs, order);
    printf("Root (Interval Method) = %lf\n", rootInterval);
    printf("Root (Bisection Method) = %lf\n", rootBisection);

    return 0;
}
