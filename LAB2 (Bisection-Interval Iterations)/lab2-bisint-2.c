#include <stdio.h>
#include <math.h>

// Function to calculate the polynomial
double polynomial(double x, double coeffs[], int order) {
    double result = 0.0;
    int i;
    for (i = order; i >= 0; i--) {
        result += coeffs[order-i] * pow(x, i);
    }
    return result;
}

// Interval method implementation
void intervalMethod(double start, double end, double step, double coeffs[], int order) {
    int iterations = 0;
    double current = start;

    while (current <= end) {
        double prevValue = polynomial(current, coeffs, order);
        double currentValue = polynomial(current + step, coeffs, order);

        if (prevValue * currentValue <= 0) { // Possible root found
            printf("\nInterval Method: Root found in interval : [%lf, %lf]\n", current, current + step);
            printf("Iteration of interval method = %d\n", iterations);
            return; 
        }

        printf("f(%lf)=%lf, f(%lf)=%lf, f(%lf)=%lf\n", 
                current, prevValue, current + step, currentValue, current + step/2.0, polynomial(current + step/2.0, coeffs, order));

        current += step;
        iterations++;
    }

    printf("Interval Method: No root found within the given interval.\n");
}

// Bisection method implementation
void bisectionMethod(double start, double end, int maxIterations, double coeffs[], int order) {
    int iterations = 0;

    while (iterations <= maxIterations) {
        double mid = (start + end) / 2.0;
        double midValue = polynomial(mid, coeffs, order);

        if (midValue == 0 || ((end - start) / 2.0) < pow(10,-3)) { // Tolerance check
            printf("Bisection Method: Root found at approximately: %lf\n", mid);
            printf("Iterations: %d\n", iterations);
            return;
        }

        //printf("Iteration of bisection method = %d\n", iterations);

        if (polynomial(start, coeffs, order) * midValue < 0) {
            end = mid; // Root lies in the left half
        } else {
            start = mid; // Root lies in the right half
        }

        iterations++;
    }

    printf("Bisection Method: Maximum iterations reached. Approximate root: %lf\n", (start + end) / 2.0);
}

int main() {
    int order;
    printf("Enter the order of the polynomial: ");
    scanf("%d", &order);

    double coeffs[order + 1];
    printf("Enter %d coefficients: ", order + 1);
    int i;
    for (i = 0; i <= order; i++) {
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
    printf("Input interval values (x1,x2), step_size and maximum iterations:\n ");
    scanf("%lf %lf %lf %d", &start, &end, &stepSize, &maxIterations);

    intervalMethod(start, end, stepSize, coeffs, order);
    printf("\n"); 
    bisectionMethod(start, end, maxIterations, coeffs, order);

    return 0;
}