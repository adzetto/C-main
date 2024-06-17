#include <stdio.h>
#include <math.h>

// Constants and global variables
double d = 0.5, L = 1.4, lambda_g = 3.6, a = 2.28; // Physical parameters of the system

// Function prototypes
double f(double x);                                             // Prototype for function f, calculates the functional value
double f_prime(double x);                                       // Prototype for derivative of function f
double K();                                                     // Prototype for calculating the wave number K
double e_r(double x);                                           // Prototype to calculate the dielectric constant
double to_radians(double degrees);                              // Converts degrees to radians
double* tolerance_value_generator();                            // Generates a series of tolerance values
double newtons_method(double x0, double tol, int* iterations);  // Newton's method for root finding

int main() {
    double x0_degrees;
    printf("Initial Point in Degrees (Between 90 and 270): ");
    scanf("%lf", &x0_degrees);

    while (x0_degrees <= 90 || x0_degrees >= 270) {
        printf("Initial Point in Degrees (Between 90 and 270): ");
        scanf("%lf", &x0_degrees);
    }

    double x0 = to_radians(x0_degrees);  // Convert initial guess to radians
    double* tolerances = tolerance_value_generator();  // Retrieve array of tolerance values

    for (int i = 0; i < 6; i++) {
        int iterations = 0;
        double root = newtons_method(x0, tolerances[i], &iterations);

        printf("Tolerance: %lf\n", tolerances[i]);
        printf("Root of f(x) where x is in radians: %lf\n", root);
        printf("Dielectric constant (e_r): %lf\n", e_r(root));
        printf("Number of iterations: %d\n", iterations);
    }
    return 0;
}

double f(double x) {
    return tan(x) - K() * x;
}

double f_prime(double x) {
    return pow(1/cos(x), 2) - K();
}

double K() {
    return tan((2 * M_PI * (d + L)) / lambda_g) / (2 * M_PI * (d / lambda_g));
}

double e_r(double x) {
    return ((pow(a/d, 2) * pow(x/M_PI, 2)) + 1) / (pow(2*a/lambda_g, 2) + 1);
}

double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double *tolerance_value_generator() {
    static double tolerances[6];
    for (int i = 0; i < 6; i++) {
        tolerances[i] = pow(10, -6 + i);
    }
    return tolerances;
}

double newtons_method(double x0, double tol, int* iterations) {
    double x1;
    do {
        x1 = x0 - f(x0) / f_prime(x0);
        (*iterations)++;

        if (fabs(x1 - x0) <= tol) {
            break;
        }
        x0 = x1;
    } while (1);
    return x1;
}
