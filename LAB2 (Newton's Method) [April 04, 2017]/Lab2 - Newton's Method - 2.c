#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double f(double x, double* coeffs, int degree) {
    double result = 0.0;
    for (int i = 0; i <= degree; i++) {
        result += coeffs[i] * pow(x, degree - i);
    }
    return result;
}

double df(double x, double* coeffs, int degree) {
    double result = 0.0;
    for (int i = 0; i < degree; i++) {
        result += (degree - i) * coeffs[i] * pow(x, degree - i - 1);
    }
    return result;
}

double newton(double x0, double dmin, double epsilon, double* coeffs, int degree) {
    double x1, fx, dfx;
    do {
        fx = f(x0, coeffs, degree);
        dfx = df(x0, coeffs, degree);
        x1 = x0 - fx / dfx;
        if (fabs(x1 - x0) < dmin) break;
        x0 = x1;
    } while (fabs(fx) > epsilon);
    return x1;
}

int main(int argc, char* argv[]) {
    double dmin = atof(argv[1]);
    double epsilon = atof(argv[2]);
    int degree = argc - 3;
    double* coeffs = (double*)malloc(sizeof(double) * (degree + 1));
    for (int i = 0; i <= degree; i++) {
        coeffs[i] = atof(argv[i + 3]);
    }
    for (double x0 = -10.0; x0 <= 10.0; x0 += 0.5) {
        double root = newton(x0, dmin, epsilon, coeffs, degree);
        printf("Initial guess: %f, root found: %f\n", x0, root);
    }
    free(coeffs);
    return 0;
}
