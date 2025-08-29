#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_TERMS 10

double f(double x, double *coeff, int n) {
    double result = 0.0;
    for (int i = 0; i < n; i++) {
        result += coeff[i] * pow(x, n - i - 1);
    }
    return result;
}

double f_prime(double x, double *coeff, int n) {
    double result = 0.0;
    for (int i = 0; i < n - 1; i++) {
        result += (n - i - 1) * coeff[i] * pow(x, n - i - 2);
    }
    return result;
}

double newton_method(double x0, double dmin, double epsilon, double *coeff, int n) {
    double x = x0;
    double x_next;
    do {
        x_next = x - f(x, coeff, n) / f_prime(x, coeff, n);
        if (fabs(x_next - x) < epsilon) {
            break;
        }
        x = x_next;
    } while (fabs(f(x, coeff, n)) > dmin);
    return x;
}

int main(int argc, char *argv[]) {
    if (argc < 4) {
        printf("Usage: %s <dmin> <epsilon> <coeff1> <coeff2> ... <coeffN>\n", argv[0]);
        return 1;
    }

    double dmin = atof(argv[1]);
    double epsilon = atof(argv[2]);
    int n = argc - 3;
    double coeff[MAX_TERMS];

    for (int i = 0; i < n; i++) {
        coeff[i] = atof(argv[i + 3]);
    }

    for (double x0 = -10.0; x0 <= 10.0; x0 += 0.5) {
        double root = newton_method(x0, dmin, epsilon, coeff, n);
        if (fabs(f(root, coeff, n)) < dmin) {
            printf("Value of root is %f\n", root);
            break;
        }
    }

    return 0;
}
