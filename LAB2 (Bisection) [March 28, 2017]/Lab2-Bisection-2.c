#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double bisection(double n, double start, double end, double tolerance, int max_iterations) {
    double mid, error, root;

    for (int i = 0; i < max_iterations; i++) {
        mid = (start + end) / 2;
        root = mid * mid;
        error = fabs(n - root);

        if (error < tolerance) {
            return mid;
        }

        if (root < n) {
            start = mid;
        } else {
            end = mid;
        }
    }

    return -1; // Return -1 if maximum iterations reached without meeting tolerance
}

int main(int argc, char *argv[]) {
    if (argc != 6) {
        printf("Usage: %s <n> <start> <end> <tolerance> <max_iterations>\n", argv[0]);
        return 1;
    }

    double n = atof(argv[1]);
    double start = atof(argv[2]);
    double end = atof(argv[3]);
    double tolerance = atof(argv[4]);
    int max_iterations = atoi(argv[5]);

    double root = bisection(n, start, end, tolerance, max_iterations);

    if (root == -1) {
        printf("Root not found within the specified iterations and tolerance.\n");
    } else {
        printf("Root found: %lf\n", root);
    }

    return 0;
}
