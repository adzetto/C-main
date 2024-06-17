#include <stdlib.h>
#include <math.h>

double f1(double x) {
    return exp(-0.1 * pow(x, 2));
}

double f2(double x) {
    return 2 * exp(-0.2 * x);
}

double rectangular_integral(double (*f)(double), double a, double b, int n) {
    double h = (b - a) / n;                        // Determine the breadth of every rectangle
    double sum = 0.0;                              // Start with a zero sum
    for (int i = 0; i < n; i++) {
        sum += f(a + i * h);                       // Take the left-hand sum of all the subinterval function values and add them up
    }
    return h * sum;                                // The integral estimate
}

double trapezoidal_integral(double (*f)(double), double a, double b, int n) {
    double h = (b - a) / n;                        // Calculate the width of each trapezoid
    double sum = 0.5 * (f(a) + f(b));              // Start with the average of the first and last function values
    for (int i = 1; i < n; i++) {
        sum += f(a + i * h);                       // Add the function values at each subinterval's midpoint
    }
    return h * sum;                                // Multiply the accumulated sum by the width to get the integral estimate
}

int *number_of_panels_generator() {
    int *panels = malloc(20 * sizeof(int));       // Allocate memory for an array of 20 integers
    for (int i = 0; i < 20; i++) {
        panels[i] = 50 * (i + 1);                 // Populate the array with multiples of 50
    }
    return panels;
}

int main() {
    int *panels = number_of_panels_generator();

    printf("%-10s | %-12s | %-12s | %-12s | %-12s | %-12s | %-12s\n", "Panels", "Rect. f1", "Trap. f1", "Rect. f2", "Trap. f2", "Rect. Area", "Trap. Area");
    printf("---------------------------------------------------------------------------------------------------\n");

    for (int i = 0; i < 20; i++) {
        double rect_f1 = rectangular_integral(f1, 0, 30, panels[i]);
        double trap_f1 = trapezoidal_integral(f1, 0, 30, panels[i]);
        double rect_f2 = rectangular_integral(f2, 0, 30, panels[i]);
        double trap_f2 = trapezoidal_integral(f2, 0, 30, panels[i]);

        double rect_area = rect_f2 - rect_f1;
        double trap_area = trap_f2 - trap_f1;

        printf("%-10d | %-12.6f | %-12.6f | %-12.6f | %-12.6f | %-12.6f | %-12.6f\n",
               panels[i], rect_f1, trap_f1, rect_f2, trap_f2, rect_area, trap_area);
    }
    free(panels);
    return 0;
}
