#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Constants
double d_val = 0.5, L_val = 1.4, lg_val = 3.6, a_val = 2.28, pi_val = M_PI;

// Function declarations
double calc_func(double x);
double calc_derivative(double x);
double calc_constant();
double calc_er(double x);
void convert_deg_to_rad(double *x);
double *generate_tol_values();
double *apply_newtons_method(double initial_guess, double tolerance);

int main() {
    
    double guess_in_degrees;
    double *tol_values, *calc_result;

    // Get initial guess in degrees from the user
    do {
        printf("Enter the initial guess (in degrees, between 90 and 270): ");
        scanf("%lf", &guess_in_degrees);
    } while (guess_in_degrees <= 90 || guess_in_degrees >= 270);

    // Convert initial guess to radians
    convert_deg_to_rad(&guess_in_degrees);

    // Generate tolerance values 
    tol_values = generate_tol_values(); 

    // Perform Newton's method for each tolerance value
    for (int i = 0; i < 6; i++) {
        calc_result = apply_newtons_method(guess_in_degrees, tol_values[i]);
        printf("Root (radians): %.8f, e_r: %.8f, Iterations: %f, Tolerance: %.1e\n", 
                calc_result[0], calc_er(calc_result[0]), calc_result[1], tol_values[i]);
        free(calc_result); // Free the memory allocated by apply_newtons_method
    }

    free(tol_values); // Free the memory allocated by generate_tol_values
    return 0;
}

// Function definitions

double calc_func(double x) {
    return tan(x) - calc_constant() * x;
}

double calc_derivative(double x) {
    return pow(1/cos(x), 2) - calc_constant();
}

double calc_constant() {
    return tan(2 * pi_val * (d_val + L_val) / lg_val) / (2 * pi_val * (d_val / lg_val));
}

double calc_er(double x) {
    return (pow(a_val / d_val, 2) * pow(x / pi_val, 2) + 1) / (pow(2 * a_val / lg_val, 2) + 1);
}

void convert_deg_to_rad(double *x) {
    *x = *x * pi_val / 180.0;
}

double *generate_tol_values() {
    double *tolerances = malloc(6 * sizeof(double));
    double tol = 1e-6;
    for (int i = 0; i < 6; i++) {
        tolerances[i] = tol;
        tol *= 10; 
    }
    return tolerances;
}

double *apply_newtons_method(double initial_guess, double tolerance) {
    double *result = malloc(2 * sizeof(double));
    double x = initial_guess;
    int iterations = 0;

    do {
        double fx = calc_func(x);
        double fpx = calc_derivative(x);
        x = x - fx / fpx; 
        iterations++;
    } while (fabs(calc_func(x)) > tolerance);

    result[0] = x;
    result[1] = iterations; 
    return result;
}