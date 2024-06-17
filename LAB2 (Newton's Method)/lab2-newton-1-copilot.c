#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Global variables
double d = 0.5, L = 1.4, lg = 3.6, a = 2.28, pi = M_PI;

// Function declarations
double g (double x);
double g_prime (double x);
double constant ();
double e_r_value (double x);
void convert_to_radian (double *x);
double *generate_tolerance ();
double *newtonsMethod (double initial_guess, double tolerance);

int
main ()
{

  double initial_guess_in_degree;
  double *tolerance_vals, *result;

  // Get initial guess in degrees from the user
  do
	{
	  printf ("Enter the initial guess (in degrees, between 90 and 270): ");
	  scanf ("%lf", &initial_guess_in_degree);
	}
  while (initial_guess_in_degree <= 90 || initial_guess_in_degree >= 270);

  // Convert initial guess to radians
  convert_to_radian (&initial_guess_in_degree);

  // Generate tolerance values 
  tolerance_vals = generate_tolerance ();

  // Perform Newton's method for each tolerance value
  for (int i = 0; i < 6; i++)
	{
	  result = newtonsMethod (initial_guess_in_degree, tolerance_vals[i]);
	  printf
		("Root (radians): %.8f, e_r: %.8f, Iterations: %f, Tolerance: %.1e\n",
		 result[0], e_r_value (result[0]), result[1], tolerance_vals[i]);
	  free (result);			// Free the memory allocated by newtonsMethod
	}

  free (tolerance_vals);		// Free the memory allocated by generate_tolerance
  return 0;
}

// Function definitions

double
g (double x)
{
  return tan (x) - constant () * x;
}

double
g_prime (double x)
{
  return pow (1 / cos (x), 2) - constant ();
}

double
constant ()
{
  return tan (2 * pi * (d + L) / lg) / (2 * pi * (d / lg));
}

double
e_r_value (double x)
{
  return (pow (a / d, 2) * pow (x / pi, 2) + 1) / (pow (2 * a / lg, 2) + 1);
}

void
convert_to_radian (double *x)
{
  *x = *x * pi / 180.0;
}

double *
generate_tolerance ()
{
  double *tolerances = malloc (6 * sizeof (double));
  double tol = 1e-6;
  for (int i = 0; i < 6; i++)
	{
	  tolerances[i] = tol;
	  tol *= 10;
	}
  return tolerances;
}

double *
newtonsMethod (double initial_guess, double tolerance)
{
  double *result = malloc (2 * sizeof (double));
  double x = initial_guess;
  int iterations = 0;

  do
	{
	  double gx = g (x);
	  double gpx = g_prime (x);
	  x = x - gx / gpx;
	  iterations++;
	}
  while (fabs (g (x)) > tolerance);

  result[0] = x;
  result[1] = iterations;
  return result;
}
