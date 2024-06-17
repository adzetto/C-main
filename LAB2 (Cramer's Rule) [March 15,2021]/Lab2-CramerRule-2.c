#include <stdio.h>
#include <math.h>
#define N 3 // Matrix dimension (3x3)

// Function prototype for getMatrix
void getMatrix(double mat[N][N]);      // Get matrix elements from user
double determinant(double mat[N][N], int n); // Calculate matrix determinant
void cofactor(double num[N][N], int f); // Calculate cofactor matrix
void transpose(double num[N][N], double fac[N][N], float r); // Transpose and find inverse
void display(double mat[N][N]);       // Display a matrix

// This function is expected to fill the mat array with some values
void getMatrix(double mat[N][N]) {
  int i, j;
  printf("Enter elements of the matrix:\n");
  for (i = 0; i < N; i++) {
    for (j = 0; j < N; j++) {
      while (scanf("%lf", &mat[i][j]) != 1) {
          printf("Invalid input. Please enter a number: ");
          scanf("%*[^\n]"); // Clear invalid input
      }
    }
  }
}


// Function to calculate the determinant of a matrix
double determinant(double mat[N][N], int n) {
    double det; // Variable to hold the determinant

    // If the matrix is 2x2
    if (n == 2) {
        // Calculate the determinant of a 2x2 matrix
        // The determinant of a 2x2 matrix [[a, b], [c, d]] is ad - bc
        det = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];
    }
    // If the matrix is 3x3
    else {
        // Calculate the determinant of a 3x3 matrix
        // The determinant of a 3x3 matrix is calculated using a rule called Sarrus' rule
        det = mat[0][0]*(mat[1][1]*mat[2][2] - mat[2][1]*mat[1][2]) - mat[0][1]*(mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0]) + mat[0][2]*(mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0]);
    }

    // Return the determinant
    return det;
}

// Function to create an identity matrix
void Identity(double I[N][N])
{
  // Loop over the rows
  for (int i=0;i<N;i++){
    // Loop over the columns
    for(int j=0;j<N;j++){
      // If row index equals column index, set the element to 1.0
      if(i==j)
        I[i][j] = 1.0;
      // Otherwise, set the element to 0.0
      else
        I[i][j] = 0.0;
    }
  }
}

// Function to display a matrix
void display(double mat[N][N]) {
    // Loop over the rows
    for (int i = 0; i < N; i++) {
        // Loop over the columns
        for (int j = 0; j < N; j++)
            // Print each element
            printf("%lf ", mat[i][j]);
        // Print a newline at the end of each row
        printf("\n");
    }
}

// Function to calculate the cofactor matrix
void cofactor(double num[N][N], int f)
{
    double b[N][N], fac[N][N]; // Temporary matrices
    int p, q, m, n, i, j; // Indices for loops
    // Loop over the rows
    for (q = 0;q < f; q++)
    {
        // Loop over the columns
        for (p = 0;p < f; p++)
        {
            m = 0;
            n = 0;
            // Loop over the rows
            for (i = 0;i < f; i++)
            {
                // Loop over the columns
                for (j = 0;j < f; j++)
                {
                    // If the current row and column are not equal to the current indices
                    if (i != q && j != p)
                    {
                        // Assign the element to the temporary matrix
                        b[m][n] = num[i][j];
                        // If the column index is less than f - 2, increment it
                        if (n < (f - 2))
                            n++;
                        // Otherwise, reset the column index and increment the row index
                        else
                        {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            // Calculate the determinant of the temporary matrix and assign it to the cofactor matrix
            fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
            printf("Calculation of the column #%d of the inverse A\n", p+1);
            display(b);
            printf("Determinant: %f\n", fac[q][p]);
        }
    }
    // Transpose the cofactor matrix
    transpose(num, fac, f);
}

// Function to calculate the transpose of a matrix and then find its inverse
void transpose(double num[N][N], double fac[N][N], float r)
{
    int i, j; // Indices for loops
    double b[N][N], inverse[N][N], d; // Temporary matrices and determinant

    // Transpose the cofactor matrix
    for (i = 0;i < r; i++)
    {
        for (j = 0;j < r; j++)
        {
            b[i][j] = fac[j][i];
        }
    }
    // Calculate the determinant of the original matrix
    d = determinant(num, r);
    // Divide each element of the transposed matrix by the determinant to get the inverse
    for (i = 0;i < r; i++)
    {
        for (j = 0;j < r; j++)
        {
            inverse[i][j] = b[i][j] / d;
        }
    }
    // Print the inverse matrix
    printf("\n\n\nInverse A matrix\n");
    for (i = 0;i < r; i++)
    {
        for (j = 0;j < r; j++)
        {
            printf("%f ", inverse[i][j]);
        }
        printf("\n");
    }
}


// Main function
int main() {
    double mat[N][N]; // Declare a NxN matrix

    getMatrix(mat); // Call the getMatrix function to fill the matrix with values

    printf("Given matrix A is :\n"); // Print a message
    display(mat); // Display the matrix
    printf("\n"); // Print a newline

    double det = determinant(mat, N); // Calculate the determinant of the matrix

    // If the determinant is zero, the matrix is not invertible
    if (det == 0)
        printf("\nInverse of Entered Matrix is not possible\n");
    // Otherwise, calculate the cofactor matrix
    else
        cofactor(mat, N);
}
