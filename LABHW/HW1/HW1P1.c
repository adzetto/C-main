#include <stdio.h>
#include <stdlib.h>

int isSymmetric(double **matrix, int n);
double determinant(double **matrix, int n);
void printMatrix(double **matrix, int n);
int checkPositiveDefinite(double **matrix, int n);
int checkNegativeDefinite(double **matrix, int n);
int checkSemidefinite(double **matrix, int n, int isPositive);

int isSymmetric(double **matrix, int n) {
    // Iterate over rows of the matrix
    for (int i = 0; i < n; i++) {
        // Only check elements below the diagonal (j < i)
        for (int j = 0; j < i; j++) {
            // Check if the element at (i, j) is not equal to the element at (j, i)
            if (matrix[i][j] != matrix[j][i]) {
                // If any element is not equal to its transpose counterpart, return 0 (not symmetric)
                return 0;
            }
        }
    }
    // If all checked elements are equal to their transposed counterparts, return 1 (symmetric)
    return 1;
}


double determinant(double **matrix, int n) {
    double det = 0.0;

    if (n == 1) {
        det = matrix[0][0];
    }

    else if (n == 2) {
        det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    }

    else if (n == 3) {
        det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }
    return det;
}


void printMatrix(double **matrix, int n) {
    printf("Matrix (%d x %d):\n", n, n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            // Print each element formatted to 4 decimal places, aligned right in a field of 10 characters
            printf("%10.4lf ", matrix[i][j]);
        }

        printf("\n");
    }
}


int checkPositiveDefinite(double **matrix, int n) {
    // Loop through all sizes of leading principal submatrices from 1x1 up to nxn
    for (int i = 1; i <= n; i++) {
        // Dynamically assign memory to the i x i submatrix
        double **subMatrix = (double **)malloc(i * sizeof(double *));
        for (int j = 0; j < i; j++) {
            subMatrix[j] = (double *)malloc(i * sizeof(double));

            // Transfer the upper-left quadrant of the initial matrix into subMatrix
            for (int k = 0; k < i; k++) {
                subMatrix[j][k] = matrix[j][k];
            }
        }

        // Calculate the determinant of the current i x i subMatrix
        double det = determinant(subMatrix, i);

        // Free the memory
        for (int j = 0; j < i; j++) free(subMatrix[j]);
        free(subMatrix);

        // Check if the determinant is non-positive. If so, the matrix is not positive definite
        if (det <= 0) return 0;
    }

    // If all determinants are positive, the matrix is positive definite, return 1
    return 1;
}


int checkNegativeDefinite(double **matrix, int n) {
    for (int i = 1; i <= n; i++) {
        double **subMatrix = (double **)malloc(i * sizeof(double *));
        for (int j = 0; j < i; j++) {
            subMatrix[j] = (double *)malloc(i * sizeof(double));

            for (int k = 0; k < i; k++) {
                subMatrix[j][k] = matrix[j][k];
            }
        }

        double det = determinant(subMatrix, i);

        for (int j = 0; j < i; j++) free(subMatrix[j]);
        free(subMatrix);

        // For a matrix to be negative definite:
        // - Determinants of even-indexed submatrices should be negative
        // - Determinants of odd-indexed submatrices should be positive
        // If these conditions are not met, return 0
        if (((i % 2 == 0) && det <= 0) || ((i % 2 != 0) && det >= 0)) return 0;
    }

    // If all determinants meet the conditions for negative definiteness, return 1
    return 1;
}


// 'isPositive' is a flag indicating whether we want to check for positive semidefiniteness (1 for positive, 0 for negative)
int checkSemidefinite(double **matrix, int n, int isPositive) {

    for (int i = 1; i <= n; i++) {
        double **subMatrix = (double **)malloc(i * sizeof(double *));
        for (int j = 0; j < i; j++) {
            subMatrix[j] = (double *)malloc(i * sizeof(double));

            for (int k = 0; k < i; k++) {
                subMatrix[j][k] = matrix[j][k];
            }
        }

        double det = determinant(subMatrix, i);

        for (int j = 0; j < i; j++) {
            free(subMatrix[j]);
        }
        free(subMatrix);

        // Check the determinant against the rules for semidefiniteness
        // If checking for positive semidefiniteness and any determinant is negative, return 0
        if (isPositive && det < 0) return 0;

        // If checking for negative semidefiniteness, use the property:
        // - The signs of the determinants of the principal minors should alternate starting with negative
        // If even index submatrices have positive determinants, or odd index submatrices have negative determinants, return 0
        if (!isPositive && ((i % 2 == 0 && det > 0) || (i % 2 != 0 && det < 0))) return 0;
    }

    // If all determinants meet the conditions for the specified type of semidefiniteness, return 1
    return 1;
}

int main() {
    int n;
    printf("Enter the matrix size (type up to 3 for a 3x3 matrix): ");
    scanf("%d", &n);

    if (n < 1 || n > 3) {
        printf("Error: Matrix size out of specified bounds.\n");
        return 1;
    }

    double **matrix = (double **)malloc(n * sizeof(double *));
    for (int i = 0; i < n; i++) {
        matrix[i] = (double *)malloc(n * sizeof(double));
        for (int j = 0; j < n; j++) {
            printf("Enter element [%d][%d]: ", i + 1, j + 1);
            scanf("%lf", &matrix[i][j]);
        }
    }

    if (!isSymmetric(matrix, n)) {
        printf("The matrix is not symmetric.\n");
    } else {
        printf("\nChecking matrix definiteness...\n\n");
        int positiveDefinite = 1;
        int negativeDefinite = 1;
        int positiveSemidefinite = 1;
        int negativeSemidefinite = 1;

        for (int i = 1; i <= n; i++) {
            // Create submatrix for leading principal minor
            double **subMatrix = (double **)malloc(i * sizeof(double *));
            for (int j = 0; j < i; j++) {
                subMatrix[j] = (double *)malloc(i * sizeof(double));
                for (int k = 0; k < i; k++) {
                    subMatrix[j][k] = matrix[j][k];
                }
            }

            // Display the submatrix and its determinant
            printf("Evaluating Leading Principal Minor %d:\n", i);
            printMatrix(subMatrix, i);
            double det = determinant(subMatrix, i);
            printf("Determinant: %.4lf\n", det);

            // Definiteness checks for each minor
            if (det <= 0) positiveDefinite = 0;
            if ((i % 2 == 0 && det >= 0) || (i % 2 != 0 && det <= 0)) negativeDefinite = 0;
            if (det < 0) positiveSemidefinite = 0;
            if ((i % 2 == 0 && det < 0) || (i % 2 != 0 && det > 0)) negativeSemidefinite = 0;

            // Free submatrix memory
            for (int j = 0; j < i; j++) {
                free(subMatrix[j]);
            }
            free(subMatrix);
        }

        // Conclude definiteness
        if (positiveDefinite) {
            printf("The matrix is positive definite.\n");
        } else if (negativeDefinite) {
            printf("The matrix is negative definite.\n");
        } else if (positiveSemidefinite) {
            printf("The matrix is positive semidefinite.\n");
        } else if (negativeSemidefinite) {
            printf("The matrix is negative semidefinite.\n");
        } else {
            printf("The matrix does not meet any of the definiteness conditions explicitly.\n");
        }
    }

    // Cleanup and exit
    for (int i = 0; i < n; i++) {
        free(matrix[i]);
    }
    free(matrix);
    return 0;
}
