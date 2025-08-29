#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#define A(I,J) (*(A + (I)*(4) + (J)))
#define B(I,J) (*(B + (I)*(3) + (J)))
#define C(I,J) (*(C + (I)*(4) + (J)))
#define D(I,J) (*(D + (I)*(4) + (J)))
#define E(I,J) (*(E + (I)*(4) + (J)))
#define F(I,J) (*(F + (I)*(4) + (J)))
#define G(I) (*(G + (I)))

// Function to calculate minors of a matrix
double minors(double *A, double *B, int r, int c) {
    int i, j, x = 0, y = 0;
    double result = 0.0;

    // Loop through the matrix
    for(i = 0; i < 4; i++) {
        if(i == r) {
            continue;
        }
        for(j = 0; j < 4; j++) {
            if(j == c) {
                continue;
            }
            B(x, y) = A(i, j);
            y++;
        }
        y = y - 3;
        x++;
    }

    // Calculate the result
    result = (B(0, 0) * ((B(1, 1) * B(2, 2) - B(2, 1) * B(1, 2))) - B(0, 1) * (B(1, 0) * B(2, 2) - B(2, 0) * B(1, 2)) + B(0, 2) * (B(1, 0) * B(2, 1) - B(2, 0) * B(1, 1)));
    return result;
}

// Function to calculate cofactor of a matrix
void cofactor(double *A, double *B, double *C) {
    int i, j;

    // Loop through the matrix
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            C(i, j) = pow(-1, i + j) * (minors(A, B, i, j));
        }
    }
}

// Function to calculate adjoint of a matrix
double adjoint(double *C, double *D) {
    int i, j;

    // Loop through the matrix
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            D(i, j) = C(j, i);
        }
    }

    return *D;
}

// Function to calculate determinant of a matrix
double det_calculator(double *work, double *C) {
    int i;
    double det = 0;

    // Loop through the matrix
    for(i = 0; i < 4; i++) {
        det += work[i][0] * C(i, 0);
    }

    return det;
}

// Function to multiply a matrix and a vector
double matrix_mult(double *F, double *G) {
    int i, k;
    double array[4][1];
    double val;

    // Loop through the matrix
    for(i = 0; i < 4; i++) {
        val = 0.0;
        for(k = 0; k < 4; k++) {
            val += F(i, k) * G(k);
        }
        array[i][0] = val;
        printf("%f\n", val);
    }
}

int main() {
    double *A, *B, *C, *D, *E, *F, *G;

    // Allocate memory for matrices
    A = (double*)malloc(4 * 4 * sizeof(double));
    B = (double*)malloc(3 * 3 * sizeof(double));
    C = (double*)malloc(4 * 4 * sizeof(double));
    D = (double*)malloc(4 * 4 * sizeof(double));
    E = (double*)malloc(4 * 4 * sizeof(double));
    F = (double*)malloc(4 * 4 * sizeof(double));
    G = (double*)malloc(4 * sizeof(double));

    // Input matrix A
    printf("Content of the first operand matrix:\n");
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            scanf("%lf", &A(i, j));
        }
    }

    // Input vector G
    printf("Enter elements in vector of size 4x1: \n");
    for(int i = 0; i < 4; i++) {
        scanf("%lf", &G(i));
    }

    // Calculate cofactor, adjoint, and determinant
    cofactor(A, B, C);
    adjoint(C, D);
    double det = det_calculator(A, C);

    // Calculate inverse of matrix
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            F(i, j) = D(i, j) / det;
        }
    }

    // Print inverse of matrix
    printf("Inverse of matrix:\n");
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            printf("%f\t", F(i, j));
        }
        printf("\n");
    }

    // Print vector G
    for(int i = 0; i < 4; i++) {
        printf("%f", G(i));
    }

    // Print vector X
    printf("X Vector:\n ");
    matrix_mult(F, G);

    return 0;
}
