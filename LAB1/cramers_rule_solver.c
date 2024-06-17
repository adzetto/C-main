#include <stdio.h>

#define N 10

void getCofactor(float A[N][N], float temp[N][N], int p, int q, int n) {
    int i = 0, j = 0;
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            if (row != p && col != q) {
                temp[i][j++] = A[row][col];
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

float determinant(float A[N][N], int n) {
    float D = 0;
    if (n == 1) {
        return A[0][0];
    }

    float temp[N][N];
    int sign = 1;
    for (int f = 0; f < n; f++) {
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);
        sign = -sign;
    }

    return D;
}

void transpose(float A[N][N], float B[N][N], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            B[i][j] = A[j][i];
        }
    }
}

int main() {
    int n;
    float A[N][N], B[N], X[N], detA;

    printf("Enter the number of variables (1-%d): ", N);
    scanf("%d", &n);

    printf("Enter the augmented matrix A|B:\n");
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            scanf("%f", &A[i][j]);
        }
    }

    printf("Enter the vector B:\n");
    for (int i = 0; i < n; i++) {
        scanf("%f", &B[i]);
    }

    detA = determinant(A, n);

    if (detA == 0) {
        printf("Determinant of A is zero. Cramer's Rule does not apply.\n");
        return 1;
    }

    float A_transpose[N][N];
    transpose(A, A_transpose, n);

    for (int i = 0; i < n; i++) {
        float tempA[N][N];
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                if (k == i) {
                    tempA[j][k] = B[j];
                } else {
                    tempA[j][k] = A[j][k];
                }
            }
        }
        X[i] = determinant(tempA, n) / detA;
    }

    printf("Solution for the system of equations Ax = B:\n");
    for (int i = 0; i < n; i++) {
        printf("x%d = %.4f\n", i+1, X[i]);
    }

    return 0;
}
