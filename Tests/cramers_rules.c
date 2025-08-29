#include <stdio.h>

#define N 3 // Change this value according to the size of your matrix

void printMatrix(float matrix[N][N+1]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N + 1; j++) {
            printf("%.2f ", matrix[i][j]);
        }
        printf("\n");
    }
}

float determinant(float matrix[N][N]) {
    float det = 0;
    float submatrix[N][N];

    if (N == 2) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    } else {
        for (int k = 0; k < N; k++) {
            int subi = 0;
            for (int i = 1; i < N; i++) {
                int subj = 0;
                for (int j = 0; j < N; j++) {
                    if (j == k)
                        continue;
                    submatrix[subi][subj] = matrix[i][j];
                    subj++;
                }
                subi++;
            }
            det += (k % 2 == 0 ? 1 : -1) * matrix[0][k] * determinant(submatrix);
        }
    }
    return det;
}

void cramer(float matrix[N][N+1]) {
    float det_A = determinant(matrix);
    if (det_A == 0) {
        printf("The system has no unique solution.\n");
        return;
    }

    float solution[N];
    float temp_matrix[N][N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            temp_matrix[i][j] = matrix[i][j];
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            temp_matrix[j][i] = matrix[j][N];
        }
        solution[i] = determinant(temp_matrix) / det_A;
        for (int j = 0; j < N; j++) {
            temp_matrix[j][i] = matrix[j][i];
        }
    }

    printf("The solution is:\n");
    for (int i = 0; i < N; i++) {
        printf("x%d = %.2f\n", i + 1, solution[i]);
    }
}

int main() {
    float matrix[N][N+1];
    //float *matrix1 = &matrix[0][0];
    printf("Enter the augmented matrix (%dx%d+1 elements):\n", N, N);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N + 1; j++) {
            scanf("%f", &matrix[i][j]);
        }
    }

    printf("\nThe augmented matrix is:\n");
    printMatrix(matrix);

    printf("\n");

    cramer(matrix);

    return 0;
}

/*#include <stdio.h>

// Function to compute the minor of a 3x3 matrix
void computeMinor(int matrix[3][3], int row, int col) {
    // Initialize variables for the submatrix
    int submatrix[2][2];
    int sub_i = 0, sub_j = 0;

    // Create the submatrix by skipping the specified row and column
    for (int i = 0; i < 3; i++) {
        if (i == row) {
            continue;  // Skip the specified row
        }
        for (int j = 0; j < 3; j++) {
            if (j == col) {
                continue;  // Skip the specified column
            }
            submatrix[sub_i][sub_j] = matrix[i][j];
            sub_j++;
        }
        sub_i++;
        sub_j = 0;
    }

    // Calculate the determinant of the submatrix
    int minor = submatrix[0][0] * submatrix[1][1] - submatrix[0][1] * submatrix[1][0];

    printf("Minor of the matrix at (%d, %d) is: %d\n", row, col, minor);
}

int main() {
    int matrix[3][3] = {{1, 2, 3},
                        {4, 5, 6},
                        {7, 8, 9}};

    // Compute the minor for each element of the matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            computeMinor(matrix, i, j);
        }
    }

    return 0;
}*/






