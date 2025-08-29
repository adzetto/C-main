#include <stdio.h>

#define MAX_SIZE 10

int isSymmetric(int mat[MAX_SIZE][MAX_SIZE], int rows, int cols) {
    // A matrix is symmetric if it is square and equal to its transpose
    if (rows != cols) {
        return 0;
    }

    int transpose[MAX_SIZE][MAX_SIZE];

    // Calculating the transpose of the matrix
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            transpose[j][i] = mat[i][j];
        }
    }

    // Checking if the matrix is equal to its transpose
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (mat[i][j] != transpose[i][j]) {
                return 0;
            }
        }
    }

    return 1;
}

int main() {
    int mat[MAX_SIZE][MAX_SIZE];
    int rows, cols;

    printf("Enter the number of rows and columns of the matrix: ");
    scanf("%d %d", &rows, &cols);

    printf("Enter the elements of the matrix:\n");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            scanf("%d", &mat[i][j]);
        }
    }

    if (isSymmetric(mat, rows, cols)==1) {
        printf("The matrix is symmetric.\n");
    } else {
        printf("The matrix is not symmetric.\n");
    }

    return 0;
}
