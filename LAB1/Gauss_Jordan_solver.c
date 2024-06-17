#include <stdio.h>

#define N 3

void printMatrix(double mat[N][N+1]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= N; j++)
            printf("%f ", mat[i][j]);
        printf("\n");
    }
}

void swapRows(double mat[N][N+1], int row1, int row2) {
    for (int i = 0; i <= N; i++) {
        double temp = mat[row1][i];
        mat[row1][i] = mat[row2][i];
        mat[row2][i] = temp;
    }
}

void gaussJordan(double mat[N][N+1]) {
    for (int i = 0; i < N; i++) {
        // Find pivot for row i
        double pivot = mat[i][i];

        if (pivot == 0) {
            int found = 0;
            for (int j = i + 1; j < N; j++) {
                if (mat[j][i] != 0) {
                    swapRows(mat, i, j);
                    found = 1;
                    break;
                }
            }

            if (!found) {
                printf("No unique solution exists.\n");
                return;
            }

            pivot = mat[i][i];
        }

        // Make the diagonal element 1
        for (int j = i; j <= N; j++)
            mat[i][j] /= pivot;

        // Make other elements in this column 0
        for (int j = 0; j < N; j++) {
            if (i != j) {
                double factor = mat[j][i];
                for (int k = i; k <= N; k++)
                    mat[j][k] -= factor * mat[i][k];
            }
        }
    }
}

void copyMat(double mat1[N][N+1], double mat2[N][N+1])
{
    int i,j;
    for(i=0;i<N;i++)
    {
        for(j=0;j<=N;j++)
        {
            mat2[i][j] = mat1[i][j];
        }
    }
}

int main() {
    double mat[N][N+1], mat2[N][N+1];

    printf("Enter the elements of the augmented matrix (A|B):\n");
    for (int i = 0; i < N; i++) {
        printf("Enter row %d (separated by spaces): ", i+1);
        for (int j = 0; j <= N; j++) {
            scanf("%lf", &mat[i][j]);
        }
    }

    copyMat(mat, mat2);

    printf("Original Matrix:\n");
    printMatrix(mat);

    gaussJordan(mat2);

    printf("\nSolution:\n");
    for (int i = 0; i < N; i++)
        printf("x%d = %f\n", i+1, mat2[i][N]);

    return 0;
}
