#include <stdio.h>
#include <stdlib.h>

void fillMatrix(double **matrix, int size){
    int i, j;
    printf("Enter the elements one by one:\n");
    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            printf("Enter element [%d][%d]: ", i, j);
            scanf("%lf", &matrix[i][j]);
        }
    }
}

double det(double **matrix, int size) {
    if (size == 1) {
        return matrix[0][0];
    } else if (size == 2) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    } else if (size == 3){
        return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
               matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
               matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }
}

int isSymmetric(double **matrix, int size) {
    int i, j;
    for (i = 0; i < size; i++) {
        for (j = 0; j < i; j++) {
            if (matrix[i][j] != matrix[j][i]) {
                return 0; 
            }
        }
    }
    return 1; 
}

void printMatrix(double **matrix, int size) {
    int i, j;
    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            printf("%.2lf ", matrix[i][j]);
        }
        printf("\n");
    }
}

void positiveDef(double **matrix, int size){
    int i, j ,k, countp = 0, countn = 0;
    printf("\nLeading principal minors of the symmetric matrix:\n\n");
    for (i = 1; i <= size; i++) {
        printf("Minor %d:\n", i);
        double **minorMatrix = (double **)malloc(i * sizeof(double *));
        for (j = 0; j < i; j++) {
            minorMatrix[j] = (double *)malloc(i * sizeof(double));
            for (k = 0; k < i; k++) {
                minorMatrix[j][k] = matrix[j][k];
            }
        }
        printMatrix(minorMatrix, i);
        printf("******************\n");
        switch (i) {
            case 1:
                printf("Determinant is %.2lf\n", det(minorMatrix,1));
                if (det(minorMatrix,1) > 0)
                    countp++;
                else if (det(minorMatrix,1) < 0)
                    countn++;
                break;
            case 2:
                printf("Determinant is %.2lf\n", det(minorMatrix,2));
                if ( det(minorMatrix,2) > 0 )
                    countp++;
                else if ( det(minorMatrix,2) < 0 )
                    countn++;
                break;
            case 3:
                printf("Determinant is %.2lf\n", det(minorMatrix,3));
                if (det(minorMatrix,3) > 0)
                    countp++;
                else if (det(minorMatrix,3) < 0)
                    countn++;
                break;
        }
        printf("******************\n");

        for (j = 0; j < i; j++) {
            free(minorMatrix[j]);
        }
        free(minorMatrix);
    }
    
    if (countp == size)
        printf("The matrix is positive definite.\n\n");
    else if (countn == size)
        printf("The matrix is negative definite.\n\n");
    else
        printf("The matrix is neither positive nor negative definite.\n\n");
}


int main() {
    
    int size;
//while(1){
	
    printf("Enter the size of the square matrix (up to 3): ");
    scanf("%d", &size);

    while(size < 1 || size > 3) {
        printf("Size should be between 1 and 3.\n");
        printf("Enter the size of the square matrix (up to 3): ");
        scanf("%d", &size);
    }

    double **matrix = (double **)malloc(size * sizeof(double *));
    if (matrix == NULL) {
        printf("Memory allocation failed.\n");
        return 1;
    }

    for (int i = 0; i < size; i++) {
        matrix[i] = (double *)malloc(size * sizeof(double));
        if (matrix[i] == NULL) {
            printf("Memory allocation failed.\n");
            return 1;
        }
    }

    fillMatrix(matrix, size);  

    if (!isSymmetric(matrix, size)) {
        printf("The matrix is not symmetric.\n");
        return 1;
    }

    positiveDef(matrix, size);

    for (int i = 0; i < size; i++) {
        free(matrix[i]);
    }
    free(matrix);
//}
    return 0;
}

