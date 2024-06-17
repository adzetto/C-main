#include <stdio.h>
#include <stdlib.h>
#include <time.h>


struct Matrix
{
    int rows, cols, **data;

};

int menuSelection();
struct Matrix createMatrix(int rows, int cols);
int *createArr(int n);
void freeMatrix(struct Matrix mat);

int *get_row_col();
int addCheck(struct Matrix m1, struct Matrix m2);
struct Matrix addMatricies(struct Matrix mat1 ,struct Matrix mat2);
void printMatrix(struct Matrix);
void fillMatrix(struct Matrix mat);
struct Matrix TransposeMatrix(struct Matrix mat);
struct Matrix multMatrix(struct Matrix mat1, struct Matrix mat2);
int determinant(struct Matrix mat);

int main()
{
    srand(time(0));
    struct Matrix Matrix1;
    struct Matrix Matrix2;

    int choice = menuSelection();
    switch (choice) {
        case 1:; /// Addition

            printf("\n Matrix 1 created ...");

            printf("\n Matrix 2 created ...");


            int *rowsANDcols = get_row_col();
            printf("\n rows and cols are achieved ...");
            Matrix1 = createMatrix(*rowsANDcols, *(rowsANDcols + 1));
            printf("\n Matrix 1 created according to the rows and cols ...");

            rowsANDcols = get_row_col();
            Matrix2 = createMatrix(*(rowsANDcols), *(rowsANDcols+1));
            printf("\n Matrix 2 created according to the rows and cols ...");

            /*
            Matrix1 = createMatrix(100, 80);
            printf("\n Matrix 1 created according to the rows and cols ...");
            Matrix2 = createMatrix(100, 80);
            printf("\n Matrix 2 created according to the rows and cols ...");
            */
            /*
            printf("\n First Matrix:");
            printMatrix(Matrix1);
            printf("\n Second Matrix:");
            printMatrix(Matrix2);
            */

            int check = addCheck(Matrix1, Matrix2);
            if(check == 0){
                return 0;
            }
            else if(check == 1){
                //add

                fillMatrix(Matrix1);
                printf("\n Matrix1 is filled ...");

                fillMatrix(Matrix2);
                printf("\n Matrix2 is filled ...");
                /**/
                struct Matrix additionalMatrix = addMatricies(Matrix1, Matrix2);

                printf("\n\n\n Additional Matrix is,\n");
                printMatrix(additionalMatrix);
                printf("\n\n");
                freeMatrix(additionalMatrix);
            }

            free(rowsANDcols);

            break;
        case 2:; /// Multiplication

            rowsANDcols = get_row_col();
            Matrix1 = createMatrix(*rowsANDcols, *(rowsANDcols + 1));
            fillMatrix(Matrix1);
            printf("\n First Matrix:\n");
            printMatrix(Matrix1);

            rowsANDcols = get_row_col();
            Matrix2 = createMatrix(*rowsANDcols, *(rowsANDcols + 1));
            fillMatrix(Matrix2);
            printf("\n Second Matrix:\n");
            printMatrix(Matrix2);

            struct Matrix resultant = multMatrix(Matrix1,Matrix2);
            printf("\n resultant matrix, \n");
            printMatrix(resultant);

            freeMatrix(resultant);
            free(rowsANDcols);
            break;
        case 3:; /// Transpose
            rowsANDcols = get_row_col();
            Matrix1 = createMatrix(*rowsANDcols, *(rowsANDcols + 1));
            fillMatrix(Matrix1);
            printf("\n Before Transpose:\n");
            printMatrix(Matrix1);
            struct Matrix transposed = createMatrix(Matrix1.cols, Matrix1.rows);
            transposed = TransposeMatrix(Matrix1);
            printf("\n Transposed Matrix:\n");
            printMatrix(transposed);

            freeMatrix(transposed);
            break;
        case 4:; /// Determinant
            rowsANDcols = get_row_col();
            Matrix1 = createMatrix(*rowsANDcols, *(rowsANDcols + 1));
            fillMatrix(Matrix1);
            printf("\n Your matrix is,\n");
            printMatrix(Matrix1);
            int det = determinant(Matrix1);
            printf("\n Determinant of your matrix is : %d \n", det);

            free(rowsANDcols);
            break;
    }

    freeMatrix(Matrix1);
    freeMatrix(Matrix2);
    return 0;
}

int menuSelection()
{
    int choice;
    printf("\n #---------- Welcome to the MATRIX OPERATOR! ----------#");
    printf("\n 1. Addition,");
    printf("\n 2. Multiplication,");
    printf("\n 3. Transpose,");
    printf("\n 4. Determinant,");
    printf("\n Enter your choice: ");
    scanf("%d", &choice);
    return choice;
}


struct Matrix createMatrix(int rows, int cols) {
    struct Matrix mat;
    mat.rows = rows;
    mat.cols = cols;

    // Allocate memory for matrix data
    mat.data = (int **)malloc(rows * sizeof(int *));
    if(mat.data == NULL){
        fprintf(stderr, "Allocation is failed at rows");
        exit(1);
    }
    for (int i = 0; i < rows; i++) {
        mat.data[i] = (int *)malloc(cols * sizeof(int));
        if(mat.data[i] == NULL){
        fprintf(stderr, "Allocation is failed at cols");
        exit(1);
        }
    }

    // Initialize matrix elements to zero
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            mat.data[i][j] = 0;
        }
    }

    return mat;
}

int *createArr(int n)
{
    int *arr = (int *)malloc(n * sizeof(int));
    return arr;
}

void printMatrix(struct Matrix mat)
{
    int i, j;
    printf("\n");
    for(i=0;i<mat.rows;i++){
        printf("|");
        for(j=0;j<mat.cols;j++){
            printf("%5d", mat.data[i][j]);
            if(j == mat.cols-1){
                printf(" ");
            }
        }
        printf("|\n");
    }
}

void freeMatrix(struct Matrix mat) {
    for (int i = 0; i < mat.rows; i++) {
        free(mat.data[i]);
    }
    free(mat.data);
}

int *get_row_col()
{
    int *data = (int *)malloc(2 * sizeof(int));
    if(data == NULL)
    {
        fprintf(stderr, "\n Allocation is failed while getting the rows and cols");
        exit(1);
    }

    int row, col;

    printf("\n Enter the number of rows of the matrix: ");
    scanf("%d", &row);
    printf("\n Enter the number of cols of the matrix: ");
    scanf("%d", &col);

    *(data) = row;
    *(data + 1) = col;

    return data;
}

void fillMatrix(struct Matrix mat)
{
    int i, j, val;
    for(i=0;i<mat.rows;i++){
        for(j=0;j<mat.cols;j++){
            printf("\n Enter the [%d][%d]: ", i, j);
            scanf("%d", &(*(*(mat.data + i) + j)));
        }
    }
}


int addCheck(struct Matrix m1, struct Matrix m2)
{
    if((m1.rows != m2.rows) && (m1.cols != m2.cols)){
        printf("\n Your matrices are not appropriate for addition operation");
        return 0;
    }
    else{
        return 1;
    }
}

struct Matrix addMatricies(struct Matrix mat1 ,struct Matrix mat2)
{
    struct Matrix resultantMat = createMatrix(mat1.rows, mat1.cols);
    int i, j;
    for(i=0;i<mat1.rows;i++){
        for(j=0;j<mat1.cols;j++){
            resultantMat.data[i][j] = mat1.data[i][j] + mat2.data[i][j];
        }
    }

    return resultantMat;
}


struct Matrix multMatrix(struct Matrix mat1, struct Matrix mat2) {
    if (mat1.cols != mat2.rows) {
        fprintf(stderr, "Error: Matrix dimensions are not compatible for multiplication.\n");
        exit(1);
    }

    struct Matrix result = createMatrix(mat1.rows, mat2.cols);

    for (int i = 0; i < mat1.rows; i++) {
        for (int j = 0; j < mat2.cols; j++) {
            result.data[i][j] = 0;
            for (int k = 0; k < mat1.cols; k++) {
                result.data[i][j] += mat1.data[i][k] * mat2.data[k][j];
            }
        }
    }

    return result;
}

struct Matrix TransposeMatrix(struct Matrix mat)
{
    struct Matrix transposed = createMatrix(mat.cols, mat.rows);
    int i,j;
    for(i=0;i<transposed.rows;i++){
        for(j=0;j<transposed.cols;j++){
            transposed.data[i][j] = mat.data[j][i];
        }
    }
    return transposed;
}

int determinant(struct Matrix mat)
{
    int det;
    if(mat.rows != mat.cols){
        printf("\n !Your Matrix is NOT APPROPRIATE for determinant!\n\n");
        exit(1);
    }

    if(mat.rows==2){
        /// 2x2 matrix
        det = (mat.data[0][0] * mat.data[1][1]) - (mat.data[0][1] * mat.data[1][0]);
    }
    else if(mat.rows == 3){
        /// 3x3 matrix
        det = ((mat.data[0][0]) * ((mat.data[1][1] * mat.data[2][2]) - (mat.data[2][1] * mat.data[1][2]))) - ((mat.data[0][1]) * ((mat.data[1][0] * mat.data[2][2]) - (mat.data[2][0] * mat.data[1][2])))  +  ((mat.data[0][2]) * ((mat.data[1][0] * mat.data[2][1]) - (mat.data[2][0] * mat.data[1][1])));
    }

    return det;
}
