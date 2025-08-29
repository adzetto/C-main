#include <stdio.h>
#define A(I,J) (*(A+(I)*nCol+(J)))
#define B(I,J) (*(B+(I)*nCol+(J)))

/**/
void create_Matrix(int nRow, int nCol,double *A){
    printf("enter the matrix is\n");
    for(int i=0;i<nRow;i++){
        for(int j=0;j<nCol;j++){
            scanf("%lf", &A(i,j));
        }
    }
}
void print_Matrix(int nRow, int nCol,double *A){
    printf("your matrix  is\n");
    for(int i=0;i<nRow;i++){
        for(int j=0;j<nCol;j++){
            printf("%lf\t",A(i,j));
        }
        printf("\n\n");
    }
}

void matAdd(int nRow ,int nCol, double *A, double *B){
    int i,j;
    for(i=0;i<nRow;i++){
        for(j=0;j<nCol;j++){
            printf("%lf ", A(i,j) + B(i,j));
        }
        printf("\n");
    }
}
void matMul(int nRow1, int nCol1, int nRow2, int nCol2,  double A[nRow1][nCol1], double B[nRow2][nCol2]) {
    int i,j,k;
    double R[nRow1][nCol2];
    printf("result matrix is\n:");
    for(i=0;i<nRow1;i++){
        for(j=0;j<nCol2;j++){
            R[i][j]	=0;
            for(k=0;k<nRow2;k++){
                R[i][j]= R[i][j]+A[i][k]*B[k][j];
            }
            printf("%lf\t",R[i][j]);
        }
        printf("\n\n");
    }

}
void matTra(int nRow,int nCol,double A[nRow][nCol]) {
    int i,j;
    double tra[nCol][nRow];

    for(i=0;i<nRow;i++){
        for(j=0;j<nCol;j++){
            tra[i][j]=A[j][i];
        }
    }
    for(i=0;i<nCol;i++){
        for(j=0;j<nRow;j++){
            printf("%.3lf\t ",tra[i][j]);
        }
        printf("\n\n");
    }
}



#undef A
#undef B

int main() {
    int row1,column1,row2,column2,select;
    double A[10][10], B[10][10];

    printf("Enter the row number:");
    scanf("%d",&row1);
    printf("Enter the column number:");
    scanf("%d",&column1);

    printf("please chose matrix operation you want to do\n");
    printf("1.Matrix addition\n2.Matrix multiplication\n3.Matrix transpose\n");
    scanf("%d",&select);

    switch(select){
        case 1:;
            printf("Matrix addition selected:\n ");
            printf("\n Create the content of the first operand matrix:\n");
            create_Matrix(row1,column1, &A[0][0]);
            printf("\n Create the content of the second operand matrix:\n");
            create_Matrix(row1,column1, &B[0][0]);
            printf("Matrix addition:\n");
            matAdd(row1,column1, &A[0][0], &B[0][0]);
            break;

        case 2:;
            printf("Matrix multiplication selected:\n");
            printf("Enter the dimensions of the second matrix (rows columns):\n");
            scanf("%d", &row2);
            scanf("%d", &column2);

            if (column1 == row2) {
                printf("the first matrix is\n");
                create_Matrix(row1, column1, &A[0][0]);
                print_Matrix(row1,column1,&A[0][0]);
                printf("the second matrix is\n");
                create_Matrix(row2, column2, &B[0][0]);
                print_Matrix(row2,column2,&B[0][0]);
                matMul(row1, column1, row2, column2, A, B);
            } else {
                printf("Error: Incompatible dimensions for matrix multiplication.\n");
            }
            break;
        case 3:;
            printf("Matrix transpose selected:\n ");
            printf("Matrix transpose:\n");
            printf("the first matrix is\n");
            create_Matrix(row1, column1, &A[0][0]);
            print_Matrix(row1,column1,&A[0][0]);
            matTra(row1,column1,A);
            break;
        default:;
            printf("WRONG KEY ENTER!!!\n");
            break;
    }



}