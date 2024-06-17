#include <stdio.h>
#include <stdlib.h>

void createMatrix(int row,int col,int mat[][100]){
    int i,j;
	printf("\nEnter the first matrix's values respectively.\nAfter each value, press enter.\nYou need To enter 100 integer values\n:");
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            scanf("%d", &mat[i][j]);
        }
    }
}

void printMatrix(int row, int col, int mat[][100]){
    int i,j;
	printf("\n\nYour matrix:\n");
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            printf("%5d", mat[i][j]);
        }
        printf("\n\n");
    }
}

void matAdd(int row, int col, int mat1[][100], int mat2[][100], int mat3[][100]) {
    int i,j;
    for(i=0;i<row;i++){
        for(j=0;j<col;j++) {
            mat3[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}
void multmatrix(int mat1[][100], int mat2[][100], int mat3[][100], int row1, int col1, int row2,int col2){
        int i,j,k;
        for (i=0;i<row1;i++){
            for (j=0;j<col2;j++){
                mat3[i][j]=0;
                for (k=0;k<col1;k++){
                    mat3[i][j]+=mat1[i][k]*mat2[k][j];
                }
            }
        }
    }
void matrix_transpose(int row,int col,int mat[][100],int resultant[][100]){
    int i,j;
	for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            resultant[j][i]=mat[i][j];
        }
    }
}
int  matrix_determinanat(int row,int col, int mat1[][100] ){
    int determinant;
    if(row==2 &&col==2){
       determinant=mat1[0][0]*mat1[1][1]-mat1[0][1]*mat1[1][0];
       return  determinant;
    }
     else if(row==3 &&col==3){
         return (mat1[0][0] * (mat1[1][1] * mat1[2][2] - mat1[1][2] * mat1[2][1])) -
           (mat1[0][1] * (mat1[1][0] * mat1[2][2] - mat1[1][2] * mat1[2][0])) +
           (mat1[0][2] * (mat1[1][0] * mat1[2][1] - mat1[1][1] * mat1[2][0]));
         
     }
     
}
int main()
{
    int mat1[100][100], mat2[100][100] ,resultant[100][100];
    int choice;
    int row1,row2,col1,col2;
    
    printf("enter the choice:1.Addition ,2.Multiplication ,3.Transpose ,4.determinant");
    scanf("%d",&choice);
    if(choice==1){
        printf("enter the rows and columns:");
        scanf("%d %d %d %d",&row1,&col1,&row2,&col2);
        while(!(row1==row2 && col1==col2)){
            printf("enter the rows and columns:");
            scanf("%d %d %d %d",&row1,&col1,&row2,&col2);
        }
        createMatrix(row1, col1, mat1);
        printMatrix(row1, col1,mat1);
        createMatrix(row2, col2, mat2);
        printMatrix(row2, col2,mat2);
        matAdd( row1 , col1,  mat1, mat2,  resultant);
        printMatrix(row1, col2, resultant);
    }
    else if (choice == 2) {
    // Multiplication
    printf("Enter the rows and columns: ");
    scanf("%d %d %d %d", &row1, &col1, &row2, &col2);

    while (!(row2 == col1)) {
        printf("Invalid dimensions for multiplication. Enter the rows and columns again: ");
        scanf("%d %d %d %d", &row1, &col1, &row2, &col2);
    }

    createMatrix(row1, col1, mat1);
    printMatrix(row1, col1, mat1);
    createMatrix(row2, col2, mat2);
    printMatrix(row2, col2, mat2);

    multmatrix(mat1, mat2, resultant, row1, col1, row2, col2);
    printMatrix(row1, col2, resultant);
}

    else if(choice==3){
        printf("Enter the rows and columns: ");
        scanf("%d %d %d %d", &row1, &col1, &row2, &col2);
        createMatrix(row1, col1, mat1);
        printMatrix(row1, col1,mat1);
        printf("\n#--- Transpose of mat1 ---#\n");
        matrix_transpose(row1,col1, mat1, resultant);
        printMatrix(row1, col1, resultant);
        
        createMatrix(row2, col2, mat2);
        printMatrix(row2, col2,mat2);
        printf("\n#--- Transpose of mat2 ---#\n");
        matrix_transpose(row2,col2, mat2, resultant);
        printMatrix(row2, col2, resultant);
        
        
    }
    else if(choice==4){
        printf("enter the rows and columns:");
        scanf("%d %d %d %d",&row1,&col1,&row2,&col2);
        while(!(((row1 == 2 && col1 == 2) || (row2 == 2 && col2 == 2)) && ((row1 == 3 && col1 == 3) || (row2 == 3 && col2 == 3)))){
        printf("enter the rows and columns:");
        scanf("%d %d %d %d",&row1,&col1,&row2,&col2);
    }
        createMatrix(row1, col1, mat1);
        printMatrix(row1, col1,mat1);
        int det1= matrix_determinanat( row1, col1, mat1);
        printf("%d",det1);
        
        createMatrix(row2, col2, mat2);
        printMatrix(row2, col2,mat2);
        int det2= matrix_determinanat( row2, col2, mat2);
        printf("%d",det2);
    }
        
    
    return 0;
}
