#include <stdio.h>
#include <stdlib.h>
/* macro for m[i][j] */
//#define mat1(I,J) (*(mat1+(I)*(10)+(J))) 
//#define mat2(I,J) (*(mat2+(I)*(10)+(J)))

void createMatrix(int row,int col,int mat[][10]){
    int i,j;
	printf("\nEnter the first matrix's values respectively.\nAfter each value, press enter.\nYou need To enter 100 integer values\n:");
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            scanf("%d", &mat[i][j]);
        }
    }
}

void printMatrix(int row, int col, int mat[][10]){
    int i,j;
	printf("\n\nYour matrix:\n");
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            printf("%5d", mat[i][j]);
        }
        printf("\n\n");
    }
}

void matAdd(int row, int col, int *mat1, int *mat2) {
    int i,j,mat3[10][10];
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            mat3[10][10]= *(mat1+i + 10*j) + *(mat2+i+ 10*j);
        }
    }
    printf("\n\nYour matrix:\n");
    for(i=0;i<row;i++){
        for(j=0;j<col;j++){
            printf("%5d", mat3[i][j]);
        }
        printf("\n\n");
    }
}



int main()
{
    int mat1[10][10], mat2[10][10];
    int choice;
    int row1,row2,col1,col2;
    int *ptr1=&mat1[0][0];
    int *ptr2=&mat1[0][0];
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
        matAdd( row1 , col1,  ptr1, ptr2);
        
	}
        
        
        return 0;
}
