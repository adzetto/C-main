#include <stdio.h>
#include <stdlib.h>
#define n 100

void create_matrix(int r ,int c,int M[][100]){

    printf("take the rows and column:");
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            scanf("%d",&M[i][j]);
        }
    }

}
void print_matrix(int r ,int c,int M[][n]) {
    printf("your matrix is:\n");
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            printf("%d\t",M[i][j]);
        }
        printf("\n");
    }

}
void add_matrix(int r,int c,int A[][n],int B[][n],int C[][n]){
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            C[i][j] = A[i][j]+B[i][j];
        }

    }

}

void multiply_matrix(int r1,int c1, int c2, int A[][n],int B[][n],int C[][n]){
    int i,j,k;
    for (i=0;i<r1;i++){
        for (j=0;j<c2;j++){
            C[i][j]=0;
            for (k=0;k<c1;k++){
                C[i][j] = C[i][j] + A[i][k]*B[k][j];
            }
        }
    }
}



void transpose_matrix(int r,int c,int A[][n],int C[][n]){
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            C[i][j]=A[j][i];
        }
        printf("\n");
    }
}

int determinant_matrix(int r,int c,int A[][n]){
    int det;
    if(r==2 && c==2) {
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    }
    else if(r==3 && c==3)
    {
        det=A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])-A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])+A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    }
    return det;
}

int main() {
    int row1, col1, row2, col2, choice,determinant;
    int A[n][n], B[n][n], C[n][n];
    while(1)
    {


        printf("please select the operation:\n");
        printf("-----------------------------");
        printf("1-matrix addition\n2-matrix multiplication\n3-matrix transpose\n4-matrix determinant\n5.Exit\n");
        printf("please enter the choice:\n");
        scanf("%d", &choice);
        if (choice == 1) {
            printf("please enter the row1 row2 and col1 col2:\n");
            scanf("%d %d %d %d", &row1, &row2, &col1, &col2);

            while (!(row1 == row2 && col1 == col2)) {
                printf("your dimentiones is not correct\n");
                printf("please  again enter the row1 row2 and col1 col2:\n");
                scanf(" %d%d%d%d", &row1, &col1, &row2, &col2);
            }
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            create_matrix(row2, col2, B);
            print_matrix(row2, col2, B);

            add_matrix(row1, col1, A, B, C);
            print_matrix(row2, col2, C);
        } else if (choice == 2) {
            printf("please enter the row1 row2 and col1 col2:\n");
            scanf("%d %d %d %d", &row1, &row2, &col1, &col2);

            while (!(col1 == row2)) {
                printf("please enter the row1 row2 and col1 col2:\n");
                scanf("%d %d %d %d", &row1, &row2, &col1, &col2);
            }
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            create_matrix(row2, col2, B);
            print_matrix(row2, col2, B);
            multiply_matrix(row1, col1, col2, A, B, C);
            print_matrix(row2, col2, C);
        }
        else if(choice==3){
            printf("please enter the row1 col1:\n");
            scanf("%d %d", &row1, &col1);
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            transpose_matrix(row1,col1, A , C);
            print_matrix(row1, col1, C);

        }
        else if (choice==4) {
            printf("please enter the row1 col1:\n");
            scanf("%d %d", &row1, &col1);
            while (!((row1 == 2 && col1 == 2) || (row1 == 3 && col1 == 3))) {
                printf("your dimentiones is not correct\n");
                printf("please  again enter the row1 col1:\n");
                scanf(" %d%d", &row1, &col1);
            }
            if ((row1 == 2 && col1 == 2)) {
                create_matrix(row1, col1, A);
                print_matrix(row1, col1, A);
              determinant=  determinant_matrix(row1, col1, A);
                printf("%d\n",determinant);



            } else if ((row1 == 3 && col1 == 3)) {
                create_matrix(row1, col1, A);
                print_matrix(row1, col1, A);
                determinant_matrix(row1, col1, C);
                determinant=  determinant_matrix(row1, col1, A);
                printf("%d\n",determinant);

            }


        }
        else if(choice == 5){break;}
    }
    return 0;

}