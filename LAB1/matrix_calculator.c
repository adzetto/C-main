#include <stdio.h>
#include <stdlib.h>
#define S 10

void create_matrix(int r, int c, int M[S][S]); 

void print_matrix(int r,int c,int M[S][S]);

void add_matrix(int r,int c,int A[S][S],int B[S][S],int C[S][S]);

void multiply_matrix(int r1,int c1, int c2, int A[S][S],int B[S][S],int C[S][S]);

void transpose_matrix(int r,int c,int A[S][S],int C[S][S]);

int determinant_matrix(int r,int c,int A[S][S]);

int main() {
	
	int row1, col1, row2, col2, choice,determinant;
    int A[S][S], B[S][S], C[S][S];
	while(1){
	printf("---Wellcome to Matrix Calculator--- \nPlease Select the Operation:\n-----------------------------\n");
        printf("1-Matrix Addition\n2-Matrix Multiplication\n3-Matrix Transpose\n4-Matrix Determinant\n5.Close the Program\n");
        printf("Please enter your choice:\n");
        scanf("%d", &choice);
	
	
	if (choice == 1) {
            printf("Please enter the row1 col1 :\n");
            scanf("%d %d", &row1, &col1);
			printf("Please enter the row2 col2:\n");
            scanf("%d %d", &row2, &col2);
			
            while (!(row1 == row2 && col1 == col2)) {
                printf("!!!Your dimentiones does not match\n");
                printf("Please enter the row1 col1 :\n");
                scanf("%d %d", &row1, &col1);
			    printf("Please enter the row2 col2:\n");
                scanf("%d %d", &row2, &col2);
            }
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            create_matrix(row2, col2, B);
            print_matrix(row2, col2, B);

            add_matrix(row1, col1, A, B, C);
            printf("Result = \n");
            print_matrix(row2, col2, C);
	}
	 else if (choice == 2) {
            printf("Please enter the row1 col1 :\n");
            scanf("%d %d", &row1, &col1);
			printf("Please enter the row2 col2:\n");
            scanf("%d %d", &row2, &col2);

            while (!(col1 == row2)) {
            	printf("!!!Your dimentiones does not match\n");
                printf("Please enter the row1 col1 :\n");
                scanf("%d %d", &row1, &col1);
			    printf("Please enter the row2 col2:\n");
                scanf("%d %d", &row2, &col2);
            }
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            create_matrix(row2, col2, B);
            print_matrix(row2, col2, B);
            
            multiply_matrix(row1, col1, col2, A, B, C);
            printf("Result = \n");
            print_matrix(row1, col2, C);
    }
    else if(choice==3){
            printf("Please enter the row1 col1:\n");
            scanf("%d %d", &row1, &col1);
            create_matrix(row1, col1, A);
            print_matrix(row1, col1, A);
            
            transpose_matrix(row1,col1, A , C);
            printf("Result = \n");
            print_matrix(col1, row1, C);

    }
    else if (choice==4) {
            printf("Please enter the row1 col1:\n");
            scanf("%d %d", &row1, &col1);
            while (!((row1 == 2 && col1 == 2) || (row1 == 3 && col1 == 3))) {
                printf("!!!Your dimentiones does not match\n\n");
                printf("please  again enter the row1 col1:\n");
                scanf(" %d%d", &row1, &col1);
            }
            if ((row1 == 2 && col1 == 2)) {
                create_matrix(row1, col1, A);
                print_matrix(row1, col1, A);
                
                determinant=  determinant_matrix(row1, col1, A);
                printf("Result = ");
                printf("%d\n",determinant);



            } else if ((row1 == 3 && col1 == 3)) {
                create_matrix(row1, col1, A);
                print_matrix(row1, col1, A);
                determinant_matrix(row1, col1, C);
                
                determinant=  determinant_matrix(row1, col1, A);
                printf("Result = ");
                printf("%d\n",determinant);

            }


        }
    else if(choice == 5){break;}
    }
	
	
	return 0;
}

void create_matrix(int r ,int c,int M[][S]){

    printf("\nWrite elements one by one\n:");
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
        	printf("(%d , %d) : ",i,j);
            scanf("%d", &M[i][j]);
        }
    }

}
void print_matrix(int r ,int c,int M[][S]) {
    printf("-------Matrix is:-------\n");
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            printf("%5d",M[i][j]);
        }
        printf("\n\n");
    }

}

void add_matrix(int r,int c,int A[][S],int B[][S],int C[][S]){
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            C[i][j] = A[i][j]+B[i][j];
        }

    }

}

void multiply_matrix(int r1,int c1, int c2, int A[][S],int B[][S],int C[][S]){
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



void transpose_matrix(int r,int c,int A[][S],int C[][S]) {
 	int i,j;
 	double tra[c][r];
 	
 	for(i=0;i<r;i++){
 		for(j=0;j<c;j++){
 			tra[j][i]=A[i][j];
		 }
	 }
	 for(i=0;i<c;i++){
	 	for(j=0;j<r;j++){
	 		printf("%f ",tra[i][j]);
		 }
	 	  printf("\n");
	 }
}

int determinant_matrix(int r,int c,int A[][S]){
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



