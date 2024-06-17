#include <stdio.h>
#include <stdlib.h>
#define A(I,J) (*(A+(I)*nCol+(J)))
#define B(I,J) (*(B+(I)*nCol+(J)))

void matAdd(int nRow ,int nCol, double *A, double *B){
	int i,j;
	double result[nRow][nCol];
	for(i=0;i<nRow;i++){
		for(j=0;j<nCol;j++){
			result[i][j]= A(i,j) + B(i,j);
			printf("%f ",result[i][j]);
		}
		printf("\n");
	}
	
}
 void matMul(int nRow1, int nCol1, int nRow2, int nCol2,  double A[nRow1][nCol1], double B[nRow2][nCol2]) {
 	int i,j,k=0;
 	double sum=0.0;
 	double R[nRow1][nCol2];
 	
 	for(i=0;i<nRow1;i++){
 		for(j=0;j<nCol2;j++){
 			 sum=0.0;	
 			for(k=0;k<nRow2;k++){
 				sum=sum+A[i][k]*B[k][j];
 			    R[i][j]=sum;
			 }
			 printf("%f ",R[i][j]);	
		 }
		 printf("\n");	 		
   }
 	
 }
 void matTra(int nRow,int nCol,double A[nRow][nCol]) {
 	int i,j;
 	double tra[nCol][nRow];
 	
 	for(i=0;i<nRow;i++){
 		for(j=0;j<nCol;j++){
 			tra[j][i]=A[i][j];
		 }
	 }
	 for(i=0;i<nCol;i++){
	 	for(j=0;j<nRow;j++){
	 		printf("%f ",tra[i][j]);
		 }
	 	  printf("\n");
	 }
 	
 	
 }



#undef A
#undef B

int main() {
 int row,column,select,i,j;
 double A[2][3] ={{1,2,3},{4,5,6}};
 double B[2][3] ={{1,1,1},{1,1,1}};
 
 printf("Enter the row number:");
 scanf("%d",&row);
 printf("Enter the column number:");
 scanf("%d",&column);
 
 printf("please chose matrix operation you want to do\n");
 printf("1.Matrix addition\n2.Matrix multiplication\n3.Matrix transpose\n");
 scanf("%d",&select);
 
 switch(select){
 	case 1:
 		printf("Matrix addition selected:\n ");
 		printf("\n Create the content of the first operand matrix:\n");
 	/*	for(i=0;i<row;i++){
 			for(j=0;j<column;j++){
 				scanf("%lf",&A[i][j]);
			 }
			 printf("\n");
		 }
 		printf("\n Create the content of the second operand matrix:\n");
 		for(i=0;i<row;i++){
 			for(j=0;j<column;j++){
 				scanf("%lf",&B[i][j]);
			 }
			 printf("\n");
		 }*/
		 printf("Matrix addition:\n");
		 matAdd(row ,column, &A[0][0],&B[0][0]);
 		break;
 	case 2:
 		printf("Matrix multiplaction selected:\n ");
 		printf("Matrix multiplaction :\n");
 		matMul(2, 3, 2, 3, A,B);
 		 
 		break;
 	case 3:
 		printf("Matrix transpose selected:\n ");
 		printf("Matrix transpose:\n");
 		 matTra(2,3,A);
 		break;
 	default:
 		printf("WRONG KEY ENTER!!!\n");
 	    break;
 }
 
 
 /*
 
 matAdd(2 ,2, &A[0][0],&B[0][0]);
 printf("\n-----\n");
 matMul(2, 2, 2, 2, A,B);
 printf("\n-----\n");
 matTra(2,2,A);
*/	
	

}
