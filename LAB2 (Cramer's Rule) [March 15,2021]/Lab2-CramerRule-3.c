#include<stdio.h>
#include<conio.h>
#include <math.h>
#define MAX_DIM 100

void DECMPS(float D[100][100],float R[100][100], float A[100][100], int n);
void INV_DIAGONAL(float inv_D[100][100], float D[100][100], int n);
void MATSUBT(float out[100][1], float m1[100][1], float m2[100][1], int n);
void MATMULT(float out[100][1], float m1[100][100], float m2[100][1], int ROWS1, int COLS1, int ROWS2, int COLS2);


int main()
{
  float A[MAX_DIM][MAX_DIM],b[MAX_DIM][1],D[MAX_DIM][MAX_DIM],R[MAX_DIM][MAX_DIM],inv_D[MAX_DIM][MAX_DIM],subt[MAX_DIM][MAX_DIM],x[MAX_DIM][1];
  float Rx[MAX_DIM][1],b_Rx[MAX_DIM][1],Ax[MAX_DIM][1],Ax_b[MAX_DIM][1];
  int n,i,j,maxi,k,iter;
  float c,sum,abs_Ax_b;


  printf("\nEnter the number of equations:");
  scanf("%d",&n);

  for(i=0;i<n;i++){
                   for(j=0;j<n;j++){
                                    printf("Enter the element (%d, %d) of A matrix:",i+1,j+1);
                                    scanf("%f",&A[i][j]);
                                    }
                   }

  printf("\nThe A matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   for(j=0;j<n;j++){
                                    printf("%f\t",A[i][j]);
                                    }
                   }
  printf("\n");

  for(i=0;i<n;i++){
                   printf("Enter the element (%d) of b matrix:",i+1);
                   scanf("%f",&b[i][0]);
                   }

  printf("\nThe b matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   printf("%f",b[i][0]);
                   }


  DECMPS(D, R, A, n);  // Decomposition of the A matrix
  INV_DIAGONAL(inv_D, D, n);  //Inverse of the diagonal matrix D

  printf("\nThe D matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   for(j=0;j<n;j++){
                                    printf("%f\t",D[i][j]);
                                    }
                   }

  printf("\nThe R matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   for(j=0;j<n;j++){
                                    printf("%f\t",R[i][j]);
                                    }
                   }

  printf("\nThe inv_D matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   for(j=0;j<n;j++){
                                    printf("%f\t",inv_D[i][j]);
                                    }
                   }

  printf("\n");
  for(i=0;i<n;i++){
                   printf("Enter the element (%d) of initial x matrix:",i+1);
                   scanf("%f",&x[i][0]);
                   }
  printf("\nThe initial x matrix is\n");
  for(i=0;i<n;i++){
                   printf("\n");
                   printf("%f",x[i][0]);
                   }

  printf("\nEnter c :");
  scanf("%f",&c);

  printf("\nEnter maximum iteration :");
  scanf("%d",&maxi);

// x(k+1)=D^-1(b-Rx(k)) iteration

  for(i=0;i<maxi;i++){
                      MATMULT(Rx,R,x,n,n,n,1);
                      MATSUBT(b_Rx,b,Rx,n);
                      MATMULT(x,inv_D,b_Rx,n,n,n,1);

                      // convergence control abs(Ax-b)<c
                      sum=0.0;
                      MATMULT(Ax,A,x,n,n,n,1);
                      MATSUBT(Ax_b,Ax,b,n);
                      for(k=0;k<n;k++){
                                       sum=Ax_b[k][0]*Ax_b[k][0]+sum;
                                       }
                      abs_Ax_b=sqrt(sum);
                      if(abs_Ax_b<c){
                                     iter=i;
                                     i=maxi;
                                     }

                      }
                      printf("\nThe output at iteration %d with c=%f is\n",iter+1,c);
                      for(j=0;j<n;j++){
                                       printf("%f \n",x[j][0]);
                                       }

  getch();
  return 0;
}



void DECMPS(float D[100][100],float R[100][100], float A[100][100], int n)
{
int i,j;

for(i=0;i<n;i++){
                 for(j=0;j<n;j++){
                                  D[i][j]=0;
                                  R[i][j]=A[i][j];
                                  }
                 }

for(i=0;i<n;i++){
                 D[i][i]=A[i][i];
                 R[i][i]=0;
                 }
}

void INV_DIAGONAL(float inv_D[100][100], float D[100][100], int n)
{
int i;

for(i=0;i<n;i++){
                 inv_D[i][i]=1/D[i][i];
                 }
}

void MATSUBT(float out[100][1], float m1[100][1], float m2[100][1], int n)
{
int i,j;
for(i=0;i<n;i++){
                 for(j=0;j<n;j++){
                                  out[i][j]=m1[i][j]-m2[i][j];
                                 }
                 }
}

void MATMULT(float out[100][1], float m1[100][100], float m2[100][1], int ROWS1, int COLS1, int ROWS2, int COLS2)
{
int i,j,k;
float sum = 0;

   for(i=0;i<ROWS1;i++)
   		for(j=0;j<COLS2;j++)
           out[i][j]=0;

    for(i=0;i<ROWS1;i++){ //row of first matrix
      for(j=0;j<COLS2;j++){  //column of second matrix
           sum=0;
           for(k=0;k<COLS1;k++)
               sum=sum+m1[i][k]*m2[k][j];
         out[i][j]=sum;
		}
	}

}
