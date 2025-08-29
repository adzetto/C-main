#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define dA(I,J) (*(dA + (I)*(4) + (J)))
#define dB(I,J) (*(dB + (I)*(3) + (J)))
#define A(I,J) (*(A + (I)*(4) + (J)))
#define B(I,J) (*(B + (I)*(3) + (J)))
#define dC(I,J) (*(dC + (I)*(4) + (J)))
#define C(I,J) (*(C + (I)*(4) + (J)))
#define D(I,J) (*(D + (I)*(4) + (J)))
#define E(I,J) (*(E + (I)*(4) + (J)))
#define dD(I,J) (*(dD + (I)*(4) + (J)))
#define dE(I,J) (*(dE + (I)*(4) + (J)))
#define F(I,J) (*(F + (I)*(4) + (J)))
#define dF(I,J) (*(dF + (I)*(4) + (J)))
#define G(I) (*(G + (I)))
#define TRUE 1
#define WORK(I,J) (*(work + (I)* (4) + (J)))
#define FALSE 0
#define dRV(I) (*(dRV + (I)))
double  minors(double *A,double *B,int r,int c){/* r kapattýðým satýr c kapattýðým sütun*/
	int i,j,x=0,y=0;
	double result=0.0;
	for(i=0; i<4; i++){
		if(i==r){
			continue;
		}
	
		for(j=0; j<4; j++){
		
		if(j==c){
			continue;
		}
		
		B(x,y)=A(i,j);
	/*	printf("x %d y %d %f",x,y,B(x,y)); */
 	y++;
	}
	y=y-3;
	x++;
	}
/*	printf("\n");*/
 /*
for(i=0; i<3;i++){
	for(j=0; j<3; j++){
		printf("%f\t",B(i,j));
	}
	printf("\n");
}*/
result=(B(0,0)*((B(1,1)*B(2,2)-B(2,1)*B(1,2)))-B(0,1)*(B(1,0)*B(2,2)-B(2,0)*B(1,2))+B(0,2)*(B(1,0)*B(2,1)-B(2,0)*B(1,1)) );
return result;

}
void cofactor(double *A,double *B,double *C){
	
	int i,j;
	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
			C(i,j)=pow(-1,i+j)*(minors(A,B,i,j));
	
		}
		
	}
	/*
	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
		printf("%f\t",C(i,j));
		}
		printf("\n");  
	}  
	*/
	} 
		double  adjoint(double *C, double *D){
	int i,j;
			 for( i=0;i<4;i++)
     {
      for( j=0;j<4;j++)
            {
                   D(i,j)=C(j,i);
               /*   printf("%f",D(i,j));*/
            }
      /*      printf("\n");*/
      }
		 	return *D;
}
double det_calculator(double *work,double *C){
	int i; float det=0;
	for(i=0; i<4; i++){
		det+=WORK(i,0)*C(i,0);
	}
	return det;
	
}
double  matrix_mult(double *F,double *G){
	int i,k,j;
	double array[4][1];
	double val;
	for(i=0; i<4; i++){
		val=0.0;
		for(k=0; k<4; k++){
			val += F(i,k)*G(k);
			
		}
		array[i][0]=val;
		printf("%f\n",val);
	}

		
	
	
}
#undef A
#undef WORK



int main(){
		double *dA,*dB,*dC,*dD,*dE,*dF;
		
		dA= (double*)malloc(4*4*sizeof(double));/* dA normal 3*3 matrix dB ise bunun minörü dolayýsýyla 2x2*/
		dB= (double*)malloc(3*3*sizeof(double));
		dC= (double*)malloc(4*4*sizeof(double));
	 	dD= (double*)malloc(4*4*sizeof(double));
		dE= (double*)malloc(4*4*sizeof(double));
			dF= (double*)malloc(4*4*sizeof(double));
		                                                    /*GÝRDÝÐÝM DEÐERLER ((1,2,1),(2,1,0),(3,-2,4));*/
		int i,j;
				printf("Content of the first operand matrix:\n");
 		for(i=0; i<4; i++){
 		for(j=0; j<4; j++){
 			scanf("%lf",&dA(i,j));
		 }
	 }
	 	  double *dRV;
	  dRV= (double*)malloc(1*4*sizeof(double));
	  	printf("Enter elements in vector of size 4x1: \n");
 		for(i=0; i<4; i++){
 		for(j=0; j<1; j++){
 			scanf("%lf",&dRV(i));
		 }
	 }
	 	cofactor(dA,dB,dC);
	 		adjoint(dC,dD);
	
	  int p1,q1;

     for( q1=0;q1<4;q1++)
     {
      for( p1=0;p1<4;p1++)
            {
                   
                  dF(p1,q1)=dD(q1,p1)/ det_calculator(dA,dC);;
            }
      }
      printf("Inverse of matrix:\n");
           for( q1=0;q1<4;q1++)
     {
      for( p1=0;p1<4;p1++)
            {
                   
                  printf("%f\t",dF(p1,q1));
            }
            printf("\n");
      }
      	for(i=0; i<4; i++){
		printf("%f",dRV(i));
	}
      printf("X Vector:\n ");
      matrix_mult(dF,dRV);

}
