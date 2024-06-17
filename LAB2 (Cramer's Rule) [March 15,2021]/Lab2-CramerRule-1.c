#include <stdio.h>
#include <math.h>


/*
Algorithm: Matrix Inversion using Cramer's Rule

Inputs:
- A: Square matrix of size N x N

Outputs:
- Inverse of matrix A, if it exists

Procedure:
1. Define the size of the square matrix N and initialize matrices A, I, and Dxyz of size N x N.
2. Initialize a function Identity(I) to generate the identity matrix of size N x N.
3. Implement the CreateDxyz_mat(A, I, A_colNum, Dxyz, invA_colNum) function to create the matrix Dxyz for each column of A using Cramer's rule.
    a. Iterate over each column index A_colNum of matrix A.
    b. For each column index, iterate over each row of A:
        i. Set Dxyz[i][j] to I[i][invA_colNum] if j equals A_colNum, else set it to A[i][j].
    c. Display the Dxyz matrix.
    d. Calculate and print the determinant of Dxyz using the determinant() function.
4. Implement the CramersRuleForInverse(A) function to apply Cramer's rule for each column of A to find the inverse.
    a. Initialize the identity matrix I.
    b. Iterate over each column index j of matrix A.
        i. Iterate over each row index i of A:
            1. Generate the Dxyz matrix for the current column using CreateDxyz_mat() function.
            2. Use the determinant() function to calculate the determinant of Dxyz.
            3. Calculate the cofactor matrix for each element of Dxyz using the cofactor() function.
            4. Transpose the cofactor matrix to obtain the adjoint matrix.
            5. Divide each element of the adjoint matrix by the determinant of Dxyz to obtain the inverse of A.
    c. Display the inverse of A.
5. Implement the determinant() function to compute the determinant of a square matrix recursively.
    a. If the size of the matrix is 1, return the single element.
    b. Initialize the determinant D to 0.
    c. Iterate over each element of the first row.
        i. Calculate the cofactor of the current element.
        ii. Add the product of the current element, its cofactor, and the sign to the determinant.
    d. Return the determinant.
6. Implement the cofactor() function to compute the cofactor matrix of a given element in a matrix.
    a. Iterate over each element of the matrix.
        i. Create a temporary matrix excluding the current row and column.
        ii. Calculate the determinant of the temporary matrix.
        iii. Multiply the determinant by (-1)^(i+j) to determine the sign of the cofactor.
        iv. Assign the sign multiplied by the determinant to the corresponding element in the cofactor matrix.
7. Implement the transpose() function to transpose a matrix.
    a. Iterate over each element of the matrix.
        i. Swap the row and column indices of each element to obtain the transpose.
8. Display the inverse of matrix A.

End Algorithm.
*/

#define N 3
double Dxyz[N][N] = {0};
double A[N][N]= {0};
double I[N][N]= {0};
double determinant(double mat[N][N], int n);
void display(double mat[N][N], int row, int col);
void transpose(double num[N][N], double fac[N][N], float r);
double determinant(double mat[N][N], int n);
void Identity(double I[N][N])
{
  for (int i=0;i<N;i++){
    for(int j=0;j<N;j++){
      if(i==j)
        I[i][j] = 1.0;
      else
        I[i][j] = 0.0;
    }
  }
}
void CreateDxyz_mat(double A[N][N],double I[N][N],int A_colNum,double Dxyz[N][N],int invA_colNum) { // for col 1› invA_colNum=0, col 2› invA_colNum=1, col 3› invA_colNum=2
for (int i=0;i<N;i++){
  for(int j=0;j<N;j++){
    if (j==A_colNum)
      Dxyz[i][j]=I[i][invA_colNum];
    else
      Dxyz[i][j]=A[i][j];
  }
}
printf("\n");
display(Dxyz, N, N);
printf("\nDeterminant of the matrix is :  %lf",determinant(Dxyz,N));

}
void CramersRuleForInverse(double A[N][N]) {
Identity(I);
for(int j=0;j<N;j++) {
for(int i=0;i<N;i++){
  Identity(I);
  CreateDxyz_mat(A,I,i,Dxyz,i);


}}
}

// Function to get cofactor of mat[p][q] in temp[][]. n is
// current dimension of mat[][]
void getCofactor(double mat[N][N], double temp[N][N], int p,
                 int q, int n)
{
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            //  Copying into temporary matrix only those
            //  element which are not in given row and
            //  column
            if (row != p && col != q)
            {
                temp[i][j++] = mat[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of mat[][]. */
double determinant(double mat[N][N], int n)
{
    double D = 0; // Initialize result


    if (n == 1)
        return mat[0][0];

    double temp[N][N];

    int sign = 1;

    for (int f = 0; f < n; f++)
    {
        // Getting Cofactor of mat[0][f]
        getCofactor(mat, temp, 0, f, n);
        D += sign * mat[0][f]
             * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}
void cofactor(double num[N][N], int f)
{
 double b[N][N], fac[N][N];
 int p, q, m, n, i, j;
 for (q = 0;q < f; q++)
 {
   for (p = 0;p < f; p++)
    {
     m = 0;
     n = 0;
     for (i = 0;i < f; i++)
     {
       for (j = 0;j < f; j++)
        {
          if (i != q && j != p)
          {
            b[m][n] = num[i][j];
            if (n < (f - 2))
             n++;
            else
             {
               n = 0;
               m++;
               }
            }
        }
      }
      fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
    }
  }
  transpose(num, fac, f);
}
void transpose(double num[N][N], double fac[N][N], float r)
{
  int i, j;
  double b[N][N], inverse[N][N], d;

  for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
         b[i][j] = fac[j][i];
        }
    }
  d = determinant(num, r);
  for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
        inverse[i][j] = b[i][j] / d;
        }
    }
   printf("\n\n\nThe inverse of matrix is : \n");
  display(inverse, N, N);
   for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
         printf("\t%f", inverse[i][j]);
        }
    printf("\n");
     }
}

/* function for displaying the matrix */
void display(double mat[N][N], int row, int col)
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
            printf("  %lf", mat[i][j]);
        printf("\n");
    }
}

// Driver program to test above functions
int main()
{
    for(int i=0; i<N;i++) {
      for(int j=0; j<N;j++) {
      printf("Enter the element of matrix [%d][%d] ",i+1,j+1);
      scanf("%lf",&A[i][j]);
      }
    }
    double d = determinant(A, N);
    printf("A matricies");
    display(A, N, N);
    printf("\nDeterminant of the matrix is : %lf\n",d);

  CramersRuleForInverse(A);
    // Function call
  if (d == 0)
   printf("\nInverse of Entered Matrix is not possible\n");
  else
   cofactor(A, N);


}
