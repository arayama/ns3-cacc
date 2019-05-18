// C++ program to find adjoint and inverse of a matrix 
//#include<bits/stdc++.h> 
#include<iostream>
using namespace std; 

// Function to get cofactor of A[p][q] in temp[][]. n is current 
// dimension of A[][] 


void transpose(double **A,double**B,int r1, int c1)
{
	for(int  i=0;i<r1;i++)
		for(int j=0;j<c1;j++)
			B[i][j]=A[j][i];
	
}

void  crossmult(double **A,double **B,double **C,int r1,int c1, int r2, int c2) // C=A*B, C is r1 by c2 size
{
	if(c1!=r2)
	{
		cout<<"Error, matrix dimensions dont agree\n";
	}
	else {
		
	//double **temp;
	//**temp=**C; 	
	for(int i=0;i<r1;i++)
	 for(int j=0;j<c2;j++){
	 	C[i][j]=0;
		 for(int k=0;k<c1;k++)
			C[i][j]+= A[i][k]*B[k][j];
	}
//	**C=**temp;
	}
}

void sum(double **A,double **B,double **C,int r1,int c1) //C=A+B
{
	//double **temp=**C;
	for(int i=0;i<r1;i++)
	 for(int j=0;j<c1;j++)
		C[i][j]=A[i][j]+B[i][j];
	//**C=**temp;
}



void getCofactor(double **A, double **temp, int p, int q, int n) 
{ 
	int i = 0, j = 0; 

	// Looping for each element of the matrix 
	for (int row = 0; row < n; row++) 
	{ 
		for (int col = 0; col < n; col++) 
		{ 
			// Copying into temporary matrix only those element 
			// which are not in given row and column 
			if (row != p && col != q) 
			{ 
				temp[i][j++] = A[row][col]; 

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
n is current dimension of A[][]. */
double  determinant(double **A, int n) 
{ 
	double D = 0; // Initialize result 

	// Base case : if matrix contains single element 
	if (n == 1) 
		return A[0][0]; 

	double **temp; // To store cofactors 
	temp = new double* [n];
        for(int i=0;i<n;i++)
		temp[i]=new double[n];	




	int sign = 1; // To store sign multiplier 

	// Iterate for each element of first row 
	for (int f = 0; f < n; f++) 
	{ 
		// Getting Cofactor of A[0][f] 
		getCofactor(A, temp, 0, f, n); 
		D += sign * A[0][f] * determinant(temp, n - 1); 

		// terms are to be added with alternate sign 
		sign = -sign; 
	} 

	return D; 
} 

// Function to get adjoint of A[N][N] in adj[N][N]. 
void adjoint(double **A,double **adj,int n) 
{ 
	if (n == 1) 
	{ 
		adj[0][0] = 1; 
		return; 
	} 

	// temp is used to store cofactors of A[][] 
	int sign = 1; 
	double **temp;
	temp = new double* [n];
        for(int i=0;i<n;i++)
		temp[i]=new double[n];	

	for (int i=0; i<n; i++) 
	{ 
		for (int j=0; j<n; j++) 
		{ 
			// Get cofactor of A[i][j] 
			getCofactor(A, temp, i, j, n); 

			// sign of adj[j][i] positive if sum of row 
			// and column indexes is even. 
			sign = ((i+j)%2==0)? 1: -1; 

			// Interchanging rows and columns to get the 
			// transpose of the cofactor matrix 
			adj[j][i] = (sign)*(determinant(temp, n-1)); 
		} 
	} 
} 

// Function to calculate and store inverse, returns false if 
// matrix is singular 
bool inverse(double **A, double **inverse,int n) 
{ 
	// Find determinant of A[][] 
	double det = determinant(A, n);
        cout << "Determinant ==  "<<det<<endl;	
	if (det == 0) 
	{ 
		cout << "Singular matrix, can't find its inverse"; 
		return false; 
	} 

	// Find adjoint 
	double **adj;
	
	adj = new double* [n];
        for(int i=0;i<n;i++)
		adj[i]=new double[n];	

		
	
	adjoint(A, adj,n); 

	// Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
	for (int i=0; i<n; i++) 
		for (int j=0; j<n; j++) 
			inverse[i][j] = adj[i][j]/double(det); 

	return true; 
} 

// Generic function to display the matrix. We use it to display 
// both adjoin and inverse. adjoin is integer matrix and inverse 
// is a double. 
template<class T> 
void display(T **A,int n) 
{ 
	for (int i=0; i<n; i++) 
	{ 
		for (int j=0; j<n; j++) 
			cout << A[i][j] << " "; 
		cout << endl; 
	} 
} 

void creatediagonal(double **A,int n, double v)
{
	
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++)
			if(i==j)
				A[i][j]=v;
			else
				A[i][j]=0.0;
}


/// function for kalman filter application 

bool kalmanfilter(double **A,double **B, double **H, double **P, double **Q, double **R, double **x, double **z,double **u,int n)
{
	/* 
	x_=A*x+B*u; //x1, x2 x_
    P_=A*P*A'+Q; // ap, at, apat, p_
    K=P_*H'/(H*P_*H'+R); // p_ht, hp_ ht, hp_ht, k  
    x=x_+K*(z-H*x_); // hx_, z__hx_, kz__hx_
    P=(eye(2)-K*H)*P_; I, kh, I__kh  
	*/

	double **x1=new double*[n];
	double **x2=new double*[n];
	double **x_=new double*[n];
	double **ap=new double*[n];
	double **at=new double*[n];
	double **apat=new double*[n];
	double **p_=new double*[n];

	double **p_ht=new double*[n];
	double  **hp_ = new double*[n];
	double **ht=new double*[n];
	double **hp_ht=new double*[n];
	double **k=new double*[n];
	double **hx_=new double*[n];
	double **z__hx=new double*[n];
	double **kz__hx=new double*[n];
	double **at=new double*[n]; 


	for




}