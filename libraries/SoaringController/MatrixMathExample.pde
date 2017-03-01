/*
 *  MatrixMathExample.pde
 *  
 *	Example code for Matrix Math utilities
 *
 *  Created by Charlie Matlack on 12/18/10. Changed name.
 *  Original code by RobH45345 on Arduino Forums, taken from unknown source.
 *
 */

#include <MatrixMath.h>

#define N  (5)
MatrixMath math;

float A[N][N];
float B[N][N];
float C[N][N];
float max = 10;  // maximum random matrix entry range

void setup() {
	Serial.begin(9600); 
        // Initialize A to a random matrix and B to the identity
        for (int i = 0; i < N; i++)
        {
          for (int j = 0; j < N; j++)
          {
            A[i][j] = random(max) - max/2.0f;
            if (i == j)
            {
              B[i][j] = 1.0f;
            } else
            { 
              B[i][j] = 0.0f;
            }
          }
        }
        
}

void loop(){
	math.MatrixMult((float*)A,(float*)B,N,N,N,(float*)C);
        
        Serial.println("\nInitial values after C = A*B:");
	math.MatrixPrint((float*)A,N,N,"A");
	math.MatrixPrint((float*)B,N,N,"B");
	math.MatrixPrint((float*)C,N,N,"C");

        math.MatrixCopy((float*)A,N,N,(float*)B);
        
        Serial.println("\nCopied A to B:");
	math.MatrixPrint((float*)A,N,N,"A");
	math.MatrixPrint((float*)B,N,N,"B");
	math.MatrixPrint((float*)C,N,N,"C");        

        math.MatrixInvert((float*)A,N);

        Serial.println("\nInverted A:");
	math.MatrixPrint((float*)A,N,N,"A");
	math.MatrixPrint((float*)B,N,N,"B");
	math.MatrixPrint((float*)C,N,N,"C");        
       
        math.MatrixMult((float*)A,(float*)B,N,N,N,(float*)C);
 
        Serial.println("\nC = A*B");
	math.MatrixPrint((float*)A,N,N,"A");
	math.MatrixPrint((float*)B,N,N,"B");
	math.MatrixPrint((float*)C,N,N,"C");      	
while(1);
}
