#include <stdio.h>
#include <assert.h>
#include <cuda.h>

#include "test.h"

// these macros are really really helpful
#  define CUDA_SAFE_CALL( call) {                                            \
    cudaError err = call;                                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",        \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        exit(EXIT_FAILURE);                                                  \
    } }

#define CHECKLASTERROR   {                                                 \
	cudaError_t err = cudaGetLastError();                                    \
	if (err != cudaSuccess) {                                                \
		fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",        \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        exit(EXIT_FAILURE);                                                  \
	} }


#define SIZ 128

__global__ void truc(unsigned int * buf) {
	if (threadIdx.x < SIZ) {
		buf[threadIdx.x] = buf[threadIdx.x] << 5;
	}
	__syncthreads();
}

int testcuda()
{
	unsigned int* foo = (unsigned int*) malloc(SIZ * sizeof(unsigned int));
	for (int x = 0; x < SIZ; ++x) {
		foo[x] = 1;
	}

	unsigned int * recf = NULL;
	CUDA_SAFE_CALL( cudaMalloc((void **) &recf, SIZ * sizeof(unsigned int)) );
	CUDA_SAFE_CALL(cudaMemcpy(recf, foo,  SIZ * sizeof(unsigned int), cudaMemcpyHostToDevice));
	truc<<<1, SIZ>>>(recf);
	CHECKLASTERROR
	CUDA_SAFE_CALL(cudaMemcpy(foo, recf, SIZ * sizeof(unsigned int), cudaMemcpyDeviceToHost));
	printf("2^5 -> %u\n", foo[5]);

	CUDA_SAFE_CALL(cudaFree(recf));

/*
int deviceCount;
cudaGetDeviceCount(&deviceCount);
printf("device count %d\n", deviceCount);

cudaDeviceProp dP;
cudaGetDeviceProperties(&dP, NULL);
//printf("Max threads per block: %d\n", dP.maxThreadsPerBlock);
//printf("Max Threads DIM: %d x %d x %d\n", dP.maxThreadsDim[0], dP.maxThreadsDim[1], dP.maxThreadsDim[2]);
//printf("Max Grid Size: %d x %d x %d\n", dP.maxGridSize[0], dP.maxGridSize[1], dP.maxGridSize[2]);

cudaDeviceProp* pDeviceProp = &dP;

     printf( "\nDevice Name \t - %s ", pDeviceProp->name );  
     printf( "\n**************************************");  
     printf( "\nTotal Global Memory\t\t -%d KB", pDeviceProp->totalGlobalMem/1024 );  
     printf( "\nShared memory available per block \t - %d KB", pDeviceProp->sharedMemPerBlock/1024 );  
     printf( "\nNumber of registers per thread block \t - %d", pDeviceProp->regsPerBlock );  
     printf( "\nWarp size in threads \t - %d", pDeviceProp->warpSize );  
     printf( "\nMemory Pitch \t - %d bytes", pDeviceProp->memPitch );  
     printf( "\nMaximum threads per block \t - %d", pDeviceProp->maxThreadsPerBlock );  
     printf( "\nMaximum Thread Dimension (block) \t - %d %d %d", pDeviceProp->maxThreadsDim[0], pDeviceProp->maxThreadsDim[1], pDeviceProp->maxThreadsDim[2] );  
     printf( "\nMaximum Thread Dimension (grid) \t - %d %d %d", pDeviceProp->maxGridSize[0], pDeviceProp->maxGridSize[1], pDeviceProp->maxGridSize[2] );  
     printf( "\nTotal constant memory \t - %d bytes", pDeviceProp->totalConstMem );  
     printf( "\nCUDA ver \t - %d.%d", pDeviceProp->major, pDeviceProp->minor );  
     printf( "\nClock rate \t - %d KHz", pDeviceProp->clockRate );  
     printf( "\nTexture Alignment \t - %d bytes", pDeviceProp->textureAlignment );  
     printf( "\nDevice Overlap \t - %s", pDeviceProp-> deviceOverlap?"Allowed":"Not Allowed" );  
     printf( "\nNumber of Multi processors \t - %d\n", pDeviceProp->multiProcessorCount );  
*/

	return 0;
}

