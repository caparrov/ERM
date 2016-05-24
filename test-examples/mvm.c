/*****************************************************************
 *
 * The following program computes a matrix multiply operation:
 *   C = X*Y,
 ******************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include "xmmintrin.h"
#include <math.h>


#define page 64


// ============ VCA: code for measuring performance counters ==================//

#define COLD_CACHE


void _ini1(double * m, size_t row, size_t col)
{
  size_t i;
  for (i = 0; i < row*col; ++i)  m[i] = (double)1.1;
}






void *malloc_aligned(size_t alignment, size_t bytes)
{
  // we need to allocate enough storage for the requested bytes, some
  // book-keeping (to store the location returned by malloc) and some extra
  // padding to allow us to find an aligned byte.  im not entirely sure if
  // 2 * alignment is enough here, its just a guess.
  const size_t total_size = bytes + (2 * alignment) + sizeof(size_t);
	
  // use malloc to allocate the memory.
  char *data = malloc(sizeof(char) * total_size);
	
  if (data)
  {
    // store the original start of the malloc'd data.
    const void * const data_start = data;
		
    // dedicate enough space to the book-keeping.
    data += sizeof(size_t);
		
    // find a memory location with correct alignment.  the alignment minus
    // the remainder of this mod operation is how many bytes forward we need
    // to move to find an aligned byte.
    const size_t offset = alignment - (((size_t)data) % alignment);
		
    // set data to the aligned memory.
    data += offset;
		
    // write the book-keeping.
    size_t *book_keeping = (size_t*)(data - sizeof(size_t));
    *book_keeping = (size_t)data_start;
  }
	
  return data;
}

void free_aligned(void *raw_data)
{
  if (raw_data)
  {
    char *data = raw_data;
		
    // we have to assume this memory was allocated with malloc_aligned.
    // this means the sizeof(size_t) bytes before data are the book-keeping
    // which points to the location we need to pass to free.
    data -= sizeof(size_t);
		
    // set data to the location stored in book-keeping.
    data = (char*)(*((size_t*)data));
		
    // free the memory.
    free(data);
  }
}



static __attribute__((noinline)) void
mvm (int n, double *A, double *x, double *y)
{
  int i;
  int j;
  double c;
  for (i = 0; i < n; i++)
  {
c = y[i];
    for (j = 0; j < n; j++)
    {
	      c += A[i * n + j] * x[j];
	    }
y[i]=c;
	 	}
  
}



int main(int argc, char *argv[])
{
  int n;
  int i, j;
  static double *A, *x, *y;
  
  
  //  Make sure a matrix size is specified
  if (argv[1] == NULL)
  {
    printf("USAGE: %s [side of matrix] [optional|size of block]\n", argv[0]);
    exit(1);
  }
  
  if ((n = atoi(argv[1])) < 0){
    exit(1);
  }
  
  A = (double *) _mm_malloc (n * n * sizeof (double), page);
  x = (double *) _mm_malloc (n * sizeof (double), page);
  y = (double *) _mm_malloc (n * sizeof (double), page);
  
  _ini1(A,n ,n);
  _ini1(x,n ,1);
  _ini1(y,n ,1);
  
  
  
#ifndef COLD_CACHE
for(i=0; i<2 ; i++)
#endif
  mvm(n,A,x,y);
  
  _mm_free(A);
  _mm_free(x);
  _mm_free(y);

  


  
  return 0;
}
