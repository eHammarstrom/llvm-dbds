#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "put.h"
#include "get.h"

#define BUF_SIZE (1024)

void f(int* a, int* b, int i)
{
  if (i > 0) {
    memcpy(a, b, 4 * sizeof(int));
  } else {
    memcpy(a, b, 8 * sizeof(int));
    memcpy(a, b, 8 * sizeof(int)); // fully redundant memcpy // OPTIMIZED
  }

  memcpy(a, b, 8 * sizeof(int)); // partially redundant memcpy // NOT OPTIMIZED
}

int main()
{
  int *a;
  int *b;

  int i = get();

  a = malloc(BUF_SIZE * sizeof(int));
  b = malloc(BUF_SIZE * sizeof(int));

  f(a, b, i);

  printf("%c\n", b[2]);
  printf("%c\n", a[4]);

  return 0;
}
