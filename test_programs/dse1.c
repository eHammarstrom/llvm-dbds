#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define BUF_SIZE (1024)

void f(int* a, int* b, int i)
{
  if (i > 0) {
    b[2] = 33; // removable if dup
  } else {
    b[0] = 10; // removable if dup
  }

  b[2] = 34;
  b[0] = 33;
}

int main()
{
  int *a;
  int *b;

  int i;
  scanf("%d", &i);

  a = malloc(BUF_SIZE * sizeof(int));
  b = malloc(BUF_SIZE * sizeof(int));

  a[2] = i;
  a[4] = i;

  f(a, b, i);

  printf("%d\n", b[2]); // 34
  printf("%d\n", a[4]); // 10
  printf("%d\n", b[0]); // 33

  return 0;
}
