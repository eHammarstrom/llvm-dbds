#include "stdlib.h"

#include "stdio.h"
#include "string.h"

#define BUF_SIZE (1024)

void f(int* a, int* b, int i)
{
  if (i > 0) {
    memcpy(b, a, 6 * sizeof(int)); // = [_, _, _, _, y, y, y, y, ...]

    // optimalB = [_, _, _, _, y, y, y, y, ...]
    // optimalA = [x, x, x, x, _, _, _, _, ...]
  } else {
    memcpy(b, a, 4 * sizeof(int)); // = [_, _, y, y, y, y, _, _, ...]
    b[3] = 'c';

    // optimalB = [_, _, y, y, y, y, _, _, ...]
    // optimalA = [x, x, _, _, _, _, _, _, ...]
  }

  b[3] = 'c';
  b[5] = 'b';
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

  printf("%d\n", b[2]); // 10
  printf("%d\n", a[4]); // 10

  return 0;
}
