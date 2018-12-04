#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define BUF_SIZE (1024)

void f(int *a, int *b, int i)
{
  printf("%d\n", a[2]); // undef

  if (i > 0) {
    a[8] = i;
    a[4] = 33333;
  } else {
    a[4] = 99999; // pass Early CSE
    b[3] = 22222; // SHOULD NOT BE REMOVED
    a[4] = i;
  }
}

/*
 * i = 0 does not work with current dbds
 */

int main()
{
  int *a;
  int *b;

  int i;
  scanf("%d", &i); // 10

  a = malloc(BUF_SIZE * sizeof(int));
  b = malloc(BUF_SIZE * sizeof(int));


  memcpy(b, a, 20 * sizeof(int));

  f(a,b,i);

  printf("%d\n", a[4]); // 33333 or i
  printf("%d\n", a[5]); // undef (0)
  printf("%d\n", a[8]); // i or undef (0)
  printf("%d\n", b[8]); // undef (0)
  printf("%d\n", b[3]); // undef (0) or 22222

  return 0;
}
