#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define BUF_SIZE (1024)

void f(int *a, int *b, int i) {
  if (i > 0) {
    memcpy(b, a, 6 * sizeof(int));
    printf("%d\n", b[2]); // 10
  } else {
    memcpy(b, a, 8 * sizeof(int));
    memcpy(b, a, 8 * sizeof(int));
  }

  memcpy(b, a, 10 * sizeof(int));
  printf("Expensive");
  printf("%d\n", b[8]); // 10
  printf("%d\n", b[4]); // 10
  printf("%d\n", b[4]); // 10
  printf("Duplication");
}

int main() {
  int *a;
  int *b;

  int i;
  scanf("%d", &i);

  a = malloc(BUF_SIZE * sizeof(int));
  b = malloc(BUF_SIZE * sizeof(int));

  a[2] = i;
  a[4] = i;
  a[8] = i;

  f(a, b, i);

  printf("%d\n", b[8]); // 10
  printf("%d\n", a[4]); // 10
  printf("%d\n", b[4]); // 10

  return 0;
}
