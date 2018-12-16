#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define BUF_SIZE (1024)

void f(char* a, char* b, char* c, int i)
{
  if (i > 0) {
    memcpy(b, a, 6 * sizeof(char));
    b[5] = '\0';
    printf("%s\n", a);
  } else {
    memcpy(c, a, 4 * sizeof(char));
    b[5] = '\0';
  }

  memcpy(c, b, 6 * sizeof(char));
}

int main()
{
  char *a;
  char *b;
  char *c;

  int i;
  scanf("%d", &i);

  a = malloc(BUF_SIZE * sizeof(char));
  b = malloc(BUF_SIZE * sizeof(char));
  c = malloc(BUF_SIZE * sizeof(char));

  scanf("%s", a);
  printf("%s\n", a);

  f(a, b, c, i);

  printf("%s\n", b);
  printf("%s\n", a);
  printf("%s\n", c);

  return 0;
}
