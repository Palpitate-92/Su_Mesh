#include <iostream>

#define INEXACT
#define REAL double /* float or REAL */

#define Two_Sum_Tail(a, b, x, y) \
  bvirt = (REAL)(x - a);         \
  avirt = x - bvirt;             \
  bround = b - bvirt;            \
  around = a - avirt;            \
  y = around + bround

#define Two_Sum(a, b, x, y) \
  x = (REAL)(a + b);        \
  Two_Sum_Tail(a, b, x, y)

void program()
{
  INEXACT REAL bvirt;
  REAL avirt, bround, around;
  REAL a = -0.20906881559312751;
  REAL b = -0.20906881159312761;
  REAL x, y;
  Two_Sum(a, b, x, y);
  return;
}

int main()
{
  program();
  return 0;
}