#include <iostream>

#define REAL double /* float or REAL */

REAL f1(REAL x)
{
  return (x * x);
}

REAL first_dx(REAL (*fx)(REAL), REAL x)
{
  REAL h = 0.001;
  REAL value;

  value = (fx(x + h) - fx(x - h)) / (2 * h);
  return value;
}

REAL second_dx(REAL (*fx)(REAL), REAL x)
{
  REAL h = 0.001;
  REAL value;

  value = (fx(x - h) - 2 * fx(x) + fx(x + h)) / (h * h);
  return value;
}

int main()
{
  REAL x = 20000.0;
  printf("**** Function Pointers ****\r\n");
  printf("Value of f(%f): %f\r\n", x, f1(x));
  printf("First derivative: %f\r\n", first_dx(f1, x));
  printf("Second derivative: %f\r\n\r\n", second_dx(f1, x));
  return 0;
}