#include <iostream>
using namespace std;

#define INEXACT /* Nothing */

#define REAL double /* float or double */
#define REALPRINT doubleprint
#define REALRAND doublerand
#define NARROWRAND narrowdoublerand
#define UNIFORMRAND uniformdoublerand

int main() {
  REAL a = REALRAND;
  cout << a;
  return 0;
}