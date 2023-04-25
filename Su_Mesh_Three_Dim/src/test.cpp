#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

const double eps = 1e-8; // 精度控制参数
const double h = 1e-5;   // 步长控制参数

// 计算函数f(x,y)在点(x,y)处的二阶偏导数
double partial_second_derivative(double x, double y, double (*f)(double, double))
{
  double h1 = h, h2 = h;
  double fxy = f(x, y);
  double fxx = (f(x + h1, y) - 2 * fxy + f(x - h1, y)) / (h1 * h1);
  double fyy = (f(x, y + h2) - 2 * fxy + f(x, y - h2)) / (h2 * h2);
  double fxy1 = (f(x + h1, y + h2) - f(x + h1, y - h2) - f(x - h1, y + h2) + f(x - h1, y - h2)) / (4 * h1 * h2);
  double d = fabs(fxx * fyy - fxy1 * fxy1);
  while (d > eps)
  {
    h1 /= 2;
    h2 /= 2;
    fxx = (f(x + h1, y) - 2 * fxy + f(x - h1, y)) / (h1 * h1);
    fyy = (f(x, y + h2) - 2 * fxy + f(x, y - h2)) / (h2 * h2);
    fxy1 = (f(x + h1, y + h2) - f(x + h1, y - h2) - f(x - h1, y + h2) + f(x - h1, y - h2)) / (4 * h1 * h2);
    d = fabs(fxx * fyy - fxy1 * fxy1);
  }
  return fxx;
}

// 计算函数f(x,y,z)在点(x,y,z)处的二阶偏导数
double partial_second_derivative(double x, double y, double z, double (*f)(double, double, double))
{
  double h1 = h, h2 = h, h3 = h;
  double fxyz = f(x, y, z);
  double fxx = (f(x + h1, y, z) - 2 * fxyz + f(x - h1, y, z)) / (h1 * h1);
  double fyy = (f(x, y + h2, z) - 2 * fxyz + f(x, y - h2, z)) / (h2 * h2);
  double fzz = (f(x, y, z + h3) - 2 * fxyz + f(x, y, z - h3)) / (h3 * h3);
  double fxy1 = (f(x + h1, y + h2, z) - f(x + h1, y - h2, z) - f(x - h1, y + h2, z) + f(x - h1, y - h2, z)) / (4 * h1 * h2);
  double fxz1 = (f(x + h1, y, z + h3) - f(x + h1, y, z - h3) - f(x - h1, y, z + h3) + f(x - h1, y, z - h3)) / (4 * h1 * h3);
  double fyz1 = (f(x, y + h2, z + h3) - f(x, y + h2, z - h3) - f(x, y - h2, z + h3) + f(x, y - h2, z - h3)) / (4 * h2 * h3);
  double d = fabs(fxx * fyy * fzz - fxx * fyz1 * fyz1 - fyy * fxz1 * fxz1 - fzz * fxy1 * fxy1 + 2 * fxy1 * fxz1 * fyz1);
  while (d > eps)
  {
    h1 /= 2;
    h2 /= 2;
    h3 /= 2;
    fxx = (f(x + h1, y, z) - 2 * fxyz + f(x - h1, y, z)) / (h1 * h1);
    fyy = (f(x, y + h2, z) - 2 * fxyz + f(x, y - h2, z)) / (h2 * h2);
    fzz = (f(x, y, z + h3) - 2 * fxyz + f(x, y, z - h3)) / (h3 * h3);
    fxy1 = (f(x + h1, y + h2, z) - f(x + h1, y - h2, z) - f(x - h1, y + h2, z) + f(x - h1, y - h2, z)) / (4 * h1 * h2);
    fxz1 = (f(x + h1, y, z + h3) - f(x + h1, y, z - h3) - f(x - h1, y, z + h3) + f(x - h1, y, z - h3)) / (4 * h1 * h3);
    fyz1 = (f(x, y + h2, z + h3) - f(x, y + h2, z - h3) - f(x, y - h2, z + h3) + f(x, y - h2, z - h3)) / (4 * h2 * h3);
    d = fabs(fxx * fyy * fzz - fxx * fyz1 * fyz1 - fyy * fxz1 * fxz1 - fzz * fxy1 * fxy1 + 2 * fxy1 * fxz1 * fyz1);
  }
  return fxx;
}

// 二元函数示例
double f(double x, double y)
{
  return exp(x * y);
}

// 三元函数示例
double g(double x, double y, double z)
{
  return sin(x * y * z) / (x + y + z);
}

int main()
{
  double x = 1.0, y = 2.0;
  double xx = 1.0, yy = 2.0, zz = 3.0;
  double fxx = calc_second_partial_derivative(f, x, y);
  double fyy = calc_second_partial_derivative(f, x, y, 1e-6);
  double fxy = calc_mixed_partial_derivative(f, x, y);
  cout << "二元函数 f(x, y) = e^(x * y)" << endl;
  cout << "fxx = " << fxx << endl;
  cout << "fyy = " << fyy << endl;
  cout << "fxy = " << fxy << endl;

  double fxyz = calc_third_partial_derivative(g, xx, yy, zz);
  double fxxz = calc_mixed_partial_derivative(g, xx, yy, 1e-6, 0, zz, 1e-6);
  double fyzz = calc_second_partial_derivative(g, xx, yy, 1e-6, 1e-6, zz);
  double fxxy = calc_mixed_partial_derivative(g, xx, yy, 1e-6, xx, zz, 1e-6);
  cout << "三元函数 g(x, y, z) = sin(x * y * z) / (x + y + z)" << endl;
  cout << "fxyz = " << fxyz << endl;
  cout << "fxxz = " << fxxz << endl;
  cout << "fyzz = " << fyzz << endl;
  cout << "fxxy = " << fxxy << endl;

  return 0;
}
