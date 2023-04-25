#include <iostream>
#include <cmath>

using namespace std;

double f(double x)
{
  return sin(x); // 定义被求导函数
}

double df(double x, double h)
{
  return (f(x + h) - f(x - h)) / (2.0 * h); // 定义导数计算公式
}

double adaptive_df(double x, double h, double eps)
{
  double delta = 1.0;
  double df_old = df(x, h);
  while (delta > eps)
  {
    h *= 0.5;
    double df_new = df(x, h);
    delta = abs(df_old - df_new);
    df_old = df_new;
  }
  return df_old;
}

int main()
{
  double x = 1.0;                         // 求导点
  double h = 0.1;                         // 初始步长
  double eps = 1e-6;                      // 容忍误差
  double df_val = adaptive_df(x, h, eps); // 自适应精度求导
  cout << "f'(x) = " << df_val << endl;
  return 0;
}
