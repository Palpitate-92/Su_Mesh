#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

const double EPS = 1e-8;       // 精度
const double DELTA = 1e-5;     // 步长
const double MAX_DELTA = 1e-2; // 最大步长
const int MAX_ITER = 100;      // 最大迭代次数

class Vector3
{ // 向量类
public:
  double x, y, z;
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  double norm() const { return sqrt(x * x + y * y + z * z); }
};

// 计算三元函数 f(x, y, z)
double f(const Vector3 &v)
{
  return exp(v.x * v.y * v.z) * sin(v.y + v.z) + v.x * v.x + v.y * v.y + v.z * v.z;
}

// 计算三元函数 f(x + dx, y + dy, z + dz) - f(x, y, z) / (dx * dy * dz)
double df(const Vector3 &v, const Vector3 &dv)
{
  Vector3 vp = Vector3(v.x + dv.x, v.y + dv.y, v.z + dv.z);
  return (f(vp) - f(v)) / (dv.x * dv.y * dv.z);
}

// 计算三元函数 f(x + dx, y + dy, z + dz) - 2*f(x, y, z) + f(x - dx, y - dy, z - dz) / (dx * dy * dz)
double ddf(const Vector3 &v, const Vector3 &dv)
{
  Vector3 vp1 = Vector3(v.x + dv.x, v.y + dv.y, v.z + dv.z);
  Vector3 vp2 = Vector3(v.x - dv.x, v.y - dv.y, v.z - dv.z);
  return (f(vp1) - 2 * f(v) + f(vp2)) / (dv.x * dv.y * dv.z);
}

// 使用自适应步长计算三元函数 f(x, y, z) 的二阶偏导数
double solve(const Vector3 &v)
{
  double delta = DELTA;
  double ddf1 = ddf(v, Vector3(delta, delta, delta));
  double ddf2 = ddf(v, Vector3(delta / 2, delta / 2, delta / 2));
  int iter = 0;
  while (abs(ddf1 - ddf2) > EPS && iter < MAX_ITER)
  { // 自适应步长
    delta /= 2;
    ddf1 = ddf(v, Vector3(delta, delta, delta));
    ddf2 = ddf(v, Vector3(delta / 2, delta / 2, delta / 2));
    if (delta < MAX_DELTA)
      break; // 步长过小
    iter++;
  }
  return ddf2;
}

int main()
{
  Vector3 v(1.0, 2.0, 3.0); // 自变量向量
  double ddf = solve(v);    // 计算二阶偏导数
  cout << "二阶偏导数: " << ddf << endl;
  return 0;
}
