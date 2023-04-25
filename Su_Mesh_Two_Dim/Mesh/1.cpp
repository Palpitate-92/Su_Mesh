/**
 * * 本程序是Su_Mesh配套的初始网格边界点生成程序，会输出一个文件，包括边界点个数、坐标、密度控制信息，以及边界点形成图形的类型
 * * 一般包括4种类型，用数字1、2、3、4来表示
 * * 1：矩形（一般为正方形）
 * * 2：圆形
 * * 3：多边形
 * * 3：直线（一般是位于上述几种图形内部，拥有单独的密度控制信息，作为线加密来使用）
 * * 输出文件第一行分别表示：总节点数，图形种类，图形节点数，图形种类，图形节点数。。。。依次重复，最后会以-1作为本行结尾，并且第一个图形总是输入图形总外边界
 * * 输出文件第二行开始分别按照第一行给出的图形种类顺序给定密度控制信息
 * * 由于第一个图形总是输入图形总外边界，所以第二行只会给出一个数字，该数字表示全局密度控制信息（即除开之后几行限定的区域，其余节点的spac都用该信息）
 * * 所有密度信息结束后重开一行，以-1作为密度信息结尾
 * * 正常内部边界图形的周围节点密度控制信息如下标定：
 * * 矩形：1 x y xL yL Ld Hd xO yO。图形种类(1)，矩形形心坐标(x,y)，矩形x、y方向上长度(xL,yL)，低密度信息Ld，高密度信息Hd。
 * *     矩形内部一般从矩形边界由高密度Hd递增到低密度Ld，形心处为最低密度Ld。
 * *     矩形外部则由形心坐标(x,y)、矩形x、y方向上长度(xO,yO)标定一个外部矩形包围原矩形，且密度由原矩形高密度Hd过渡到外部矩形低密度Ld
 * * 圆形：2 x y r Ld Hd rO。图形种类(2)，圆心坐标(x,y)，圆形半径(r)，低密度信息Ld，高密度信息Hd。圆形内部一般从圆形边界由高密度Hd递增到低密度Ld，圆心为最低密度Ld。
 * *     圆心外部则由圆心坐标(x,y)、圆形半径(rO)标定一个外部圆形包围原圆形，且密度由原圆形高密度Hd过渡到外部圆形低密度Ld
 * * 输出文件之后所有行分别表示：当前节点编号，当前节点坐标（x，y），当前节点理想单元尺寸值
 */

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>

constexpr int k = 2;

// 正方形外边界
constexpr auto DIM = 2;                                         // 声明维度DIM为常值
constexpr auto SquareNode_perface = 20 * k;                     // 每条边平分为SquareNode_perface份
constexpr auto SquareFaceLong = 2.0 * k;                        // 正方形外边界长度
constexpr auto step_size = SquareFaceLong / SquareNode_perface; // 每个节点前进步长
constexpr auto Square_num = SquareNode_perface * 4;             // 正方形边界总节点数
// ? 正方形网格单元边一般理想长度为正方形边界每个节点间距的0.8倍
constexpr auto Square_spac = step_size * 0.8;

// 圆形内边界1
constexpr auto Pi = 3.14159265358979;                   // 声明圆周率
constexpr auto center_x = 0.5 * k;                      // 声明圆心X坐标
constexpr auto center_y = 0.5 * k;                      // 声明圆心Y坐标
constexpr auto radius = 0.3 * k;                        // 声明圆半径
constexpr auto RoundNode_perface = 15 * k;              // 每1/4个圆弧分为RoundNode_perface份
constexpr auto angle_step = Pi / 2 / RoundNode_perface; // 声明每一步前进的弧度
constexpr auto Round_num = RoundNode_perface * 4;       // 圆形边界总节点数
// ? 圆形网格单元边一般理想长度为圆形边界每个节点弧度间距的0.8倍
constexpr auto Round_spac = angle_step * radius * 0.8;
// ? 该图形为内边界，需要一些其他信息
constexpr auto OutRound_Radius = radius + 0.3 * k; // 外部圆形半径
constexpr auto Round_LDensity = Square_spac * 0.8; // 低密度信息

// 圆形内边界2
// constexpr auto Pi = 3.14159265358979;                   // 声明圆周率
constexpr auto center_x_2 = 1.5 * k;                        // 声明圆心X坐标
constexpr auto center_y_2 = 1.5 * k;                        // 声明圆心Y坐标
constexpr auto radius_2 = 0.2 * k;                          // 声明圆半径
constexpr auto RoundNode_perface_2 = 10;                    // 每1/4个圆弧分为RoundNode_perface份
constexpr auto angle_step_2 = Pi / 2 / RoundNode_perface_2; // 声明每一步前进的弧度
constexpr auto Round_num_2 = RoundNode_perface_2 * 4;       // 圆形边界总节点数
// ? 圆形网格单元边一般理想长度为圆形边界每个节点弧度间距的0.8倍
constexpr auto Round_spac_2 = angle_step_2 * radius_2 * 0.8;
// ? 该图形为内边界，需要一些其他信息
constexpr auto OutRound_Radius_2 = radius_2 + 0.2 * k; // 外部圆形半径
constexpr auto Round_LDensity_2 = Square_spac;     // 低密度信息

// 总节点数
constexpr auto node_num = Square_num + Round_num + Round_num_2; // 总节点数

// 生成正方形边界
void Gen_square(double square_node[][DIM])
{
  int square_cnt = 0; // 正方形节点编号计数
  // 从(0,0)开始生成节点
  for (int i = 0; i < SquareNode_perface; i++)
  {
    if (i == 0)
      square_node[square_cnt][0] = 0;
    else
      square_node[square_cnt][0] = square_node[square_cnt - 1][0] + SquareFaceLong / SquareNode_perface;
    square_node[square_cnt][1] = 0;
    square_cnt++;
  }
  // 从(SquareFaceLong,0)开始生成节点
  for (int i = 0; i < SquareNode_perface; i++)
  {
    if (i == 0)
      square_node[square_cnt][1] = 0;
    else
      square_node[square_cnt][1] = square_node[square_cnt - 1][1] + SquareFaceLong / SquareNode_perface;
    square_node[square_cnt][0] = SquareFaceLong;
    square_cnt++;
  }
  // 从(SquareFaceLong,SquareFaceLong)开始生成节点
  for (int i = 0; i < SquareNode_perface; i++)
  {
    if (i == 0)
      square_node[square_cnt][0] = SquareFaceLong;
    else
      square_node[square_cnt][0] = square_node[square_cnt - 1][0] - SquareFaceLong / SquareNode_perface;
    square_node[square_cnt][1] = SquareFaceLong;
    square_cnt++;
  }
  // 从(0,SquareFaceLong)开始生成节点
  for (int i = 0; i < SquareNode_perface; i++)
  {
    if (i == 0)
      square_node[square_cnt][1] = SquareFaceLong;
    else
      square_node[square_cnt][1] = square_node[square_cnt - 1][1] - SquareFaceLong / SquareNode_perface;
    square_node[square_cnt][0] = 0;
    square_cnt++;
  }
  return;
}

// 生成圆形内边界
void Gen_round(double round_node[][DIM], double x, double y, double r, int perface, double step)
{
  int round_cnt = 0; // 圆形节点编号计数
  //  从(x,y-r)开始生成节点
  for (int i = 0; i < perface; i++)
  {
    round_node[round_cnt][0] = x + sin(i * step) * r;
    round_node[round_cnt][1] = y - cos(i * step) * r;
    round_cnt++;
  }
  // 从(x+r,y)开始生成节点
  for (int i = 0; i < perface; i++)
  {
    round_node[round_cnt][0] = x + cos(i * step) * r;
    round_node[round_cnt][1] = y + sin(i * step) * r;
    round_cnt++;
  }
  // 从(x,y+r)开始生成节点
  for (int i = 0; i < perface; i++)
  {
    round_node[round_cnt][0] = x - sin(i * step) * r;
    round_node[round_cnt][1] = y + cos(i * step) * r;
    round_cnt++;
  }
  // 从(x-r,y)开始生成节点
  for (int i = 0; i < perface; i++)
  {
    round_node[round_cnt][0] = x - cos(i * step) * r;
    round_node[round_cnt][1] = y - sin(i * step) * r;
    round_cnt++;
  }
  return;
}

int main()
{
  double square_node[Square_num][DIM], round_node[Round_num][DIM], round_node_2[Round_num_2][DIM]; // 储存正方形、圆形节点变量
  // 生成正方形外边界
  Gen_square(square_node);
  // 生成圆形内边界
  Gen_round(round_node, center_x, center_y, radius, RoundNode_perface, angle_step);
  Gen_round(round_node_2, center_x_2, center_y_2, radius_2, RoundNode_perface_2, angle_step_2);
  std::fstream outfile;                                      // 声明输出文件流
  outfile.open("in.smesh", std::ios::out | std::ios::trunc); // 创建或者打开out.smesh，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                        // 判断文件是是否成功打开
    std::cout << "error on open in.smesh"; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  // 输出节点信息
  else
  {
    // 输出总控制信息
    outfile << node_num << ' ' << 1 << ' ' << Square_num << ' ' << 2 << ' ' << Round_num << ' '
            << 2 << ' ' << Round_num_2 << ' ' << -1 << '\n';
    // 输出密度控制信息
    outfile << 1 << ' ' << Square_spac << '\n';
    outfile << 2 << ' ' << center_x << ' ' << center_y << ' ' << radius << ' ' << Round_LDensity << ' '
            << Round_spac << ' ' << OutRound_Radius << '\n';
    outfile << 2 << ' ' << center_x_2 << ' ' << center_y_2 << ' ' << radius_2 << ' ' << Round_LDensity_2 << ' '
            << Round_spac_2 << ' ' << OutRound_Radius_2 << '\n';
    outfile << "-1\n";
    int cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 先输出正方形外边界
    for (int i = 0; i < Square_num; i++)
    {
      outfile << cnt << ' ' << square_node[i][0] << ' ' << square_node[i][1] << ' ' << Square_spac << '\n';
      cnt += 1; // 每成功输出一个节点，cnt++
    }
    // 输出圆形内边界1
    for (int i = 0; i < Round_num; i++)
    {
      outfile << cnt << ' ' << round_node[i][0] << ' ' << round_node[i][1] << ' ' << Round_spac << '\n';
      cnt += 1; // 每成功输出一个节点，cnt++
    }
    // 输出圆形内边界2
    for (int i = 0; i < Round_num_2; i++)
    {
      outfile << cnt << ' ' << round_node_2[i][0] << ' ' << round_node_2[i][1] << ' ' << Round_spac_2 << '\n';
      cnt += 1; // 每成功输出一个节点，cnt++
    }
  }
  outfile << 0; // 初始输入单元为0
  outfile.close();
  return 0;
}
