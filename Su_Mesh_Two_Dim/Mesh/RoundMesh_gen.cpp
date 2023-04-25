#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>

constexpr auto Pi = 3.14159265358979;              // 声明圆周率
constexpr auto center_x = 0.5;                     // 声明圆心X坐标
constexpr auto center_y = 0.5;                     // 声明圆心Y坐标
constexpr auto radius = 0.5;                       // 声明圆半径
constexpr auto DIM = 2;                            // 声明维度DIM为常值
constexpr auto node_perface = 20;                  // 每条边平分为20份
constexpr auto node_num = node_perface * 4;        // 每个边生成20个点
constexpr auto angle_step = Pi / 2 / node_perface; // 声明每一步前进的弧度

int main()
{
  int node_cnt = 0; // 节点编号计数
  double node_size = node_perface;
  double node[node_num][3]; // 节点变量
  // 从(0.5,0)开始生成节点
  for (int i = node_cnt; i < node_perface * 1; i++)
  {
    node[i][0] = center_x + sin(i * angle_step) / 2;
    node[i][1] = center_y - cos(i * angle_step) / 2;
    node[i][2] = 0;
    node_cnt++;
  }
  // 从(1,0.5)开始生成节点
  for (int i = node_cnt; i < node_perface * 2; i++)
  {
    node[i][0] = center_x + sin(i * angle_step) / 2;
    node[i][1] = center_y - cos(i * angle_step) / 2;
    node[i][2] = 0;
    node_cnt++;
  }
  // 从(0.5,1)开始生成节点
  for (int i = node_cnt; i < node_perface * 3; i++)
  {
    node[i][0] = center_x + sin(i * angle_step) / 2;
    node[i][1] = center_y - cos(i * angle_step) / 2;
    node[i][2] = 0;
    node_cnt++;
  }
  // 从(0,0.5)开始生成节点
  for (int i = node_cnt; i < node_perface * 4; i++)
  {
    node[i][0] = center_x + sin(i * angle_step) / 2;
    node[i][1] = center_y - cos(i * angle_step) / 2;
    node[i][2] = 0;
    node_cnt++;
  }
  std::fstream outfile;                                         // 声明输出文件流
  outfile.open("round.smesh", std::ios::out | std::ios::trunc); // 创建或者打开out.smesh，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                        // 判断文件是是否成功打开
    std::cout << "error on open in.smesh"; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else // 以tetview网格文件方式输出网格信息
  {
    //* 输出节点信息
    outfile << node_num << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    for (int i = 0; i < node_num; i++)
    {
      outfile << node_cnt << ' ' << node[node_cnt][0] << ' ' << node[node_cnt][1] << ' ' << node[node_cnt][2] << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
  }
  outfile << 0;
  outfile.close();
  return 0;
}