#include <iostream>
#include <fstream>

constexpr auto DIM = 2;                                   // 声明维度DIM为常值
constexpr auto node_perface = 8;                          // 每条边平分为SquareNode_perface份
constexpr auto SquareFaceLong = 100.0;                    // 正方形外边界长度
constexpr auto step_size = SquareFaceLong / node_perface; // 每个节点前进步长
constexpr auto node_num = node_perface * 4;               // 正方形边界总节点数
// ? 正方形网格单元边一般理想长度为正方形边界每个节点间距的0.8倍
constexpr auto Square_spac = step_size * 0.92;

int main()
{
  int node_cnt = 0;         // 节点编号计数
  double node[node_num][3]; // 节点变量
  for (int i = 0; i < node_perface; i++)
  {
    if (i == 0)
      node[node_cnt][0] = 0;
    else
      node[node_cnt][0] = node[node_cnt - 1][0] + step_size;
    node[node_cnt][1] = 0;
    node[node_cnt][2] = 0;
    node_cnt++;
  }
  for (int i = 0; i < node_perface; i++)
  {
    if (i == 0)
      node[node_cnt][1] = 0;
    else
      node[node_cnt][1] = node[node_cnt - 1][1] + step_size;
    node[node_cnt][0] = SquareFaceLong;
    node[node_cnt][2] = 0;
    node_cnt++;
  }
  for (int i = 0; i < node_perface; i++)
  {
    if (i == 0)
      node[node_cnt][0] = SquareFaceLong;
    else
      node[node_cnt][0] = node[node_cnt - 1][0] - step_size;
    node[node_cnt][1] = SquareFaceLong;
    node[node_cnt][2] = 0;
    node_cnt++;
  }
  for (int i = 0; i < node_perface; i++)
  {
    if (i == 0)
      node[node_cnt][1] = SquareFaceLong;
    else
      node[node_cnt][1] = node[node_cnt - 1][1] - step_size;
    node[node_cnt][0] = 0;
    node[node_cnt][2] = 0;
    node_cnt++;
  }
  std::fstream outfile;                                      // 声明输出文件流
  outfile.open("in.smesh", std::ios::out | std::ios::trunc); // 创建或者打开out.smesh，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                        // 判断文件是是否成功打开
    std::cout << "error on open in.smesh"; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else // 以tetview网格文件方式输出网格信息
  {
    //* 输出节点信息
    outfile << node_num << ' ' << 1 << ' ' << node_num << ' ' << -1 << '\n';
    outfile << 1 << ' ' << Square_spac << ' ' << -1 << '\n';
    node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    for (int i = 0; i < node_num; i++)
    {
      outfile << node_cnt << ' ' << node[node_cnt][0] << ' ' << node[node_cnt][1] << ' ' << Square_spac << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
  }
  outfile << 0;
  outfile.close();
  return 0;
}