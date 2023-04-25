/*
 * 以Su_Mesh生成的正方形网格为基准，生成六面体表面网格
 */

#include "Gen_hexahedron.h"

// 读取输入文件
void ReadInFile(std::vector<ELEM> *elem, int *elem_num, int *InitElem_num, std::vector<NODE> *node,
                int *node_num, int *InitNode_num, std::string infileName)
{
  std::fstream infile; // 声明输入文件流
  std::string filePath = "MESH\\" + infileName;
  infile.open(filePath, std::ios::in);                     // 打开infileName，并且只能进行读取操作
  std::cout << "Reading from the file......" << std::endl; // 正在读取文件
  if (!infile.is_open())
  {                                                           // 判断文件是否存在或者是否成功打开
    std::cout << "error on open " << infileName << std::endl; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else
  {
    std::cout << "Reading successfully!" << std::endl; // 读取文件成功
    infile >> *node_num;                               // 读入总节点数
    *InitNode_num = *node_num;                         // 储存初始节点数目
    // 读入密度控制信息
    double node_t[] = {0, 0, 0, 0}; // 声明临时数组node_t用于接收输入数据，便于赋值到NODE类
    NODE node_tp;                   // 声明临时NODE类用于压入容器
    int tp;                         // 声明用于临时存放值的变量
    infile >> tp >> tp >> tp;       // 过滤无关信息
    for (int i = 0; i < *node_num; i++)
    {
      memset(node_t, 0, sizeof(node_t)); // 初始化node_t
      InitNode(&node_tp);                // 初始化node_tp
      infile >> tp;
      infile >> node_t[0] >> node_t[1] >> node_t[2] >> node_t[3]; // 读入节点坐标，以及密度信息
      for (int j = 0; j < DIM + 1; j++)
        node_tp.tr_pos(j, node_t[j]); // 节点坐标赋值到node_tp
      node_tp.tr_spac(node_t[3]);     // 节点密度信息赋值到node_tp
      node->push_back(node_tp);       // 压入容器node
    }
    infile >> *elem_num;         // 读取输入网格单元数
    *InitElem_num = *elem_num;   // 储存初始网格单元数目
    double elem_t[] = {0, 0, 0}; // 声明临时数组elem_t用于接收输入数据，便于赋值到ELEM类
    ELEM elem_tp;                // 声明临时ELEM类用于压入容器
    infile >> tp;
    for (int i = 0; i < *elem_num; i++)
    {
      memset(elem_t, 0, sizeof(elem_t)); // 初始化elem_t
      InitElem(&elem_tp);                // 初始化elem_tp
      infile >> tp;
      infile >> elem_t[0] >> elem_t[1] >> elem_t[2]; // 读入网格单元节点编号
      for (int j = 0; j < DIM + 1; j++)
        elem_tp.tr_form(j, elem_t[j]); // 网格单元节点编号赋值到elem_tp
      elem->push_back(elem_tp);        // 压入容器elem
    }
  }
  infile.close();
  return;
}

// 输出生成网格文件
void OutputFile(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, std::string outfileName)
{
  std::fstream outfile; // 声明输出文件流
  std::string filePath = "MESH\\" + outfileName;
  outfile.open(filePath, std::ios::out | std::ios::trunc); // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                                            // 判断文件是是否成功打开
    std::cout << "error on open " << outfileName << std::endl; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else // 以tetview网格文件方式输出网格信息
  {
    // 输出节点信息
    outfile << *node_num << ' ' << DIM + 1 << ' ' << 0 << ' ' << 0 << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 使用迭代器遍历node，输出节点信息
    for (std::vector<NODE>::iterator node_iter = node->begin(); node_iter != node->end(); ++node_iter)
    {
      outfile << node_cnt << std::fixed << std::setprecision(11) << ' ' << node_iter->get_pos(0) << ' ' << node_iter->get_pos(1) << ' '
              << node_iter->get_pos(2);
      outfile << std::fixed << std::setprecision(1) << ' ' << node_iter->get_spac() << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
    // 输出网格单元信息
    outfile << *elem_num << ' ' << 0 << '\n';
    int elem_NodeNum = 3; // 定义网格单元节点数，显然Delaunay三角化生成的网格单元是三角形有3个节点
    // 使用迭代器遍历elem，输出网格单元信息
    for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
      outfile << elem_NodeNum << ' ' << elem_iter->get_form(0) << ' ' << elem_iter->get_form(1) << ' ' << elem_iter->get_form(2) << '\n';
    outfile << 0 << '\n'
            << 0;                                                      // 以tetview网格文件方式输出网格信息
    std::cout << outfileName << " outing successfully !" << std::endl; // 文件输出成功
  }
  outfile.close();
  return;
}

// 以当前正方形网格为基准，生成六面体表面网格
void Gen_hexahedron(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  // 首先拓展输入正方形网格
  //  当前网格作为底面，三维坐标系下是XY0面，首先生成“前”面，即X0Z面
  Gen_Face_X0Z(elem, elem_num, node, node_num);
  Gen_Face_XY1(elem, elem_num, node, node_num);
  Gen_Face_X1Z(elem, elem_num, node, node_num);
  Gen_Face_0YZ(elem, elem_num, node, node_num);
  Gen_Face_1YZ(elem, elem_num, node, node_num);
  return;
}

// 删除重复节点，修改表面网格信息
void Removal_Extra_Node(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  std::vector<int> nodeNum_Repetition;  // 声明一个容器，储存与当前判断节点位置相同的所有节点编号（除开当前节点）
  std::vector<int> elemNum_IncludeNode; // 声明一个容器，储存包含当前判断节点的所有网格单元编号
  // 遍历node容器
  for (int node_iter = 0; node_iter < *node_num; node_iter++)
  {
    std::vector<int>().swap(nodeNum_Repetition); // 初始化node_nodeNum_Repetition，并释放容器空间
    // 查找节点编号，如果没找到，说明当前节点是六面体六个面内节点而非六面体边框节点，跳过
    if (!Search_nodeRepetition(node, node_num, node_iter, &nodeNum_Repetition))
      continue;
    // 找到后，删除容器nodeNum_Repetition内所有节点，并将包含这些节点的网格单元的节点编号赋值为当前判断节点
    // 遍历nodeNum_Repetition容器
    for (std::vector<int>::iterator nodeNum_iter = nodeNum_Repetition.begin(); nodeNum_iter != nodeNum_Repetition.end(); ++nodeNum_iter)
    {
      std::vector<int>().swap(elemNum_IncludeNode); // 初始化node_nodeNum_Repetition，并释放容器空间
      FindBall_slow(elem, node, *nodeNum_iter, &elemNum_IncludeNode);
      // 遍历elemNum_IncludeNode容器，修改所有网格单元的节点编号
      for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
      {
        elem->at(*elemNum_iter).tr_form(ElemIncludeNode(elem->at(*elemNum_iter), *nodeNum_iter), node_iter);
        elem->at(*elemNum_iter).Sort();
      }
      // 由于要在node容器内删除当前判断节点，首先要将当前判断节点与node容器当前最后一个节点交换位置，再删除最后一个节点
      // 所有还需要修改包含node容器当前最后一个节点的所有网格单元的节点编号
      std::vector<int>().swap(elemNum_IncludeNode); // 初始化node_nodeNum_Repetition，并释放容器空间
      FindBall_slow(elem, node, *node_num - 1, &elemNum_IncludeNode);
      // 遍历elemNum_IncludeNode容器，修改所有网格单元的节点编号
      for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
      {
        elem->at(*elemNum_iter).tr_form(ElemIncludeNode(elem->at(*elemNum_iter), *node_num - 1), *nodeNum_iter);
        elem->at(*elemNum_iter).Sort();
      }
      // 交换位置
      std::swap(node->at(*nodeNum_iter), node->at(*node_num - 1));
      // 待删除节点永远是node容器内最后一个元素，直接删除就行
      node->pop_back();
      *node_num -= 1;
    }
  }
  return;
}

// 粗略检查当前节点和网格单元是否有误
void Check_elem_node(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  std::vector<int> elemNum_IncludeNode; // 声明一个容器，储存包含当前判断节点的所有网格单元编号
  // 遍历所有节点，判断是否有悬空节点，即不参与网格单元形成的节点
  for (int node_iter = 0; node_iter < *node_num; node_iter++)
  {
    std::vector<int>().swap(elemNum_IncludeNode); // 初始化elemNum_IncludeNode，并释放容器空间
    // 查找包含当前判断节点的所有网格单元
    FindBall_slow(elem, node, node_iter, &elemNum_IncludeNode);
    if (elemNum_IncludeNode.empty())
    {
      std::cout << "Node judge error!\n";
      exit(-1);
    }
  }
  // 遍历所有网格单元，判断网格单元是否包含node容器外的节点
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
    // 一个网格单元有三个节点
    for (int i = 0; i < DIM + 1; i++)
      if (elem_iter->get_form(i) >= *node_num)
      {
        std::cout << "Elem judge error!\n";
        exit(-1);
      }
  return;
}

int main()
{
  std::vector<ELEM> elem; // 创建ELEM类型容器
  int elem_num = -1;      // 定义网格单元数，并初始化为-1，-1为未输入网格单元
  int InitElem_num;       // 定义初始总网格单元数
  std::vector<NODE> node; // 创建NODE类型容器
  int node_num = -1;      // 定义总网格节点数，并初始化为-1，-1为未输入网格节点
  int InitNode_num;       // 定义初始总节点数
  // * 读取输入文件
  ReadInFile(&elem, &elem_num, &InitElem_num, &node, &node_num, &InitNode_num, "Square_Delaunay.smesh");
  Gen_hexahedron(&elem, &elem_num, &node, &node_num); // 以当前正方形网格为基准，生成六面体表面网格
  // 删除重复节点，修改表面网格信息
  Removal_Extra_Node(&elem, &elem_num, &node, &node_num);
  // 粗略检查当前节点和网格单元是否有误
  Check_elem_node(&elem, &elem_num, &node, &node_num);
  OutputFile(&elem, &elem_num, &node, &node_num, "Hexahedron_Delaunay.smesh");
  return 0;
}