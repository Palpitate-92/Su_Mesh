#include "Su_Mesh.h" //引用头文件
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>

// 读取输入文件
void ReadInFile(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int *borderNode_num, int *borderType, double *density_control, std::string infileName)
{
  std::fstream infile;                                     // 声明输入文件流
  std::string filePath = "MESH\\" + infileName;
  infile.open(filePath, std::ios::in);                     // 打开infileName，并且只能进行读取操作
  std::cout << "Reading from the file......" << std::endl; // 正在读取文件
  if (!infile.is_open())
  {                                                        // 判断文件是否存在或者是否成功打开
    std::cout << "error on open " << infileName
              << std::endl;                                // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else
  {
    std::cout << "Reading successfully!" << std::endl; // 读取文件成功
    infile >> *node_num;                               // 读入总节点数总节点数
    *borderNode_num = *node_num;                       // 储存总节点数目
    // 读入边界点类型
    do
      infile >> *borderType;
    while (*(borderType++) != -1);
    // 读入密度控制信息
    double tp1;
    do
    {
      infile >> tp1;
      *density_control++ = tp1;
    } while (tp1 != -1);
    double node_t[] = {
        0, 0, 0}; // 声明临时数组node_t用于接收输入数据，便于赋值到NODE类
    NODE node_tp; // 声明临时NODE类用于压入容器
    int tp;       // 声明用于临时存放值的变量
    for (int i = 0; i < *node_num; i++)
    {
      memset(node_t, 0, sizeof(node_t));             // 初始化node_t
      InitNode(&node_tp);                            // 初始化node_tp
      infile >> tp;
      infile >> node_t[0] >> node_t[1] >> node_t[2]; // 读入节点坐标
      for (int j = 0; j < DIM; j++)
        node_tp.tr_pos(j, node_t[j]);                // 节点坐标赋值到node_tp
      node_tp.tr_spac(node_t[2]);                    // 节点密度信息赋值到node_tp
      node->push_back(node_tp);                      // 压入容器node
    }
    infile >> *elem_num;                             // 读取输入网格单元数
    if (!*elem_num)                                  // 如果没有输入网格单元，关闭文件退出该程序
    {
      infile.close();
      return;
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
  outfile.open(filePath,
               std::ios::out |
                   std::ios::trunc); // 创建或者打开输出文件，如果此文件已经存在,
                                     // 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                  // 判断文件是是否成功打开
    std::cout << "error on open " << outfileName
              << std::endl;          // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else // 以tetview网格文件方式输出网格信息
  {
    // 输出节点信息
    outfile << *node_num << ' ' << DIM + 1 << ' ' << 0 << ' ' << 0 << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 使用迭代器遍历node，输出节点信息
    for (std::vector<NODE>::iterator node_iter = node->begin();
         node_iter != node->end();
         ++node_iter)
    {
      outfile << node_cnt << std::fixed << std::setprecision(11) << ' '
              << node_iter->get_pos(0) << ' ' << node_iter->get_pos(1);
      outfile << std::fixed << std::setprecision(1) << " 0 "
              << node_iter->get_spac() << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
    // 输出网格单元信息
    outfile << *elem_num << ' ' << 0 << '\n';
    int elem_NodeNum =
        3; // 定义网格单元节点数，显然Delaunay三角化生成的网格单元是三角形有3个节点
    // 使用迭代器遍历elem，输出网格单元信息
    for (std::vector<ELEM>::iterator elem_iter = elem->begin();
         elem_iter != elem->end();
         ++elem_iter)
      outfile << elem_NodeNum << ' ' << elem_iter->get_form(0) << ' '
              << elem_iter->get_form(1) << ' ' << elem_iter->get_form(2)
              << '\n';
    outfile << 0 << '\n'
            << 0;           // 以tetview网格文件方式输出网格信息
    std::cout << outfileName << " outing successfully !"
              << std::endl; // 文件输出成功
  }
  outfile.close();
  return;
}

// 生成边界边
void Gen_BorderEdge(int node_num, int *borderType, std::vector<EDGE> *border_edge, int *border_edge_num)
{
  int type;         // 声明一个变量，用于判断边界点形成图形的类型
  int border_num;   // 声明一个变量，用来储存type类型图形的边界点数量
  int node_cnt = 0; // 声明一个变量，用来储存当前总处理过的边界点数
  EDGE edge_tp;     // 声明一个用于临时储存生成边界边的edge类
  // 依据borderType内信息生成边界边
  do
  {
    type = *borderType++;
    border_num = *borderType++;
    // 如果是内部点，则跳过
    if (type == 5)
      continue;
    for (int i = node_cnt; i < border_num + node_cnt - 1; i++)
    {
      InitEdge(&edge_tp); // 初始化edge_tp
      edge_tp.tr_form(0, i);
      edge_tp.tr_form(1, i + 1);
      border_edge->push_back(edge_tp); // 将当前生成的边界边压入border_edge中，形成边界边生成
      *border_edge_num += 1;
    }
    InitEdge(&edge_tp);           // 初始化edge_tp
    edge_tp.tr_form(0, node_cnt); // 最后一个节点是与最前一个节点形成的边
    edge_tp.tr_form(1, border_num + node_cnt - 1);
    border_edge->push_back(
        edge_tp); // 将当前生成的边界边压入border_edge中，形成边界边生成
    *border_edge_num += 1;
    node_cnt += border_num;
  } while (*borderType != -1); //-1是*borderType的结尾
  return;
}

// 生成初始Delaunay三角化，插入初始Delaunay三角化四边形边框的四个顶角节点，输出该三角化信息文件Ini_Delaunay.smesh
void Output_IniDeDelaunay(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int *borderType, int DeFrame_numPos[])
{
  // 生成初始Delaunay三角化四边形边框，并插入四个顶角节点
  Gen_IniDeFrame(node, node_num, DeFrame_numPos);
  // 生成初始Delaunay三角化
  Gen_IniDeDelaunay(elem, elem_num, node, DeFrame_numPos);
  std::fstream outfile; // 声明输出文件流
  // 创建或者打开Ini_Delaunay.smesh，如果此文件已经存在,
  // 则打开文件之前把文件长度截断为0
  outfile.open("MESH\\Ini_Delaunay.smesh", std::ios::out | std::ios::trunc);
  if (!outfile.is_open())
  {                                                  // 判断文件是是否成功打开
    std::cout << "error on open Ini_Delaunay.smesh"; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  // 以tetview网格文件方式输出网格信息
  else
  {
    // 输出节点信息
    outfile << *node_num << ' ' << DIM + 1 << ' ' << 0 << ' ' << 0 << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 使用迭代器遍历node，输出节点信息
    for (std::vector<NODE>::iterator node_iter = node->begin();
         node_iter != node->end();
         ++node_iter)
    {
      outfile << node_cnt << ' ' << node_iter->get_pos(0) << ' '
              << node_iter->get_pos(1) << " 0" << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
    // 输出网格单元信息
    // 首先获取原始输入图形数量
    int cnt = 0;
    while (*(borderType + cnt) != -1)
      cnt++;
    cnt = cnt / 2;
    outfile << (*elem_num) + cnt << ' ' << 0 << '\n'; // 包含输入图形单元
    int type;                                         // 声明一个变量，用于判断边界点形成图形的类型
    int border_num;                                   // 声明一个变量，用来储存type类型图形的边界点数量
    int nodeDeal_cnt = 0;                             // 声明一个变量，用来储存当前总处理过的边界点数
    // 依据borderType内信息生成原始输入图形
    do
    {
      type = *borderType++;
      border_num = *borderType++;
      outfile << border_num; // 当前图形节点数
      for (int i = nodeDeal_cnt; i < border_num + nodeDeal_cnt; i++)
        outfile << ' ' << i;
      outfile << '\n';
      nodeDeal_cnt += border_num;
    } while (*borderType != -1); //-1是*borderType的结尾
    int elem_NodeNum =
        3;                       // 定义网格单元节点数，显然Delaunay三角化生成的网格单元是三角形有3个节点
    // 使用迭代器遍历elem，输出网格单元信息
    for (std::vector<ELEM>::iterator elem_iter = elem->begin();
         elem_iter != elem->end();
         ++elem_iter)
      outfile << elem_NodeNum << ' ' << elem_iter->get_form(0) << ' '
              << elem_iter->get_form(1) << ' ' << elem_iter->get_form(2)
              << '\n';
    outfile << 0 << '\n'; // 以tetview网格文件方式输出网格信息
    outfile << 0;
    std::cout << "Ini_Delaunay.smesh outing successfully!"
              << std::endl; // 文件输出成功
  }
  outfile.close();
  return;
}

// 边界点插入
void InsertBoundaryPoint(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int borderNode_num)
{
  ELEM elem_tp;                            // 声明一个用于临时储存生成网格单元的变量
  NODE node_Insert;                        // 声明一个用于临时储存待插入边界点的变量
  EDGE edge_tp;                            // 声明一个用于临时储存生成的边的变量
  for (int k = 0; k < borderNode_num; k++) // 一个个插入边界点
  {
    InitNode(&node_Insert);                // 初始化node_Insert
    node_Insert = node->at(k);             // 一个个插入边界点
    // 声明一个变量，用来储存任意一个外接圆包含待插入节点的网格单元编号，作为基单元来查找空腔
    // 为简化搜索流程，从包含上次插入的边界点的网格单元开始搜索
    int elemNum_IncludeNode = FindElemIncludeNode(
        elem, node, node_Insert, k == 0 ? 0 : node->at(k - 1).get_elem());
    // 创建一个空腔容器，用来储存外接圆包含待插入节点node_Insert的网格单元编号
    std::vector<int> elemNum_Cav;
    elemNum_Cav.push_back(
        elemNum_IncludeNode);                                                                           // 压入直接包含待插入节点的网格单元编号
    FindElemCav(elem, elem_num, node, &borderNode_num, &elemNum_Cav, node_Insert, elemNum_IncludeNode); // 空腔查找
    // RepairCavity(elem, elem_num, node, node_num, &elemNum_Cav,
    // node_Insert);//空腔修复
    std::vector<EDGE>
        edge_Cav; // 创建一个容器，用来储存空腔边界，一个边由两个节点构成
    /**
     * * 在edge_Cav内查找当前搜索的网格单元边界，如果搜到end()，代表没有找到，该边是空腔边，可以插入；
     * * 如果没有搜到搜到end()，代表该边不是空腔边，不予插入，并且在edge_Cav内删除该边
     */
    // 将所有空腔边界压入edge_Cav
    for (unsigned long long i = 0; i < elemNum_Cav.size(); i++)
    {
      InitEdge(&edge_tp);                 // 初始化edge_tp
      int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元
      // 定义一个变量，储存网格单元所有边，便于后续对边的访问
      int edge_Cavtp[3][2] = {
          {elem->at(elemNum_tp).get_form(0),
           elem->at(elemNum_tp).get_form(1)},  // 显然一个网格单元有三条边
          {elem->at(elemNum_tp).get_form(1),
           elem->at(elemNum_tp).get_form(2)},  // 取节点编号的时候从小到大排列
          {elem->at(elemNum_tp).get_form(0),
           elem->at(elemNum_tp).get_form(2)}}; // 便于后边各种查找
      // 一个网格单元有三条边
      for (int j = 0; j < 3; j++)
      {
        InitEdge(&edge_tp); // 初始化edge_tp
        edge_tp.tr_form(0, edge_Cavtp[j][0]);
        edge_tp.tr_form(1, edge_Cavtp[j][1]);
        std::vector<EDGE>::iterator
            edge_Iter; // 创建一个迭代器，便于删除非空腔边界边
        if ((edge_Iter = std::find(edge_Cav.begin(), edge_Cav.end(), edge_tp)) == edge_Cav.end())
          edge_Cav.push_back(edge_tp);
        else
          edge_Cav.erase(
              edge_Iter); // 显然，该边被插入过elemNum_edge，说明该边不是空腔边界边，删除边
      }
    }
    InitElemCav(
        elem,
        &elemNum_Cav); // 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
    // 对每个空腔边界进行操作，生成待插入点与当前空腔边界形成的网格单元，首先更新空腔边界边的相邻关系
    for (unsigned long long i = 0; i < edge_Cav.size(); i++)
    {
      // 前elemNum_Cav.size()个边与待插入点生成的网格单元编号采用空腔中删除的网格单元编号
      if (i < elemNum_Cav.size())
      {
        int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元编号
        // 显然可以直接给新生成的网格单元赋节点编号
        elem->at(elemNum_tp).tr_form(0, k);
        elem->at(elemNum_tp).tr_form(1, edge_Cav.at(i).get_form(0));
        elem->at(elemNum_tp).tr_form(2, edge_Cav.at(i).get_form(1));
        // 交换网格单元节点信息form数组，使其从小到大排列
        elem->at(elemNum_tp).Sort();
        // 改变该网格单元的三个节点的elem值，使其全部指向该单元
        node->at(k).tr_elem(elemNum_tp);
        node->at(edge_Cav.at(i).get_form(0)).tr_elem(elemNum_tp);
        node->at(edge_Cav.at(i).get_form(1)).tr_elem(elemNum_tp);
        Update_Djacency(elem, node, edge_Cav.at(i), "slow"); // 更新该边相邻关系
      }
      // 之后生成的网格全压入elem中，并赋予新的编号
      else
      {
        InitElem(&elem_tp); // 初始化elem_tp
        // 显然可以直接给新生成的网格单元赋节点编号
        elem_tp.tr_form(0, k);
        elem_tp.tr_form(1, edge_Cav.at(i).get_form(0));
        elem_tp.tr_form(2, edge_Cav.at(i).get_form(1));
        // 交换网格单元节点信息form数组，使其从小到大排列
        elem_tp.Sort();
        // 改变该网格单元的三个节点的elem值，使其全部指向该单元
        node->at(k).tr_elem(*elem_num);
        node->at(edge_Cav.at(i).get_form(0)).tr_elem(*elem_num);
        node->at(edge_Cav.at(i).get_form(1)).tr_elem(*elem_num);
        elem->push_back(elem_tp); // 将elem_tp1压入elem中，形成网格单元的生成
        *elem_num += 1;
        Update_Djacency(elem, node, edge_Cav.at(i),
                        "slow"); // 查找包含该边的所有网格单元，并更新相邻关系
      }
    }
    std::vector<int>
        elemNum_edgeJudge; // 创建一个容器，用来储存检查过的空腔边节点
    // 更新插入点与空腔边节点连接形成的边的相邻关系
    for (unsigned long long i = 0; i < edge_Cav.size(); i++)
    {
      // 注意，一个边有2个节点
      // 节点一
      if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(0)) == elemNum_edgeJudge.end())
      {
        // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
        InitEdge(&edge_tp);
        // 将待插入节点的编号与当前操作节点编号做比较，使其在数组内从小到大排列，便于检索边界
        edge_tp.tr_form(0, std::min(k, edge_Cav.at(i).get_form(0)));
        edge_tp.tr_form(1, std::max(k, edge_Cav.at(i).get_form(0)));
        Update_Djacency(elem, node, edge_tp,
                        "slow");         // 查找包含该边的所有网格单元，并更新相邻关系
        elemNum_edgeJudge.push_back(
            edge_Cav.at(i).get_form(0)); // 插入该节点编号到elemNum_edgeJudge
        if (elemNum_edgeJudge.size() ==
            edge_Cav
                .size()) // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
          break;
      }
      // 节点二
      if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(1)) == elemNum_edgeJudge.end())
      {
        // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
        InitEdge(&edge_tp);
        // 将待插入节点的编号与当前操作节点编号做比较，使其在数组内从小到大排列，便于检索边界
        edge_tp.tr_form(0, std::min(k, edge_Cav.at(i).get_form(1)));
        edge_tp.tr_form(1, std::max(k, edge_Cav.at(i).get_form(1)));
        Update_Djacency(elem, node, edge_tp,
                        "slow");         // 查找包含该边的所有网格单元，并更新相邻关系
        elemNum_edgeJudge.push_back(
            edge_Cav.at(i).get_form(1)); // 插入该节点编号到elemNum_edgeJudge
        if (elemNum_edgeJudge.size() ==
            edge_Cav
                .size()) // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
          break;
      }
    }
  }
  return;
}

// 搜索初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，并打上标识
void Search_DeFrame_Grid(std::vector<ELEM> *elem, int elem_num, int DeFrame_numPos[])
{
  for (std::vector<ELEM>::iterator elem_iter = elem->begin();
       elem_iter != elem->end();
       ++elem_iter)
  {
    // 由于初始Delaunay三角化方形边框的四个顶角节点标号总是最大，所以直接查找每个单元的最大节点标号即可
    if (elem_iter->get_form(2) == DeFrame_numPos[0] ||
        elem_iter->get_form(2) == DeFrame_numPos[1] ||
        elem_iter->get_form(2) == DeFrame_numPos[2] ||
        elem_iter->get_form(2) == DeFrame_numPos[3])
      elem_iter->tr_DeFrame_Grid(true);
  }
  return;
}

// 内部点生成和插入
void AutoRefine(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, double *density_control)
{
  std::vector<NODE> node_Insert; // 创建储存待插入节点的容器
  // 创建一个容器，用来储存生成插入点时直接包含待插入节点的网格单元编号，标号与node_Insert一一对应
  // 该容器作为后续网格变化后再次查找包含某插入节点的网格提供便利，能优化查找时间
  std::vector<int> elemNum_IncludeNodeInitial;
  std::vector<NODE> node_InsSucc; // 创建一个容器，用来储存成功插入的节点
  // int cnt = 1;
  do
  {
    std::vector<NODE>().swap(node_Insert); // 初始化node_Insert，并释放容器空间
    std::vector<int>().swap(
        elemNum_IncludeNodeInitial);       // 初始化elemNum_IncludeNodeInitial，并释放容器空间
    std::vector<NODE>().swap(
        node_InsSucc);                     // 初始化node_InsSucc，并释放容器空间
    // 内部点生成
    CreateFieldNodes(elem, elem_num, node, node_num, &node_Insert, density_control, &elemNum_IncludeNodeInitial);
    bool Sign;                // 判断节点是否成功插入
    if (!node_Insert.empty()) // 如果待插入节点容器不为空，则开始插入节点
    {
      for (unsigned long long i = 0; i < node_Insert.size(); i++)
      {
        // 声明一个变量，用来储存任意一个外接圆包含待插入节点的网格单元编号，作为基单元来查找空腔
        int elemNum_IncludeNodeIns;
        // 查找一个外接圆包含待插入节点的网格单元编号
        elemNum_IncludeNodeIns = FindElemIncludeNode(
            elem, node, node_Insert.at(i), elemNum_IncludeNodeInitial.at(i));
        // 如果该值为-1，说明程序运行错误
        if (elemNum_IncludeNodeIns == -1)
        {
          std::cout << "Error file output:" << std::endl;
          OutputFile(elem, elem_num, node, node_num, "Error_file");
          exit(-1);
        }
        // 插入该节点，并得到节点是否成功插入的信息
        Sign = InsertFieldPoint(elem, elem_num, node, node_num, node_Insert.at(i), elemNum_IncludeNodeIns);
        if (Sign)
        {
          node_InsSucc.push_back(node_Insert.at(
              i));            // 如果节点成功插入，则将成功插入的节点放入容器node_InsSucc
          node->push_back(node_Insert.at(
              i));            // 将成功插入的节点放入容器node中，形成节点插入
          node->at(*node_num).tr_elem(
              *elem_num - 1); // 最新生成的网格单元一定包含最新插入的节点
          *node_num += 1;     // 每插入一个节点记一次数
        }
      }
    }
    // std::string str = ".smesh";
    // str = std::to_string(cnt) + str;

    // OutputFile(elem, elem_num, node, node_num, "Boundary_Delaunay.smesh"); //
    // 输出网格信息文件 Final_Delaunay.smesh
    //  cnt++;
  } while (
      !node_InsSucc.empty()); // 如果该次循环成功插入了节点，则再执行一次循环
  return;
}

// 通过对角交换实现约束边界恢复
void Constrained_BoundaryRecovery(std::vector<ELEM> *elem,
                                  std::vector<NODE> *node,
                                  std::vector<EDGE> *border_edge,
                                  int *border_edge_num)
{
  std::vector<int>
      elemNum_IncludeEdge; // 声明一个容器，荣来储存包含*edge_iter边的网格单元编号
  // 检索每个边界边，看当前Delaunay三角化网格单元是否包含该边界边
  for (std::vector<EDGE>::iterator edge_iter = border_edge->begin(); edge_iter != border_edge->end(); ++edge_iter)
  {
    std::vector<int>().swap(elemNum_IncludeEdge);                    // 初始化elemNum_IncludeEdge，并释放容器空间
    FindShell(elem, node, *edge_iter, &elemNum_IncludeEdge, "fast"); // 查找包含*edge_iter边的网格单元编号
    // 如果elemNum_IncludeEdge容器大小为零，则代表当前查找的边界边不存在于当前三角化内，需要边界恢复
    if (elemNum_IncludeEdge.size() == 0)
    {
      std::cout << "The boundary formed by node " << edge_iter->get_form(0) << " and " << edge_iter->get_form(1) << " is not in the current triangulation!" << std::endl;
      std::cout << "Restoring boundary......" << std::endl;
      // 通过不断对角交换实现边界恢复
      Swap_BoundaryRecovery(elem, node, *edge_iter);
      std::cout << "Boundary restored successfully!" << std::endl;
    }
  }
  return;
}

// 根据内部边界信息，查找复连通域内外部网格单元
void Find_complex_connected_domains(std::vector<ELEM> *elem,
                                    int *elem_num,
                                    std::vector<EDGE> border_edge,
                                    int DeFrame_numPos[],
                                    std::vector<int> *elemNum_Remove)
{
  int *external_elem_judge = (int *)malloc((*elem_num) * sizeof(int));
  memset(external_elem_judge, 0, (*elem_num) * sizeof(int)); // 1代表非外部单元，-1代表外部单元,0代表未被判断
  int ini_elemNum;
  for (int i = 0; i < *elem_num; i++)
    if (elem->at(i).get_DeFrame_Grid())
    {
      ini_elemNum = i;
      break;
    }
  external_elem_judge[ini_elemNum] = -1;
  std::deque<int> elemNum_wait;
  std::vector<int> elemNum_succ;
  elemNum_wait.push_back(ini_elemNum);
  elemNum_succ.push_back(ini_elemNum);
  ELEM elem_tp;
  EDGE edge_tp;
  int elemNum_tp;
  int elemNum_tp_neig;
  while (elemNum_succ.size() < *elem_num)
  {
    elemNum_tp = elemNum_wait.front();
    elem_tp = elem->at(elemNum_tp);
    elemNum_wait.pop_front();
    for (int i = 0; i <= DIM; i++)
    {
      elemNum_tp_neig = elem_tp.get_neig(i);
      if (elemNum_tp_neig == -1)
        continue;
      if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elemNum_tp_neig) != elemNum_succ.end())
        continue;
      edge_tp = Node_Opposite_Face(elem_tp, elem_tp.get_form(i));
      if (std::find(border_edge.begin(), border_edge.end(), edge_tp) == border_edge.end())
        external_elem_judge[elemNum_tp_neig] = external_elem_judge[elemNum_tp];
      else
        external_elem_judge[elemNum_tp_neig] = -external_elem_judge[elemNum_tp];
      elemNum_wait.push_back(elemNum_tp_neig);
      elemNum_succ.push_back(elemNum_tp_neig);
    }
  }
  for (int i = 0; i < *elem_num; i++)
  {
    if (external_elem_judge[i] == -1)
      elemNum_Remove->push_back(i);
    if (external_elem_judge[i] == 0)
      std::cout << i << ' ';
  }
  free(external_elem_judge);
  return;
}

// 去掉外部单元并缩减容器
void Removal_ExGrid(std::vector<ELEM> *elem,
                    int *elem_num,
                    std::vector<NODE> *node,
                    int *node_num,
                    std::vector<EDGE> border_edge,
                    int DeFrame_numPos[],
                    int judge)
{
  int *nodeNum_Remove = NULL;      // 声明一个指针，用来指向储存待移除节点的连续内存空间
  std::vector<int> elemNum_Remove; // 声明一个容器，用来储存待移除网格单元编号
  // 此处nodeNum_Remove直接指向初始Delaunay三角化方形边框的四个顶角节点数组就行
  nodeNum_Remove = DeFrame_numPos;
  if (judge == 1)
    // 根据内部边界信息，查找复连通域内外部网格单元
    Find_complex_connected_domains(elem, elem_num, border_edge, DeFrame_numPos, &elemNum_Remove);
  if (judge == 2)
  {
    for (int i = 0; i < *elem_num; i++)
      if (elem->at(i).get_DeFrame_Grid())
        elemNum_Remove.push_back(i);
  }
  Removal_NodeElem(elem, elem_num, node, node_num, &elemNum_Remove, nodeNum_Remove); // 在node容器内删除节点与单元并更新所有信息
  return;
}

// 检查elem类中相邻信息的准确性
void Check_ElemAdjacency_accuracy(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  if (elem->size() == 0)
    return;
  if (elem->size() != *elem_num)
    std::cout << "elem_num is not true!" << std::endl;
  // 检索每个网格单元
  ELEM elem_tp;
  for (int i = 0; i < *elem_num; i++)
  {
    InitElem(&elem_tp);
    elem_tp = elem->at(i);
    if (!elem_tp.Sort_judge())
      std::cout << "Mesh node numbering order is wrong!\n";
    for (int j = 0; j < DIM + 1; j++)
    {
      if (elem_tp.get_neig(j) != -1)
      {
        if (elem_tp.get_neig(j) > *elem_num)
          std::cout << "the value of the elem_num is wrong" << std::endl;
        else if (!Elem_Adjacent(elem->at(elem_tp.get_neig(j)), elem->at(i)))
          std::cout << "The value of the " << i << " elem is wrong"
                    << std::endl;
      }
    }
  }
  return;
}

// 检查node类中elem的准确性
void Check_NodeElem_accuracy(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  if (node->size() != *node_num)
    std::cout << "node_num is not true!" << std::endl;
  // 检索每个节点，看每个节点的elem是否准确
  for (int i = 0; i < *node_num; i++)
  {
    int elem_Numtp =
        node->at(i)
            .get_elem(); // 拿出第i个节点的elem值，即第elem_Numtp个网格单元
    if (elem_Numtp >= *elem_num)
      std::cout << "the value of the elem_num  is wrong" << std::endl;
    else if (elem_Numtp == -1)
      continue;
    else if (elem->at(elem_Numtp).get_form(0) == i ||
             elem->at(elem_Numtp).get_form(1) == i ||
             elem->at(elem_Numtp).get_form(2) == i)
      continue;
    std::cout << "The value of the " << i << " node is wrong" << std::endl;
  }
  return;
}

// 计算各初始网格单元质量并保存
void Calculate_Shape_Quality(std::vector<ELEM> *elem, std::vector<NODE> *node)
{
  NODE node_tp1, node_tp2, node_tp3; // 记录当前判断网格的三个节点
  // 计算各初始网格单元质量
  for (std::vector<ELEM>::iterator elem_iter = elem->begin();
       elem_iter != elem->end();
       ++elem_iter)
  {
    // 取出三角形三个顶点NODE类
    node_tp1 = node->at(elem_iter->get_form(0));
    node_tp2 = node->at(elem_iter->get_form(1));
    node_tp3 = node->at(elem_iter->get_form(2));
    elem_iter->tr_quality(abs(
        Shape_Quality(node_tp1, node_tp2,
                      node_tp3))); // 此处不需要判断节点顺序，取绝对值，避免负值
  }
  return;
}

// 输出当前三角化的网格单元质量信息
void Quality_Information(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node)
{
  int low_quality_elem = 0;  // 定义一个变量，储存劣质网格数量
  double all_quality = 0;    // 定义一个变量，储存所有网格单元的质量，初始化为0
  double lowest_quality = 1; // 定义一个变量，储存网格单元的最小质量，初始化为1
  // 搜索各网格单元质量
  for (std::vector<ELEM>::iterator elem_iter = elem->begin();
       elem_iter != elem->end();
       ++elem_iter)
  {
    if (elem_iter->get_quality() < Minimum_Shape_Quality_Smoothing)
      low_quality_elem++;
    all_quality += elem_iter->get_quality();
    lowest_quality = elem_iter->get_quality() < lowest_quality
                         ? elem_iter->get_quality()
                         : lowest_quality;
  }
  std::cout << "The average quality of the current triangulation is "
            << all_quality / *elem_num << ".\n";
  std::cout << "The lowest quality of the current triangulation is "
            << lowest_quality << ".\n";
  std::cout << "The number of Bad Meshes is " << low_quality_elem << std::endl;
  return;
}

// 先判断每条非网格边界边长度，若比值超过Max_Ratio，则插点二分
void Dichotomize_Boundary(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int borderNode_num)
{
  int elem_num1;
  NODE node_tp;                   // 声明一个NODE类，储存临时生成节点
  std::vector<EDGE> AllElem_edge; // 声明一个容器，储存所有非网格边界边
  EDGE edge_tp;                   // 声明一个EDGE类，储存临时生成边
  double edge_length;             // 声明一个变量，储存边长度
  bool Sign;                      // 声明一个变量，判断节点是否成功插入
  // 遍历所有网格储存所有非网格边界边
  for (std::vector<ELEM>::iterator elem_iter = elem->begin();
       elem_iter != elem->end();
       ++elem_iter)
  {
    // 每个网格有三条边，每个for循环中当前处理的网格边是当前网格单元elem_iter->get_form(i)节点所对应的边
    for (int i = 0; i < DIM + 1; i++)
    {
      edge_tp = Node_Opposite_Face(
          *elem_iter, elem_iter->get_form(i)); // 得到节点对应单元边
      // 如果该边是边界边，则跳过
      if (edge_tp.get_form(0) < borderNode_num &&
          edge_tp.get_form(1) < borderNode_num)
        continue;
      if (std::find(AllElem_edge.begin(), AllElem_edge.end(), edge_tp) ==
          AllElem_edge.end())
        AllElem_edge.push_back(edge_tp);
    }
  }
  // 遍历所有边，插入节点
  for (std::vector<EDGE>::iterator edge_iter = AllElem_edge.begin();
       edge_iter != AllElem_edge.end();
       ++edge_iter)
  {
    edge_length =
        get_EdgeLength(node->at(edge_iter->get_form(0)),
                       node->at(edge_iter->get_form(1))); // 得到该边长度
    if (edge_length / node->at(edge_iter->get_form(0)).get_spac() <=
            Max_Ratio &&
        edge_length / node->at(edge_iter->get_form(1)).get_spac() <= Max_Ratio)
      continue;
    // 运行到这里说明应该在当前判断边中点加节点了
    node_tp =
        node->at(edge_iter->get_form(0)) + node->at(edge_iter->get_form(1));
    node_tp.tr_pos(0, node_tp.get_pos(0) / 2);
    node_tp.tr_pos(1, node_tp.get_pos(1) / 2);
    // 插入该节点，并得到节点是否成功插入的信息
    Sign =
        InsertFieldPoint(elem, elem_num, node, node_num, node_tp, *edge_iter);
    if (Sign)
    {
      node->push_back(node_tp); // 将成功插入的节点放入容器node中，形成节点插入
      node->at(*node_num).tr_elem(
          *elem_num - 1);       // 最新生成的网格单元一定包含最新插入的节点
      *node_num += 1;           // 每插入一个节点记一次数}
    }
  }
  return;
}

// 利用拓扑变换实现网格质量优化，此处具体实现方式是边交换技术
void Quality_Optimization_EdgeSwapping(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int borderNode_num)
{
  int elemNum_opp;           // 声明一个变量，储存待判断网格单元的相邻网格单元编号
  EDGE
      edge_tp;               // 声明一个EDGE类，储存待交换边，即两个网格单元对角交换后会删除的边
  NODE
      currentElem_node;      // 声明一个NODE类，储存当前判断单元中与待交换网格边相对的节点
  NODE
      oppElem_node;          // 声明一个NODE类，储存当前判断单元相邻网格单元中与待交换网格边相对的节点
  double
      shape_quality_current; // 声明一个变量，储存边交换前，两个三角形的总质量
  double
      shape_quality_after;   // 声明一个变量，储存边交换后，两个三角形的总低质量
  // 直接遍历每个网格，判断每个网格的三个相邻网格（最多是三个），当能提高共用一条内边的两个三角形的总质量，
  // 并且其中一个三角形的质量小于Minimum_Shape_Quality_Swap时，进行边交换实现局部重构
  for (int elemNum_iter = 0; elemNum_iter < *elem_num; elemNum_iter++)
  {
    // 每个网格单元最多有三个相邻网格
    for (int i = 0; i < DIM + 1; i++)
    {
      if (elem->at(elemNum_iter).get_neig(i) ==
          -1) // 如果当前判断边没有相邻单元，则进入下一次for循环
        continue;
      elemNum_opp = elem->at(elemNum_iter)
                        .get_neig(i); // 得到当前判断单元的相邻网格单元编号
      // 得到当前判断单元与elem_tp_opp单元的相邻共用边，即待交换边
      edge_tp = Node_Opposite_Face(elem->at(elemNum_iter),
                                   elem->at(elemNum_iter).get_form(i));
      // 如果待交换边是边界边，则跳过
      if (edge_tp.get_form(0) < borderNode_num &&
          edge_tp.get_form(1) < borderNode_num)
        continue;
      currentElem_node = node->at(
          elem->at(elemNum_iter)
              .get_form(i)); // 得到当前判断单元中与待交换网格边相对的节点
      // 得到当前判断单元相邻网格单元中与待交换网格边相对的节点
      oppElem_node = node->at(
          elem->at(elemNum_opp)
              .get_form(Face_Opposite_Node(elem->at(elemNum_opp), edge_tp)));
      shape_quality_current =
          elem->at(elemNum_iter).get_quality() +
          elem->at(elemNum_opp)
              .get_quality(); // 得到边交换前，两个三角形的总质量
      // 得到边交换后，两个三角形的总质量
      shape_quality_after = abs(Shape_Quality(node->at(edge_tp.get_form(0)),
                                              currentElem_node,
                                              oppElem_node)) +
                            abs(Shape_Quality(node->at(edge_tp.get_form(1)),
                                              currentElem_node,
                                              oppElem_node));
      // 如果待交换边edge_tp交换后，两个三角形的总质量提高，并且其中一个三角形的质量小于Minimum_Shape_Quality_Swap时，进行边交换操作，并更新网格质量信息
      if (shape_quality_after > shape_quality_current &&
          (shape_quality_current < Minimum_Shape_Quality_Swap ||
           shape_quality_after < Minimum_Shape_Quality_Swap))
      {
        // 首先判断该两个网格单元组成的四边形是否是凹四边形，由于凹四边形的对角线必定不相交，所以只需要判断该四边形对角线是否相交，若相交则禁止此次对角交换
        if (Concave_Quadrilateral(elem, elemNum_iter, elemNum_opp, node))
          continue;
        TwoElem_DiagonalSwap(elem, node, elemNum_iter, elemNum_opp);
        elem->at(elemNum_iter)
            .tr_quality(abs(
                Shape_Quality(node->at(elem->at(elemNum_iter).get_form(0)),
                              node->at(elem->at(elemNum_iter).get_form(1)),
                              node->at(elem->at(elemNum_iter).get_form(2)))));
        elem->at(elemNum_opp)
            .tr_quality(abs(
                Shape_Quality(node->at(elem->at(elemNum_opp).get_form(0)),
                              node->at(elem->at(elemNum_opp).get_form(1)),
                              node->at(elem->at(elemNum_opp).get_form(2)))));
      }
    }
  }
  return;
}

// 利用节点光顺实现网格质量优化
void Quality_Optimization_Smoothing(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int borderNode_num)
{
  std::vector<int> Init_inferior_elem; // 声明一个容器，储存初始劣质单元编号
  // 遍历elem容器，储存初始劣质单元编号
  for (int i = 0; i < *elem_num; i++)
    if (elem->at(i).get_quality() < Minimum_Shape_Quality_Smoothing)
      Init_inferior_elem.push_back(i);
  // 构造局部优化域，用待优化节点编号来代表每个优化域，具体优化域则是当前三角化中包含节点的所有网格单元
  std::vector<int>
      Laplacian_Smoothing_nodeNum; // 声明一个容器，储存需要进行Laplacian光顺的节点编号
  Construct_Local_Optimization_Domain_All(elem, node, borderNode_num, Init_inferior_elem, &Laplacian_Smoothing_nodeNum);
  //  进行智能Laplacian光顺对每一个节点的位置进行调整
  for (std::vector<int>::iterator iter = Laplacian_Smoothing_nodeNum.begin();
       iter != Laplacian_Smoothing_nodeNum.end();
       ++iter)
    Laplacian_Smoothing(elem, node, *iter);
  // 遍历Laplacian_Smoothing_nodeNum容器（即优化后的所有节点），判断每个节点的ball内是否全是高质量单元
  // 若依然包含劣质单元，则插入Optimization_based_Smoothing_nodeNum容器，进行Optimization_based光顺
  std::vector<int>
      Optimization_based_Smoothing_nodeNum; // 声明一个容器，储存需要进行Optimization_based光顺的节点编号
  std::vector<int>
      elemNum_IncludeNode;                  // 声明一个容器，用来储存包含待判断节点的所有网格单元编号
  for (std::vector<int>::iterator nodeNum_iter =
           Laplacian_Smoothing_nodeNum.begin();
       nodeNum_iter != Laplacian_Smoothing_nodeNum.end();
       ++nodeNum_iter)
  {
    std::vector<int>().swap(
        elemNum_IncludeNode); // 初始化elemNum_IncludeNode，并释放容器空间
    FindBall_fast(elem, node, *nodeNum_iter, &elemNum_IncludeNode);
    // 遍历elemNum_IncludeNode，若包含劣质网格则将当前判断节点插入Optimization_based_Smoothing_nodeNum容器
    for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin();
         elemNum_iter != elemNum_IncludeNode.end();
         ++elemNum_iter)
      if (elem->at(*elemNum_iter).get_quality() <
          Minimum_Shape_Quality_Smoothing)
      {
        Optimization_based_Smoothing_nodeNum.push_back(*nodeNum_iter);
        break;
      }
  }
  // 进行Optimization_based光顺对每一个节点的位置进行调整
  for (std::vector<int>::iterator iter =
           Optimization_based_Smoothing_nodeNum.begin();
       iter != Optimization_based_Smoothing_nodeNum.end();
       ++iter)
    Optimization_based_Smoothing_The_all(elem, node, *iter);
  return;
}

int main()
{
  std::vector<ELEM> elem;                    // 创建ELEM类型容器
  int elem_num = -1;                         // 定义网格单元数，并初始化为-1，-1为未输入网格单元
  std::vector<NODE> node;                    // 创建NODE类型容器
  int node_num = -1;                         // 定义总网格节点数，并初始化为-1，-1为未输入网格节点
  int borderNode_num;                        // 定义总边界点数
  int *borderType = new int[100];            // 定义边界点类型指针，并分配一个储存空间，用来储存边界点类型，具体信息查阅ComplexMesh_gen.cpp文件
  double *density_control = new double[100]; // 定义密度控制信息指针，并分配一个储存空间，用来储存密度控制信息，具体信息查阅ComplexMesh_gen.cpp文件
  std::vector<EDGE> border_edge;             // 创建edge类型容器，用来储存输入图形边界边
  int border_edge_num = 0;                   // 定义边界边数，并初始化为0，0为未生成边界边
  // * 读取输入文件
  ReadInFile(&elem, &elem_num, &node, &node_num, &borderNode_num, borderType, density_control, "circle_point.smesh");
  // 生成边界边
  Gen_BorderEdge(node_num, borderType, &border_edge, &border_edge_num);
  // 初始Delaunay三角化方形边框以4倍于输入图形初始方形边框的面积，对称包含输入图形初始方形边框
  // 定义初始Delaunay三角化方形边框的四个顶角节点在node容器内的位置
  int DeFrame_numPos[] = {-1, -1, -1, -1};
  // 生成初始Delaunay三角化，插入初始Delaunay三角化四边形边框的四个顶角节点，输出该三角化信息文件
  // Ini_Delaunay.smesh
  Output_IniDeDelaunay(&elem, &elem_num, &node, &node_num, borderType, DeFrame_numPos);
  // * 边界点插入
  InsertBoundaryPoint(&elem, &elem_num, &node, borderNode_num);
  // 输出边界点插入后网格信息文件 Boundary_Delaunay.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Boundary_Delaunay.smesh");
  //  搜索初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，并打上标识
  Search_DeFrame_Grid(&elem, elem_num, DeFrame_numPos);
  // * 内部点生成和插入
  AutoRefine(&elem, &elem_num, &node, &node_num, density_control);
  // 输出包含初始Delaunay三角化方形边框的网格信息文件 Frame_Delaunay.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Frame_Delaunay.smesh");
  // * 通过对角交换实现约束边界恢复
  Constrained_BoundaryRecovery(&elem, &node, &border_edge, &border_edge_num);
  // InsertSteinerPoints(&elem, &elem_num, &node, &node_num, &border_edge,&border_edge_num); // 插入Steiner点实现保形边界恢复
  // 去掉外部单元
  Removal_ExGrid(&elem, &elem_num, &node, &node_num, border_edge, DeFrame_numPos, 2);
  // 检查elem类中相邻信息的准确性
  Check_ElemAdjacency_accuracy(&elem, &elem_num, &node, &node_num);
  // 检查node类中elem的准确性
  Check_NodeElem_accuracy(&elem, &elem_num, &node, &node_num);
  // 网格优化前，首先计算各初始网格单元质量
  Calculate_Shape_Quality(&elem, &node);
  // 输出网格质量优化前信息文件 Before_Quality.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Before_Quality.smesh");
  // 输出优化前三角化的网格单元质量信息
  std::cout << "Output information about the quality of mesh elements triangulated before optimization:\n";
  Quality_Information(&elem, &elem_num, &node);
  for (int i = 0; i < 3; i++)
  {
    //  * 网格质量优化
    // 先后利用拓扑变换实现网格质量优化
    Quality_Optimization_EdgeSwapping(&elem, &elem_num, &node, &node_num, borderNode_num);
    // std::string str = std::to_string(i) + "_swap.smesh";
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
    // 再利用节点光顺实现网格质量优化
    Quality_Optimization_Smoothing(&elem, &elem_num, &node, &node_num, borderNode_num);
    // 先判断每条非网格边界边长度，若比值超过Max_Ratio，则插点二分
    // if (i == 0)
    //   Dichotomize_Boundary(&elem, &elem_num, &node, &node_num,
    //   borderNode_num);
    // str = std::to_string(i) + "_smoothing.smesh";
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
    //  输出优化后三角化的网格单元质量信息
    // std::cout << "Output optimized triangulated mesh element quality
    // information:\n"; Quality_Information(&elem, &elem_num, &node);
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
  }
  // 输出优化后三角化的网格单元质量信息
  std::cout << "Output optimized triangulated mesh element quality information:\n";
  Quality_Information(&elem, &elem_num, &node);
  // 输出网格质量优化后信息文件 Square_Delaunay.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Final_Delaunay.smesh");
  delete[] borderType;
  delete[] density_control;
  return 0;
}

/**
 * ctrl + k 然后 ctrl + 0 将大括号收缩
 * * std::vector<int>().swap(elemNum_IncludeNode);          //
 * 初始化elemNum_IncludeNode，并释放容器空间
 * * for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter !=
 * elem->end();++elem_iter)
 * * if (std::find(elemNum_wait.begin(), elemNum_wait.end(),
 * elem->at(elemNum_tp).get_neig(i)) == elemNum_wait.end())
 * * for (std::vector<EDGE>::iterator edge_iter = border_edge->begin();
 * edge_iter != border_edge->end(); ++edge_iter)
 */