#include "Su_Mesh.h"

double _DATA_PROCESS::sign(double value)
{
    if (value > 0)
        return 1;
    else if (value < 0)
        return -1;
    else
        return 0;
}

double _DATA_PROCESS::get_dist(double A[3], double B[3])
{
    return double(sqrt(pow(A[0] - B[0], 2) + pow(A[1] - B[1], 2) + pow(A[2] - B[2], 2)));
}

NODE _DATA_PROCESS::center_of_gravity(NODE node_tp1, NODE node_tp2, NODE node_tp3, NODE node_tp4)
{
    NODE node_tp;
    double point_Coord[4][3] = {
        {node_tp1.pos[0], node_tp1.pos[1], node_tp1.pos[2]},
        {node_tp2.pos[0], node_tp2.pos[1], node_tp2.pos[2]},
        {node_tp3.pos[0], node_tp3.pos[1], node_tp3.pos[2]},
        {node_tp4.pos[0], node_tp4.pos[1], node_tp4.pos[2]},
    };
    node_tp.pos[0] = (point_Coord[0][0] + point_Coord[1][0] + point_Coord[2][0] + point_Coord[3][0]) / 4;
    node_tp.pos[1] = (point_Coord[0][1] + point_Coord[1][1] + point_Coord[2][1] + point_Coord[3][1]) / 4;
    node_tp.pos[2] = (point_Coord[0][2] + point_Coord[1][2] + point_Coord[2][2] + point_Coord[3][2]) / 4;
    return node_tp;
}

double _DATA_PROCESS::in_sphere(NODE node_tp1, NODE node_tp2, NODE node_tp3, NODE node_tp4, NODE node_tp)
{
    // 一个网格单元有四个节点
    double point_Coord[4][3] = {
        {node_tp1.pos[0], node_tp1.pos[1], node_tp1.pos[2]},
        {node_tp2.pos[0], node_tp2.pos[1], node_tp2.pos[2]},
        {node_tp3.pos[0], node_tp3.pos[1], node_tp3.pos[2]},
        {node_tp4.pos[0], node_tp4.pos[1], node_tp4.pos[2]},
    };
    double center_triangle[3]{};                                              // 定义一个数组，储存四面体重心
    double node_tp_pos[3] = {node_tp.pos[0], node_tp.pos[1], node_tp.pos[2]}; // 定义一个数组，储存待插入点坐标
    center_triangle[0] = (point_Coord[0][0] + point_Coord[1][0] + point_Coord[2][0] + point_Coord[3][0]) / 4;
    center_triangle[1] = (point_Coord[0][1] + point_Coord[1][1] + point_Coord[2][1] + point_Coord[3][1]) / 4;
    center_triangle[2] = (point_Coord[0][2] + point_Coord[1][2] + point_Coord[2][2] + point_Coord[3][2]) / 4;
    double judge;
    double value;
    _SHEWCHUK Shewchuk;
    judge = Shewchuk.insphere(point_Coord[0], point_Coord[1], point_Coord[2], point_Coord[3], center_triangle);
    // value = Shewchuk.insphere(point_Coord[0], point_Coord[1], point_Coord[2], point_Coord[3], node_tp_pos);
    // value = Shewchuk.insphere(point_Coord[0], point_Coord[2], point_Coord[1], point_Coord[3], node_tp_pos);
    if (judge > 0)
        value = Shewchuk.insphere(point_Coord[0], point_Coord[1], point_Coord[2], point_Coord[3], node_tp_pos);
    else
        value = Shewchuk.insphere(point_Coord[0], point_Coord[2], point_Coord[1], point_Coord[3], node_tp_pos);
    return value;
}

double _DATA_PROCESS::in_tetrahedron(NODE node_tetrahedron[4], NODE node_tp)
{
    _SHEWCHUK shewchuk;
    // 利用有向体积来判断待判断节点是否在四面体内部
    // 一个网格单元有四个面，首先储存这四个面，利用数组编号来标定网格面，数组每行前三个标定网格面，第四个是网格面节点顺序标定点
    int faceNum_tp[4][4] = {{0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 3, 1}, {1, 2, 3, 0}};
    int cnt = 0; // 记录四面体有向体积为正值的数量
    for (int i = 0; i < 4; i++)
    {
        // 修改faceNum_tp的节点顺序，使其正方向排序
        if (shewchuk.orient3d(node_tetrahedron[faceNum_tp[i][0]].pos, node_tetrahedron[faceNum_tp[i][1]].pos, node_tetrahedron[faceNum_tp[i][2]].pos, node_tetrahedron[faceNum_tp[i][3]].pos) < 0)
            std::swap(faceNum_tp[i][0], faceNum_tp[i][1]);
        // 当前网格面节点顺序确定好后，直接判定
        if (shewchuk.orient3d(node_tetrahedron[faceNum_tp[i][0]].pos, node_tetrahedron[faceNum_tp[i][1]].pos, node_tetrahedron[faceNum_tp[i][2]].pos, node_tp.pos) < 0)
            return -1;
        cnt++;
    }
    if (cnt == 4)
        return 1;
    return 0.0;
}

double _DATA_PROCESS::triangle_area(NODE node_A, NODE node_B, NODE node_C)
{
    double vector_AC[] = {node_A.pos[0] - node_C.pos[0], node_A.pos[1] - node_C.pos[1], node_A.pos[2] - node_C.pos[2]};
    double vector_BC[] = {node_B.pos[0] - node_C.pos[0], node_B.pos[1] - node_C.pos[1], node_B.pos[2] - node_C.pos[2]};
    double cross_product[] = {vector_AC[1] * vector_BC[2] - vector_AC[2] * vector_BC[1],
                              vector_AC[2] * vector_BC[0] - vector_AC[0] * vector_BC[2],
                              vector_AC[0] * vector_BC[1] - vector_AC[1] * vector_BC[0]};
    double area = sqrt(cross_product[0] * cross_product[0] + cross_product[1] * cross_product[1] + cross_product[2] * cross_product[2]);
    return area / 2;
}

bool _DATA_PROCESS::point_internal_triangle(NODE node_A, NODE node_B, NODE node_C, NODE point)
{
    double tri_all = triangle_area(node_A, node_B, node_C);
    double tri_1 = triangle_area(node_A, node_B, point);
    double tri_2 = triangle_area(node_A, node_C, point);
    double tri_3 = triangle_area(node_B, node_C, point);
    if (abs(tri_all - tri_1 - tri_2 - tri_3) <= Max_deviation_point_internal)
        return true;
    else
        return false;
}

double _DATA_PROCESS::tetrahedral_volume(NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
    double vector_AD[] = {node_A.pos[0] - node_D.pos[0], node_A.pos[1] - node_D.pos[1], node_A.pos[2] - node_D.pos[2]};
    double vector_BD[] = {node_B.pos[0] - node_D.pos[0], node_B.pos[1] - node_D.pos[1], node_B.pos[2] - node_D.pos[2]};
    double vector_CD[] = {node_C.pos[0] - node_D.pos[0], node_C.pos[1] - node_D.pos[1], node_C.pos[2] - node_D.pos[2]};
    double volume = vector_AD[0] * vector_BD[1] * vector_CD[2] +
                    vector_BD[0] * vector_CD[1] * vector_AD[2] +
                    vector_CD[0] * vector_AD[1] * vector_BD[2] -
                    vector_AD[0] * vector_CD[1] * vector_BD[2] -
                    vector_BD[0] * vector_AD[1] * vector_CD[2] -
                    vector_CD[0] * vector_BD[1] * vector_AD[2];
    return volume / 6;
}

bool _DATA_PROCESS::Edge_Edge_Intersection(Point *intersection_point, NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
    // 首先定义CGAL的顶点类
    Point_3 p1{node_A.pos[0], node_A.pos[1], node_A.pos[2]},
        p2{node_B.pos[0], node_B.pos[1], node_B.pos[2]},
        p3{node_C.pos[0], node_C.pos[1], node_C.pos[2]},
        p4{node_D.pos[0], node_D.pos[1], node_D.pos[2]};
    // 再定义CGAL的线段类
    Segment_3 seg1(p1, p2), seg2(p3, p4);
    // 计算相交结果
    const auto result = intersection(seg1, seg2);
    // 如果相交
    if (result)
    {
        // 保证相交图形是一个点
        if (const Point_3 *p = boost::get<Point_3>(&*result))
        {
            intersection_point->pos[0] = p->x();
            intersection_point->pos[1] = p->y();
            intersection_point->pos[2] = p->z();
            return true;
        }
        //else
        //{
        //    std::cout << "Edge-to-edge intersection judgment error!\n";
        //    system("pause");
        //}
    }
    return false;
}

bool _DATA_PROCESS::Edge_Face_Intersection(Point *intersection_point, NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E)
{
    // 首先定义CGAL的顶点类
    Point_3 p1{node_A.pos[0], node_A.pos[1], node_A.pos[2]},
        p2{node_B.pos[0], node_B.pos[1], node_B.pos[2]},
        p3{node_C.pos[0], node_C.pos[1], node_C.pos[2]},
        p4{node_D.pos[0], node_D.pos[1], node_D.pos[2]},
        p5{node_E.pos[0], node_E.pos[1], node_E.pos[2]};
    // 再定义CGAL的线段类
    Segment_3 seg(p1, p2);
    // 定义CGAL的三角形类
    Triangle_3 tri(p3, p4, p5);
    // 计算相交结果
    const auto result = intersection(seg, tri);
    // 如果相交
    if (result)
    {
        // 保证相交图形是一个点
        if (const Point_3 *p = boost::get<Point_3>(&*result))
        {
            intersection_point->pos[0] = p->x();
            intersection_point->pos[1] = p->y();
            intersection_point->pos[2] = p->z();
            return true;
        }
        //else
        //{
        //    std::cout << "Edge-to-face intersection judgment error!\n";
        //    system("pause");
        //}
    }
    return false;
}

int _DATA_PROCESS::Edge_Elem_Intersection(NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E, NODE node_F)
{
    // 首先定义CGAL的顶点类
    Point_3 p1{node_A.pos[0], node_A.pos[1], node_A.pos[2]},
        p2{node_B.pos[0], node_B.pos[1], node_B.pos[2]},
        p3{node_C.pos[0], node_C.pos[1], node_C.pos[2]},
        p4{node_D.pos[0], node_D.pos[1], node_D.pos[2]},
        p5{node_E.pos[0], node_E.pos[1], node_E.pos[2]},
        p6{node_F.pos[0], node_F.pos[1], node_F.pos[2]};
    // 再定义CGAL的线段类
    Segment_3 seg(p1, p2);
    // 定义CGAL的四面体类
    Tetrahedron_3 tet1(p3, p4, p5, p6);
    // 计算相交结果
    const auto result = intersection(seg, tet1);
    // 如果相交
    if (result)
    {
        if (boost::get<Point_3>(&*result))
            return 1;
        else
            return 2;
    }
    return 0;
    //else
    //{
    //    std::cout << "Edge-to-elem intersection judgment error!\n";
    //    system("pause");
    //}
}

int _DATA_PROCESS::Face_Elem_Intersection(NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E, NODE node_F, NODE node_G)
{
    // 首先定义CGAL的顶点类
    Point_3 p1{node_A.pos[0], node_A.pos[1], node_A.pos[2]},
        p2{node_B.pos[0], node_B.pos[1], node_B.pos[2]},
        p3{node_C.pos[0], node_C.pos[1], node_C.pos[2]},
        p4{node_D.pos[0], node_D.pos[1], node_D.pos[2]},
        p5{node_E.pos[0], node_E.pos[1], node_E.pos[2]},
        p6{node_F.pos[0], node_F.pos[1], node_F.pos[2]},
        p7{node_G.pos[0], node_G.pos[1], node_G.pos[2]};
    // 再定义CGAL的面类
    Triangle_3 tri(p1, p2, p3);
    // 定义CGAL的四面体类
    Tetrahedron_3 tet1(p4, p5, p6, p7);
    // 计算相交结果
    const auto result = intersection(tri, tet1);
    // 如果相交
    if (result)
    {
        if (boost::get<Point_3>(&*result))
            return 1;
        else if (boost::get<Segment_3>(&*result))
            return 2;
        else if (boost::get<Triangle_3>(&*result))
            return 3;
        else if (const std::vector<CGAL::Point_3<CGAL::Epick>, std::allocator<CGAL::Point_3<CGAL::Epick>>> *a = boost::get<std::vector<CGAL::Point_3<CGAL::Epick>, std::allocator<CGAL::Point_3<CGAL::Epick>>>>(&*result))
            return 4;
        else
            return 0;
    }
    return 0;
}

void _DATA_PROCESS::Matrix_transpose(double matrix_1[3][1], double matrix_2[3])
{
    matrix_2[0] = matrix_1[0][0];
    matrix_2[1] = matrix_1[1][0];
    matrix_2[2] = matrix_1[2][0];
    return;
}

double _DATA_PROCESS::Vector_Module(double matrix[3][1])
{
    return double(sqrt(pow(matrix[0][0], 2) + pow(matrix[1][0], 2) + pow(matrix[2][0], 2)));
}

double _DATA_PROCESS::Matrix_Determinant(double matrix[2][2])
{
    return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
}

double _DATA_PROCESS::Matrix_Determinant(double matrix[3][3])
{
    double determinant = 0;
    determinant += matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
    determinant -= matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]);
    determinant += matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    return determinant;
}

void _DATA_PROCESS::Inverse_Matrix(double matrix[3][3])
{
    // 定义一个变量，储存原始矩阵
    double Init_matrix[3][3] = {{matrix[0][0], matrix[0][1], matrix[0][2]},
                                {matrix[1][0], matrix[1][1], matrix[1][2]},
                                {matrix[2][0], matrix[2][1], matrix[2][2]}};
    // 求伴随矩阵前的系数，即矩阵行列式的倒数
    double Weights = 1 / Matrix_Determinant(matrix);
    // 用矩阵的伴随矩阵来对矩阵求逆
    matrix[0][0] = Weights * (Init_matrix[1][1] * Init_matrix[2][2] - Init_matrix[1][2] * Init_matrix[2][1]);
    matrix[0][1] = Weights * (Init_matrix[0][2] * Init_matrix[2][1] - Init_matrix[0][1] * Init_matrix[2][2]);
    matrix[0][2] = Weights * (Init_matrix[0][1] * Init_matrix[1][2] - Init_matrix[0][2] * Init_matrix[1][1]);
    matrix[1][0] = Weights * (Init_matrix[1][2] * Init_matrix[2][0] - Init_matrix[1][0] * Init_matrix[2][2]);
    matrix[1][1] = Weights * (Init_matrix[0][0] * Init_matrix[2][2] - Init_matrix[0][2] * Init_matrix[2][0]);
    matrix[1][2] = Weights * (Init_matrix[1][0] * Init_matrix[0][2] - Init_matrix[1][2] * Init_matrix[0][0]);
    matrix[2][0] = Weights * (Init_matrix[1][0] * Init_matrix[2][1] - Init_matrix[1][1] * Init_matrix[2][0]);
    matrix[2][1] = Weights * (Init_matrix[0][1] * Init_matrix[2][0] - Init_matrix[0][0] * Init_matrix[2][1]);
    matrix[2][2] = Weights * (Init_matrix[0][0] * Init_matrix[1][1] - Init_matrix[0][1] * Init_matrix[1][0]);
    return;
}

double _DATA_PROCESS::matrix_product(double matrix_1[3], double matrix_2[3][1])
{
    return matrix_1[0] * matrix_2[0][0] + matrix_1[1] * matrix_2[1][0] + matrix_1[2] * matrix_2[2][0];
}

void _DATA_PROCESS::matrix_product(double matrix_1[3], double matrix_2[3][3], double product[3])
{
    for (int i = 0; i < 3; i++)
        product[i] = matrix_1[0] * matrix_2[0][i] + matrix_1[1] * matrix_2[1][i] + matrix_1[2] * matrix_2[2][i];
    return;
}

void _DATA_PROCESS::matrix_product(double matrix_1[3][1], double matrix_2[3], double product[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            product[i][j] = matrix_1[i][0] * matrix_2[j];
    return;
}

void _DATA_PROCESS::matrix_product(double matrix_1[3][3], double matrix_2[3][1], double product[3][1])
{
    product[0][0] = matrix_1[0][0] * matrix_2[0][0] + matrix_1[0][1] * matrix_2[1][0] + matrix_1[0][2] * matrix_2[2][0];
    product[1][0] = matrix_1[1][0] * matrix_2[0][0] + matrix_1[1][1] * matrix_2[1][0] + matrix_1[1][2] * matrix_2[2][0];
    product[2][0] = matrix_1[2][0] * matrix_2[0][0] + matrix_1[2][1] * matrix_2[1][0] + matrix_1[2][2] * matrix_2[2][0];
    return;
}

void _DATA_PROCESS::matrix_product(double matrix_1[3][3], double matrix_2[3][3], double product[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            product[i][j] = matrix_1[i][0] * matrix_2[0][j] + matrix_1[i][1] * matrix_2[1][j] + matrix_1[i][2] * matrix_2[2][j];
    return;
}

double _DATA_PROCESS::matrix_product(double matrix_1[3], double matrix_2[3][3], double matrix_3[3][1])
{
    double product[3];
    matrix_product(matrix_1, matrix_2, product);
    return matrix_product(product, matrix_3);
}

void _DATA_PROCESS::matrix_product(double matrix_1[3][3], double matrix_2[3][1], double matrix_3[3], double matrix_4[3][3], double product[3][3])
{
    double matrix_pro_1[3][1];
    matrix_product(matrix_1, matrix_2, matrix_pro_1);
    double matrix_pro_2[3][3];
    matrix_product(matrix_pro_1, matrix_3, matrix_pro_2);
    matrix_product(matrix_pro_2, matrix_4, product);
    return;
}

void _DATA_PROCESS::Positivity_Hessian_Matrix(double hessian_matrix[3][3])
{
    int Weights; // 声明一个变量，用于储存权重
    // 使1阶顺序主子式大于0
    if (hessian_matrix[0][0] <= 0)
    {
        Weights = int(abs(hessian_matrix[0][0])) + 1;
        hessian_matrix[0][0] += Weights;
        hessian_matrix[1][1] += Weights;
        hessian_matrix[2][2] += Weights;
    }
    // 使2阶顺序主子式大于0
    if (hessian_matrix[0][0] * hessian_matrix[1][1] - hessian_matrix[0][1] * hessian_matrix[1][0] <= 0)
    {
        // 可以验证Weights一定大于0
        Weights = int((-(hessian_matrix[0][0] + hessian_matrix[1][1]) +
                       sqrt(pow(hessian_matrix[0][0] + hessian_matrix[1][1], 2) -
                            4 * (hessian_matrix[0][0] * hessian_matrix[1][1] - hessian_matrix[0][1] * hessian_matrix[1][0]))) /
                      2);
        hessian_matrix[0][0] += Weights;
        // 由于hessian_matrix[0][1]==hessian_matrix[1][0]，此处hessian_matrix[1][1] += Weights后，hessian_matrix[1][1]的值一定大于0
        hessian_matrix[1][1] += Weights;
        hessian_matrix[2][2] += Weights;
    }
    // 使3阶顺序主子式大于0
    while (Matrix_Determinant(hessian_matrix) <= 0)
    {
        hessian_matrix[0][0] += 1;
        hessian_matrix[1][1] += 1;
        hessian_matrix[2][2] += 1;
    }
    return;
}
