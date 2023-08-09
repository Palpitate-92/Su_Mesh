#include "Su_Mesh.h"

ELEM::ELEM()
{
    for (int i = 0; i < DIM + 1; i++)
    {
        form[i] = -1;
        neig[i] = -1;
    }
    quality = -1;
}

ELEM::ELEM(int nodeNum_1, int nodeNum_2, int nodeNum_3, int nodeNum_4, int elemNum_1, int elemNum_2, int elemNum_3, int elemNum_4)
{
    form[0] = nodeNum_1;
    form[1] = nodeNum_2;
    form[2] = nodeNum_3;
    form[3] = nodeNum_4;
    neig[0] = elemNum_1;
    neig[1] = elemNum_2;
    neig[2] = elemNum_3;
    neig[3] = elemNum_4;
    quality = -1;
}

void ELEM::Swap(int i, int j)
{
    std::swap(form[i], form[j]);
    std::swap(neig[i], neig[j]);
}

void ELEM::Sort()
{
    // 先让form[0]位置为最小
    if (form[0] > form[1])
        Swap(0, 1);
    if (form[0] > form[2])
        Swap(0, 2);
    if (form[0] > form[3])
        Swap(0, 3);
    // 再让form[1]位置为第二小
    if (form[1] > form[2])
        Swap(1, 2);
    if (form[1] > form[3])
        Swap(1, 3);
    // 最后让form[2]位置为第三小
    if (form[2] > form[3])
        Swap(2, 3);
}

ELEM ELEM::operator=(const int value[8])
{
    for (int i = 0; i < DIM + 1; i++)
    {
        form[i] = value[i];
        neig[i] = value[i + 4];
    }
    return *this;
}

NODE::NODE()
{
    for (int i = 0; i < DIM; i++)
        pos[i] = 0; // 节点坐标应该初始化为零
    elem = -1;
    spac = 0;
}

NODE::NODE(double x, double y, double z)
{
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    elem = -1;
    spac = 0;
}

NODE::NODE(const Point &pot)
{
    std::copy(pot.pos, pot.pos + 3, pos);
    elem = -1;
    spac = -1;
}

NODE NODE::operator+(const NODE &node) const
{
    NODE node_tp;
    node_tp.pos[0] = pos[0] + node.pos[0];
    node_tp.pos[1] = pos[1] + node.pos[1];
    node_tp.pos[2] = pos[2] + node.pos[2];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
}

NODE NODE::operator-(const NODE &node) const
{
    NODE node_tp;
    node_tp.pos[0] = pos[0] - node.pos[0];
    node_tp.pos[1] = pos[1] - node.pos[1];
    node_tp.pos[2] = pos[2] - node.pos[2];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
}

NODE NODE::operator+(const double value[3][1]) const
{
    NODE node_tp;
    node_tp.pos[0] = pos[0] + value[0][0];
    node_tp.pos[1] = pos[1] + value[1][0];
    node_tp.pos[2] = pos[2] + value[2][0];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
}

NODE NODE::operator*(const double value) const
{
    NODE node_tp;
    node_tp.pos[0] = pos[0] * value;
    node_tp.pos[1] = pos[1] * value;
    node_tp.pos[2] = pos[2] * value;
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
}

bool NODE::operator==(const NODE &node) const
{
    return (pos[0] == node.pos[0] && pos[1] == node.pos[1] && pos[2] == node.pos[2]);
}

FACE::FACE()
{
    for (int i = 0; i < DIM; i++)
        form[i] = -1;
}

FACE::FACE(int nodeNum_1, int nodeNum_2, int nodeNum_3)
{
    form[0] = nodeNum_1;
    form[1] = nodeNum_2;
    form[2] = nodeNum_3;
}

void FACE::Sort()
{
    // 先让form+0位置为最小
    if (form[0] > form[1])
        std::swap(form[0], form[1]);
    if (form[0] > form[2])
        std::swap(form[0], form[2]);
    // 再让form+1位置为第二小
    if (form[1] > form[2])
        std::swap(form[1], form[2]);
}

bool FACE::operator==(const FACE &face) const
{
    FACE face_tp1 = *this;
    FACE face_tp2 = face;
    face_tp1.Sort();
    face_tp2.Sort();
    // 所有值相同才相同
    return ((face_tp1.form[0] == face_tp2.form[0]) && (face_tp1.form[1] == face_tp2.form[1]) && (face_tp1.form[2] == face_tp2.form[2]));
}

EDGE::EDGE()
{
    for (int i = 0; i < DIM - 1; i++)
        form[i] = -1;
}

EDGE::EDGE(int nodeNum_1, int nodeNum_2)
{
    form[0] = nodeNum_1;
    form[1] = nodeNum_2;
}

void EDGE::Sort()
{
    if (form[0] > form[1])
        std::swap(form[0], form[1]);
}

void EDGE::Swap()
{
    std::swap(form[0], form[1]);
}

bool EDGE::operator==(const EDGE &edge) const
{
    return ((edge.form[0] == form[0]) && (edge.form[1] == form[1])); // 所有值相同才相同
}

Point::Point()
{
    for (int i = 0; i < 3; i++)
        pos[i] = 0;
}

Point Point::operator+(const Point &point)
{
    Point point_tp;
    point_tp.pos[0] = pos[0] + point.pos[0];
    point_tp.pos[1] = pos[1] + point.pos[1];
    point_tp.pos[2] = pos[2] + point.pos[2];
    return point_tp;
}

Point Point::operator/(const double &value)
{
    Point point_tp;
    point_tp.pos[0] = pos[0] / value;
    point_tp.pos[1] = pos[1] / value;
    point_tp.pos[2] = pos[2] / value;
    return point_tp;
}

Point Point::operator=(const NODE &node)
{
    std::copy(node.pos, node.pos + 3, pos);
    return *this;
}

Pathl::Pathl()
{
    for (int i = 0; i < DIM + 1; i++)
    {
        form[i] = -1;
        neig[i] = -1;
    }
    elem_num = -1;
    type[0] = -1, type[1] = -1;
    memset(pot, 0, sizeof(pot));
    for (int i = 0; i < 6; i++)
        node_num[i] = -1;
    Decom_elem = nullptr;
    Decom_elem_num = 0;
    Decom_type_two_sides = '\0';
}

bool Pathl::operator==(const int &value) const
{
    return (elem_num == value);
}

Pathl Pathl::operator=(const ELEM &elem)
{
    memcpy(form, elem.form, sizeof(elem.form));
    memcpy(neig, elem.neig, sizeof(elem.neig));
    return *this;
}

double Pathl::get_pot_distance()
{
    return double(sqrt(pow(pot[0].pos[0] - pot[1].pos[0], 2) + pow(pot[0].pos[1] - pot[1].pos[1], 2) + pow(pot[0].pos[2] - pot[1].pos[2], 2)));
}

Pathl::~Pathl()
{
    //if (Decom_elem != nullptr)
    //{
    //    free(Decom_elem);
    //    Decom_elem = nullptr;
    //}
    free(Decom_elem);
    Decom_elem = nullptr;
};