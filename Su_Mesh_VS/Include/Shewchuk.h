/*
 * 声明Shewchuk的基于自适应精度算术实现的几何谓词
 */

// #pragma once
#ifndef _SHEWCHUK_H
#define _SHEWCHUK_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define INEXACT /* Nothing */

#define REAL double /* float or double */
#define REALPRINT doubleprint
#define REALRAND doublerand
#define NARROWRAND narrowdoublerand
#define UNIFORMRAND uniformdoublerand

#define Absolute(a) ((a) >= 0.0 ? (a) : -(a))

#define Fast_Two_Sum_Tail(a, b, x, y) \
  bvirt = x - a;                      \
  y = b - bvirt

#define Fast_Two_Sum(a, b, x, y) \
  x = (REAL)(a + b);             \
  Fast_Two_Sum_Tail(a, b, x, y)

#define Fast_Two_Diff_Tail(a, b, x, y) \
  bvirt = a - x;                       \
  y = bvirt - b

#define Fast_Two_Diff(a, b, x, y) \
  x = (REAL)(a - b);              \
  Fast_Two_Diff_Tail(a, b, x, y)

#define Two_Sum_Tail(a, b, x, y) \
  bvirt = (REAL)(x - a);         \
  avirt = x - bvirt;             \
  bround = b - bvirt;            \
  around = a - avirt;            \
  y = around + bround

#define Two_Sum(a, b, x, y) \
  x = (REAL)(a + b);        \
  Two_Sum_Tail(a, b, x, y)

#define Two_Diff_Tail(a, b, x, y) \
  bvirt = (REAL)(a - x);          \
  avirt = x + bvirt;              \
  bround = bvirt - b;             \
  around = a - avirt;             \
  y = around + bround

#define Two_Diff(a, b, x, y) \
  x = (REAL)(a - b);         \
  Two_Diff_Tail(a, b, x, y)

#define Split(a, ahi, alo)  \
  c = (REAL)(splitter * a); \
  abig = (REAL)(c - a);     \
  ahi = c - abig;           \
  alo = a - ahi

#define Two_Product_Tail(a, b, x, y) \
  Split(a, ahi, alo);                \
  Split(b, bhi, blo);                \
  err1 = x - (ahi * bhi);            \
  err2 = err1 - (alo * bhi);         \
  err3 = err2 - (ahi * blo);         \
  y = (alo * blo) - err3

#define Two_Product(a, b, x, y) \
  x = (REAL)(a * b);            \
  Two_Product_Tail(a, b, x, y)

#define Two_Product_Presplit(a, b, bhi, blo, x, y) \
  x = (REAL)(a * b);                               \
  Split(a, ahi, alo);                              \
  err1 = x - (ahi * bhi);                          \
  err2 = err1 - (alo * bhi);                       \
  err3 = err2 - (ahi * blo);                       \
  y = (alo * blo) - err3

#define Two_Product_2Presplit(a, ahi, alo, b, bhi, blo, x, y) \
  x = (REAL)(a * b);                                          \
  err1 = x - (ahi * bhi);                                     \
  err2 = err1 - (alo * bhi);                                  \
  err3 = err2 - (ahi * blo);                                  \
  y = (alo * blo) - err3

#define Square_Tail(a, x, y)         \
  Split(a, ahi, alo);                \
  err1 = x - (ahi * ahi);            \
  err3 = err1 - ((ahi + ahi) * alo); \
  y = (alo * alo) - err3

#define Square(a, x, y) \
  x = (REAL)(a * a);    \
  Square_Tail(a, x, y)

#define Two_One_Sum(a1, a0, b, x2, x1, x0) \
  Two_Sum(a0, b, _i, x0);                  \
  Two_Sum(a1, _i, x2, x1)

#define Two_One_Diff(a1, a0, b, x2, x1, x0) \
  Two_Diff(a0, b, _i, x0);                  \
  Two_Sum(a1, _i, x2, x1)

#define Two_Two_Sum(a1, a0, b1, b0, x3, x2, x1, x0) \
  Two_One_Sum(a1, a0, b0, _j, _0, x0);              \
  Two_One_Sum(_j, _0, b1, x3, x2, x1)

#define Two_Two_Diff(a1, a0, b1, b0, x3, x2, x1, x0) \
  Two_One_Diff(a1, a0, b0, _j, _0, x0);              \
  Two_One_Diff(_j, _0, b1, x3, x2, x1)

#define Four_One_Sum(a3, a2, a1, a0, b, x4, x3, x2, x1, x0) \
  Two_One_Sum(a1, a0, b, _j, x1, x0);                       \
  Two_One_Sum(a3, a2, _j, x4, x3, x2)

#define Four_Two_Sum(a3, a2, a1, a0, b1, b0, x5, x4, x3, x2, x1, x0) \
  Four_One_Sum(a3, a2, a1, a0, b0, _k, _2, _1, _0, x0);              \
  Four_One_Sum(_k, _2, _1, _0, b1, x5, x4, x3, x2, x1)

#define Four_Four_Sum(a3, a2, a1, a0, b4, b3, b1, b0, x7, x6, x5, x4, x3, x2, \
                      x1, x0)                                                 \
  Four_Two_Sum(a3, a2, a1, a0, b1, b0, _l, _2, _1, _0, x1, x0);               \
  Four_Two_Sum(_l, _2, _1, _0, b4, b3, x7, x6, x5, x4, x3, x2)

#define Eight_One_Sum(a7, a6, a5, a4, a3, a2, a1, a0, b, x8, x7, x6, x5, x4, \
                      x3, x2, x1, x0)                                        \
  Four_One_Sum(a3, a2, a1, a0, b, _j, x3, x2, x1, x0);                       \
  Four_One_Sum(a7, a6, a5, a4, _j, x8, x7, x6, x5, x4)

#define Eight_Two_Sum(a7, a6, a5, a4, a3, a2, a1, a0, b1, b0, x9, x8, x7,   \
                      x6, x5, x4, x3, x2, x1, x0)                           \
  Eight_One_Sum(a7, a6, a5, a4, a3, a2, a1, a0, b0, _k, _6, _5, _4, _3, _2, \
                _1, _0, x0);                                                \
  Eight_One_Sum(_k, _6, _5, _4, _3, _2, _1, _0, b1, x9, x8, x7, x6, x5, x4, \
                x3, x2, x1)

#define Eight_Four_Sum(a7, a6, a5, a4, a3, a2, a1, a0, b4, b3, b1, b0, x11, \
                       x10, x9, x8, x7, x6, x5, x4, x3, x2, x1, x0)         \
  Eight_Two_Sum(a7, a6, a5, a4, a3, a2, a1, a0, b1, b0, _l, _6, _5, _4, _3, \
                _2, _1, _0, x1, x0);                                        \
  Eight_Two_Sum(_l, _6, _5, _4, _3, _2, _1, _0, b4, b3, x11, x10, x9, x8,   \
                x7, x6, x5, x4, x3, x2)

#define Two_One_Product(a1, a0, b, x3, x2, x1, x0) \
  Split(b, bhi, blo);                              \
  Two_Product_Presplit(a0, b, bhi, blo, _i, x0);   \
  Two_Product_Presplit(a1, b, bhi, blo, _j, _0);   \
  Two_Sum(_i, _0, _k, x1);                         \
  Fast_Two_Sum(_j, _k, x3, x2)

#define Four_One_Product(a3, a2, a1, a0, b, x7, x6, x5, x4, x3, x2, x1, x0) \
  Split(b, bhi, blo);                                                       \
  Two_Product_Presplit(a0, b, bhi, blo, _i, x0);                            \
  Two_Product_Presplit(a1, b, bhi, blo, _j, _0);                            \
  Two_Sum(_i, _0, _k, x1);                                                  \
  Fast_Two_Sum(_j, _k, _i, x2);                                             \
  Two_Product_Presplit(a2, b, bhi, blo, _j, _0);                            \
  Two_Sum(_i, _0, _k, x3);                                                  \
  Fast_Two_Sum(_j, _k, _i, x4);                                             \
  Two_Product_Presplit(a3, b, bhi, blo, _j, _0);                            \
  Two_Sum(_i, _0, _k, x5);                                                  \
  Fast_Two_Sum(_j, _k, x7, x6)

#define Two_Two_Product(a1, a0, b1, b0, x7, x6, x5, x4, x3, x2, x1, x0) \
  Split(a0, a0hi, a0lo);                                                \
  Split(b0, bhi, blo);                                                  \
  Two_Product_2Presplit(a0, a0hi, a0lo, b0, bhi, blo, _i, x0);          \
  Split(a1, a1hi, a1lo);                                                \
  Two_Product_2Presplit(a1, a1hi, a1lo, b0, bhi, blo, _j, _0);          \
  Two_Sum(_i, _0, _k, _1);                                              \
  Fast_Two_Sum(_j, _k, _l, _2);                                         \
  Split(b1, bhi, blo);                                                  \
  Two_Product_2Presplit(a0, a0hi, a0lo, b1, bhi, blo, _i, _0);          \
  Two_Sum(_1, _0, _k, x1);                                              \
  Two_Sum(_2, _k, _j, _1);                                              \
  Two_Sum(_l, _j, _m, _2);                                              \
  Two_Product_2Presplit(a1, a1hi, a1lo, b1, bhi, blo, _j, _0);          \
  Two_Sum(_i, _0, _n, _0);                                              \
  Two_Sum(_1, _0, _i, x2);                                              \
  Two_Sum(_2, _i, _k, _1);                                              \
  Two_Sum(_m, _k, _l, _2);                                              \
  Two_Sum(_j, _n, _k, _0);                                              \
  Two_Sum(_1, _0, _j, x3);                                              \
  Two_Sum(_2, _j, _i, _1);                                              \
  Two_Sum(_l, _i, _m, _2);                                              \
  Two_Sum(_1, _k, _i, x4);                                              \
  Two_Sum(_2, _i, _k, x5);                                              \
  Two_Sum(_m, _k, x7, x6)

#define Two_Square(a1, a0, x5, x4, x3, x2, x1, x0) \
  Square(a0, _j, x0);                              \
  _0 = a0 + a0;                                    \
  Two_Product(a1, _0, _k, _1);                     \
  Two_One_Sum(_k, _1, _j, _l, _2, x1);             \
  Square(a1, _j, _1);                              \
  Two_Two_Sum(_j, _1, _l, _2, x5, x4, x3, x2)

class _SHEWCHUK
{
public:
  /*
  orient2dfast() 近似二维方向测试。 不稳健。
  orient2dexact() 精确的二维方向测试。 强壮的。
  orient2dslow() 另一个精确的 2D 方向测试。 强壮的。
  orient2d() 自适应精确二维方向测试。 强壮的。

  如果点pa、pb、pc按逆时针顺序出现则返回正值；
  如果它们按顺时针顺序出现，则为负值； 如果它们共线则为零。
  结果也是由三个点定义的三角形的符号面积的两倍的粗略近似值。

  只应使用第一个和最后一个例程； 中间两个是计时。

  最后三个使用精确算术来确保正确答案。 返回的结果是矩阵的行列式。
  仅在 orient2d() 中，此行列式是自适应计算的，从某种意义上说，
  精确算术仅用于确保返回值具有正确符号所需的程度。
  因此，orient2d() 通常非常快，但当输入点共线或接近共线时运行速度会更慢。
  */
  REAL orient2dadapt(REAL *pa, REAL *pb, REAL *pc, REAL detsum);
  REAL orient2d(REAL *pa, REAL *pb, REAL *pc);
  /*
  orient3dfast()近似 3D 方向测试。 不稳健。
  orient3dexact()精确的 3D 方向测试。 强壮的。
  orient3dslow()另一个精确的 3D 方向测试。 强壮的。
  orient3d()自适应精确 3D 方向测试。 强壮的。
  如果点pd位于通过pa、pb和pc的平面下方，则返回正值；“下方”被定义为从平面上方看时 pa、pb 和 pc 以逆时针顺序出现。
  如果pd位于平面上方，则返回负值。如果点共面，则返回零。结果也是由四个点定义的四面体的有符号体积的六倍的粗略近似值。
  只应使用第一个和最后一个例程；中间两个是时间。
  最后三个使用精确算术来确保正确答案。返回的结果是矩阵的行列式。
  仅在orient3d()中，此行列式是自适应计算的，因为精确算术仅用于确保返回值具有正确符号所需的程度。
  因此，orient3d()通常非常快，但当输入点共面或接近共面时运行速度会更慢。
  */
  REAL orient3dadapt(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL permanent);
  REAL orient3d(REAL *pa, REAL *pb, REAL *pc, REAL *pd);
  /*
  incirclefast() 近似二维内切圆测试。 不稳健。
  incircleexact() 精确的二维内切圆测试。 强壮的。
  incircleslow() 另一个精确的 2D 内切圆测试。 强壮的。
  incircle() 自适应精确二维内切圆测试。 强壮的。
  如果点 pd 位于通过 pa、pb 和 pc 的圆内，则返回正值；
  如果它在外面，则为负值； 如果四个点共圆则为零。pa、pb、pc
  点必须按逆时针顺序排列，否则结果的符号会颠倒。
  只应使用第一个和最后一个例程； 中间两个是时间。
  最后三个使用精确算术来确保正确答案。 返回的结果是矩阵的行列式。
  仅在 incircle() 中，此行列式是自适应计算的，从某种意义上说，
  精确算术仅用于确保返回值具有正确符号所需的程度。
  因此，incircle() 通常非常快，但当输入点同圆或接近同圆时运行速度会更慢。
  */
  REAL incircleadapt(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL permanent);
  REAL incircle(REAL *pa, REAL *pb, REAL *pc, REAL *pd);
  /*
  inspherefast()近似3D insphere测试。不稳健。
  insphereexact()精确的3D insphere测试。强壮的。
  insphereslow()另一个精确的3D insphere测试。强壮的。
  insphere()自适应精确3D insphere测试。强壮的。
  如果点pe位于通 pa、pb、pc和 pd的球体内，则返回正值；如果它在外面，则为负值；如果五个点是同球面的，则为零。
  点pa、pb、pc和pd必须排序，以便它们具有正方向（由orient3d()定义），否则结果的符号将反转。
  只应使用第一个和最后一个例程；中间两个是时间。
  最后三个使用精确算术来确保正确答案。返回的结果是矩阵的行列式。
  仅在insphere()中，此行列式是自适应计算的，因为精确算术仅用于确保返回值具有正确符号所需的程度。
  因此，insphere()通常非常快，但当输入点是同球面或接近同球面时运行速度会更慢。
  */
  REAL insphereexact(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe);
  REAL insphereadapt(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe, REAL permanent);
  REAL insphere(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe);

  void exactinit();
  int grow_expansion(int elen, REAL *e, REAL b, REAL *h);
  int grow_expansion_zeroelim(int elen, REAL *e, REAL b, REAL *h);
  int expansion_sum(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int expansion_sum_zeroelim1(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int expansion_sum_zeroelim2(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int fast_expansion_sum(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int fast_expansion_sum_zeroelim(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int linear_expansion_sum(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int linear_expansion_sum_zeroelim(int elen, REAL *e, int flen, REAL *f, REAL *h);
  int scale_expansion(int elen, REAL *e, REAL b, REAL *h);
  int scale_expansion_zeroelim(int elen, REAL *e, REAL b, REAL *h);
  int compress(int elen, REAL *e, REAL *h);
  REAL estimate(int elen, REAL *e);

private:
};

#endif