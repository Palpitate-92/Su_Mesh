#include "Su_Mesh.h"

extern const double Pi = 3.14159265358979;                      // 声明圆周率 3.14159265358979
extern const double Delta = 0.1;                                // 声明函数求导时的delta
extern const double tolar = 1e-6;                               // 声明函数求导时的两次步长间导数值容忍的最大误差
extern const double Minimum_Shape_Quality_SliverRemoval = 0.02; // 网格单元质量的极值，质量小于该值的网格单元被认定为广义薄元
extern const double Minimum_Shape_Quality_Smoothing = 0.75;     // 网格单元质量的极值，质量小于该值的网格单元运行节点光顺
extern const double Minimum_Shape_Quality_Face_Transform = 0.5; // 网格单元质量的极值，质量小于该值的网格单元运行面交换
extern const double Delta_volume = 1e-5;                        // 声明面交换合法性判断时，前后交换域总体积的最大误差值
extern const double Min_step = 1e-6;                            // 最小步长，基于优化的光顺内使用
extern const int Max_iter = 7;                                  // 最大迭代次数，基于优化的光顺内使用
extern const double Min_imp = 1e-4;                             // 最小差距值，基于优化的光顺使用，比较两次迭代后，优化域目标函数的值变换，小于该值则退出循环，接受坐标
extern const double Min_gradient = 1e-4;                        // 最小梯度值，梯度向量的模小于该值时代表达到极值点处，停止迭代
extern const double Min_steepest_descent = 1e-4;                // 最小最速下降方向值，最速下降方向向量的模小于该值时代表达到极值点处，停止迭代
extern const double Min_helper_function = 1e-5;                 // 最小辅助函数差距值
extern const double c1 = 1e-3;                                  // Armijo准则的常数
extern const double c2 = 0.9;                                   // Wolfe准则的常数
extern const double Max_deviation_point_internal = 1e-4;        // 用于判断点是否在三角形内部时的最大误差
extern const double Max_steiner_point_internal = 1.0;           // 在恢复边界边时，用于判断是否该对steiner点进行合并，该值为一比值，与模型最短边界边相乘值即为最大需合并的steiner点间距离