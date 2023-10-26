#pragma once
#ifndef FCL_GENERATOR_3D
#define FCL_GENERATOR_3D

#include "eigen/Eigen"
#include "include/fcl/fcl.h"
#include "include/ccd/ccd.h"

const double eps = 1e-12;    //抖动精度精度要求高一点
const int maxn = 210;
constexpr double M_PI = 3.14159265358979323846;

/// <summary>
/// //用rand函数来生成一个非常小的随机数
/// </summary>
/// <returns></returns>
double rand_eps();

struct _Point {    //定义点的结构体
    double x, y, z;    //xyz三个坐标
    /// <summary>
    /// 微小扰动，给每个坐标都加一个极小的随机数
    /// </summary>
    void shake();

    /// <summary>
    /// 转换一下
    /// </summary>
    /// <param name="points"></param>
    /// <returns></returns>
    std::vector<fcl::Vector3<double>> Convert2FclPoints(const std::vector<_Point>& points);

    /// <summary>
    /// 重载一下减号运算符
    /// </summary>
    _Point operator-(_Point t);

    /// <summary>
    /// 叉乘
    /// </summary>
    /// <param name="t">原点到t点的向量</param>
    /// <returns></returns>
    _Point operator*(_Point t);

    /// <summary>
    /// 点积
    /// </summary>
    /// <param name="t">原点到t点的向量</param>
    /// <returns></returns>
    double operator&(_Point t);

    /// <summary>
    /// 求向量的模长，三个坐标也能存向量
    /// </summary>
    /// <returns>点到原点向量模长</returns>
    double len();

};

struct _Plane {    //定义平面的结构体
    int v[3];    //三个顶点

    /// <summary>
    /// 求法向量
    /// </summary>
    /// <param name="p">点向量</param>
    /// <returns>单位点向量</returns>
    _Point norm(std::vector<_Point> p);

    /// <summary>
    /// 判断一个点是否在平面上方
    /// </summary>
    /// <param name="t">点</param>
    /// <param name="p">点集，用于找到点集中平面所在点而已</param>
    /// <returns></returns>
    bool above(_Point t, std::vector<_Point> p);

    /// <summary>
    /// 求一平面的面积
    /// </summary>
    /// <param name="p">点集</param>
    /// <returns></returns>
    double area(std::vector<_Point> p);
};

std::vector<fcl::Vector3<double>> Convert2FclPoints(const std::vector<_Point>& points);

/// <summary>
/// 任务：
/// 1. 接收Raw的3D和2D点集
/// 2. 3D点集执行增量法凸包算法获取表面三角面片轮廓
/// 3. 2D点集执行Gramham-Scan算法获取平面凸包点集
/// 4. 将凸包点集转化为FCL库的Convex碰撞类型
/// 5 .执行碰撞检测
/// 6. 输出碰撞或已碰撞距离容差
/// </summary>
class FclConvex3D
{
public:

    /// <summary>
    /// 构造函数
    /// </summary>
    /// <param name="point_set">输入三维点集</param>
    FclConvex3D(const std::vector<_Point>& point_set);

    /// <summary>
    /// 求取三维凸包
    /// </summary>
    void convex3D();

    /// <summary>
    /// 获取与fcl库对接的Convex面定义序列
    /// </summary>
    /// <returns>详见fcl库的convex构造函数中的参数const std::shared_ptr<const std::vector<int>>& faces</returns>
    std::vector<int> GenFaces();

    /// <summary>
    /// 获取面的数量
    /// </summary>
    /// <returns>凸包三角面片数量</returns>
    int GetNumfaces();

private:
    std::vector<_Point> points;
    _Plane planes[maxn], tp[maxn];    //plane存凸包上的平面，tp用来更新凸包，每次凸包上要留的平面和新加的平面都存进tp，要删的平面不存，最后将tp在复制给plane，就实现了凸包的更新
    std::vector<std::vector<bool>> g; // 邻接矩阵
    int n, m;    //n是总点数，m是凸包中平面的数量
};



#endif // !FCL_GENERATOR_3D
