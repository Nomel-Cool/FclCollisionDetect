#include "CollisionDetect3D.h"

double rand_eps()
{
    return ((double)rand() / RAND_MAX - 0.5) * eps;    //用rand生成一个-0.5到0.5之间的数，再乘eps，就得到了一个非常小的随机数
}

std::vector<fcl::Vector3<double>> Convert2FclPoints(const std::vector<_Point>& points)
{
    std::vector<fcl::Vector3<double>> fcl_point_list;
    for (const auto& p : points)
    {
        fcl_point_list.push_back({ p.x,p.y,p.z });
    }
    return fcl_point_list;
}

#pragma region 点定义

void _Point::shake()
{
    x += rand_eps(), y += rand_eps(), z += rand_eps();
}

_Point _Point::operator-(_Point t)
{
    return { x - t.x, y - t.y, z - t.z };
}

_Point _Point::operator*(_Point t)
{
    return { y * t.z - t.y * z, t.x * z - x * t.z, x * t.y - y * t.x };
}

double _Point::operator&(_Point t)
{
    return x * t.x + y * t.y + z * t.z;
}

double _Point::len()
{
    return sqrt(x * x + y * y + z * z);
}
#pragma endregion

#pragma region 面定义

_Point _Plane::norm(std::vector<_Point> p)
{
    return (p[v[1]] - p[v[0]]) * (p[v[2]] - p[v[0]]);    //平面中两向量的叉积
}

bool _Plane::above(_Point t, std::vector<_Point> p)
{
    return ((t - p[v[0]]) & norm(p)) >= 0;    //用向量和法向量的点积判断
}

double _Plane::area(std::vector<_Point> p)
{
    return norm(p).len() / 2;    //法向量的模长除以2
}

#pragma endregion

#pragma region 3D FCL 碰撞凸包

FclConvex3D::FclConvex3D(const std::vector<_Point>& point_set)
{
    if (point_set.size() < 3)throw;

    n = point_set.size();
    m = 0;

    // 初始化点集
    points.resize(n);
    for (int i = 0; i < n; i++)
    {
        points[i].x = point_set[i].x;
        points[i].y = point_set[i].y;
        points[i].z = point_set[i].z;
        points[i].shake();
    }

    // 初始化n×n邻接矩阵
    g.resize(n);
    for (int j = 0; j < n; j++)
    {
        g[j].resize(n, false);
    }
}

void FclConvex3D::convex3D()
{
    planes[m++] = { 0,1,2 };    //初始化凸包，随便三个点存入，确定最开始的一个平面，这里取得是前三个点
    planes[m++] = { 2,1,0 };    //因为不知道第一个平面怎么样是逆时针，所以都存一遍，顺时针存的一会会被删掉
    for (int i = 3; i < n; i++)    //从第四个点开始循环每个点
    {
        int cnt = 0;
        for (int j = 0; j < m; j++)    //循环每个平面
        {
            bool fg = planes[j].above(points[i], points);    //判断这个点是否在该平面上方
            if (!fg)        //如果是下方的话，说明照不到
                tp[cnt++] = planes[j];    //存进tp数组
            for (int k = 0; k < 3; k++)    //循环该平面的三条边
                g[planes[j].v[k]][planes[j].v[(k + 1) % 3]] = fg;    //ab边照不照得到情况赋值给g[a][b]
        }
        for (int j = 0; j < m; j++)    //然后就循环每个平面的每条边
        {
            for (int k = 0; k < 3; k++)
            {
                int a = planes[j].v[k], b = planes[j].v[(k + 1) % 3];
                if (g[a][b] && !g[b][a])        //判断该边是否被照到了一次，即是否是交界线的边
                    tp[cnt++] = { a,b,i };    //若是，加新平面abi，ab一定是逆时针的，i在后面
            }
        }
        m = cnt;    //将tp再赋值给plane
        for (int j = 0; j < m; j++)
            planes[j] = tp[j];
    }
}

std::vector<int> FclConvex3D::GenFaces()
{
    std::vector<int> faces;
    faces.resize((3 + 1) * m);
    for (int i = 0; i < 4 * m; i += 4)
    {
        faces[i] = 3;
        faces[i + 1] = (planes[i / 4].v[0]);
        faces[i + 2] = (planes[i / 4].v[1]);
        faces[i + 3] = (planes[i / 4].v[2]);
    }
    return faces;
}

int FclConvex3D::GetNumfaces()
{
    return m;
}

#pragma endregion
