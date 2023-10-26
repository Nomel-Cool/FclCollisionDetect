#include "CollisionDetect3D.h"

double rand_eps()
{
    return ((double)rand() / RAND_MAX - 0.5) * eps;    //��rand����һ��-0.5��0.5֮��������ٳ�eps���͵õ���һ���ǳ�С�������
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

#pragma region �㶨��

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

#pragma region �涨��

_Point _Plane::norm(std::vector<_Point> p)
{
    return (p[v[1]] - p[v[0]]) * (p[v[2]] - p[v[0]]);    //ƽ�����������Ĳ��
}

bool _Plane::above(_Point t, std::vector<_Point> p)
{
    return ((t - p[v[0]]) & norm(p)) >= 0;    //�������ͷ������ĵ���ж�
}

double _Plane::area(std::vector<_Point> p)
{
    return norm(p).len() / 2;    //��������ģ������2
}

#pragma endregion

#pragma region 3D FCL ��ײ͹��

FclConvex3D::FclConvex3D(const std::vector<_Point>& point_set)
{
    if (point_set.size() < 3)throw;

    n = point_set.size();
    m = 0;

    // ��ʼ���㼯
    points.resize(n);
    for (int i = 0; i < n; i++)
    {
        points[i].x = point_set[i].x;
        points[i].y = point_set[i].y;
        points[i].z = point_set[i].z;
        points[i].shake();
    }

    // ��ʼ��n��n�ڽӾ���
    g.resize(n);
    for (int j = 0; j < n; j++)
    {
        g[j].resize(n, false);
    }
}

void FclConvex3D::convex3D()
{
    planes[m++] = { 0,1,2 };    //��ʼ��͹���������������룬ȷ���ʼ��һ��ƽ�棬����ȡ����ǰ������
    planes[m++] = { 2,1,0 };    //��Ϊ��֪����һ��ƽ����ô������ʱ�룬���Զ���һ�飬˳ʱ����һ��ᱻɾ��
    for (int i = 3; i < n; i++)    //�ӵ��ĸ��㿪ʼѭ��ÿ����
    {
        int cnt = 0;
        for (int j = 0; j < m; j++)    //ѭ��ÿ��ƽ��
        {
            bool fg = planes[j].above(points[i], points);    //�ж�������Ƿ��ڸ�ƽ���Ϸ�
            if (!fg)        //������·��Ļ���˵���ղ���
                tp[cnt++] = planes[j];    //���tp����
            for (int k = 0; k < 3; k++)    //ѭ����ƽ���������
                g[planes[j].v[k]][planes[j].v[(k + 1) % 3]] = fg;    //ab���ղ��յõ������ֵ��g[a][b]
        }
        for (int j = 0; j < m; j++)    //Ȼ���ѭ��ÿ��ƽ���ÿ����
        {
            for (int k = 0; k < 3; k++)
            {
                int a = planes[j].v[k], b = planes[j].v[(k + 1) % 3];
                if (g[a][b] && !g[b][a])        //�жϸñ��Ƿ��յ���һ�Σ����Ƿ��ǽ����ߵı�
                    tp[cnt++] = { a,b,i };    //���ǣ�����ƽ��abi��abһ������ʱ��ģ�i�ں���
            }
        }
        m = cnt;    //��tp�ٸ�ֵ��plane
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
