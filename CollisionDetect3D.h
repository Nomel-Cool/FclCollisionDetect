#pragma once
#ifndef FCL_GENERATOR_3D
#define FCL_GENERATOR_3D

#include "eigen/Eigen"
#include "include/fcl/fcl.h"
#include "include/ccd/ccd.h"

const double eps = 1e-12;    //�������Ⱦ���Ҫ���һ��
const int maxn = 210;
constexpr double M_PI = 3.14159265358979323846;

/// <summary>
/// //��rand����������һ���ǳ�С�������
/// </summary>
/// <returns></returns>
double rand_eps();

struct _Point {    //�����Ľṹ��
    double x, y, z;    //xyz��������
    /// <summary>
    /// ΢С�Ŷ�����ÿ�����궼��һ����С�������
    /// </summary>
    void shake();

    /// <summary>
    /// ת��һ��
    /// </summary>
    /// <param name="points"></param>
    /// <returns></returns>
    std::vector<fcl::Vector3<double>> Convert2FclPoints(const std::vector<_Point>& points);

    /// <summary>
    /// ����һ�¼��������
    /// </summary>
    _Point operator-(_Point t);

    /// <summary>
    /// ���
    /// </summary>
    /// <param name="t">ԭ�㵽t�������</param>
    /// <returns></returns>
    _Point operator*(_Point t);

    /// <summary>
    /// ���
    /// </summary>
    /// <param name="t">ԭ�㵽t�������</param>
    /// <returns></returns>
    double operator&(_Point t);

    /// <summary>
    /// ��������ģ������������Ҳ�ܴ�����
    /// </summary>
    /// <returns>�㵽ԭ������ģ��</returns>
    double len();

};

struct _Plane {    //����ƽ��Ľṹ��
    int v[3];    //��������

    /// <summary>
    /// ������
    /// </summary>
    /// <param name="p">������</param>
    /// <returns>��λ������</returns>
    _Point norm(std::vector<_Point> p);

    /// <summary>
    /// �ж�һ�����Ƿ���ƽ���Ϸ�
    /// </summary>
    /// <param name="t">��</param>
    /// <param name="p">�㼯�������ҵ��㼯��ƽ�����ڵ����</param>
    /// <returns></returns>
    bool above(_Point t, std::vector<_Point> p);

    /// <summary>
    /// ��һƽ������
    /// </summary>
    /// <param name="p">�㼯</param>
    /// <returns></returns>
    double area(std::vector<_Point> p);
};

std::vector<fcl::Vector3<double>> Convert2FclPoints(const std::vector<_Point>& points);

/// <summary>
/// ����
/// 1. ����Raw��3D��2D�㼯
/// 2. 3D�㼯ִ��������͹���㷨��ȡ����������Ƭ����
/// 3. 2D�㼯ִ��Gramham-Scan�㷨��ȡƽ��͹���㼯
/// 4. ��͹���㼯ת��ΪFCL���Convex��ײ����
/// 5 .ִ����ײ���
/// 6. �����ײ������ײ�����ݲ�
/// </summary>
class FclConvex3D
{
public:

    /// <summary>
    /// ���캯��
    /// </summary>
    /// <param name="point_set">������ά�㼯</param>
    FclConvex3D(const std::vector<_Point>& point_set);

    /// <summary>
    /// ��ȡ��ά͹��
    /// </summary>
    void convex3D();

    /// <summary>
    /// ��ȡ��fcl��Խӵ�Convex�涨������
    /// </summary>
    /// <returns>���fcl���convex���캯���еĲ���const std::shared_ptr<const std::vector<int>>& faces</returns>
    std::vector<int> GenFaces();

    /// <summary>
    /// ��ȡ�������
    /// </summary>
    /// <returns>͹��������Ƭ����</returns>
    int GetNumfaces();

private:
    std::vector<_Point> points;
    _Plane planes[maxn], tp[maxn];    //plane��͹���ϵ�ƽ�棬tp��������͹����ÿ��͹����Ҫ����ƽ����¼ӵ�ƽ�涼���tp��Ҫɾ��ƽ�治�棬���tp�ڸ��Ƹ�plane����ʵ����͹���ĸ���
    std::vector<std::vector<bool>> g; // �ڽӾ���
    int n, m;    //n���ܵ�����m��͹����ƽ�������
};



#endif // !FCL_GENERATOR_3D
