#include <iostream>
#include <math.h>

#include "CollisionDetect3D.h"


//  Convex(
//       const std::shared_ptr<const std::vector<Vector3<S>>>& vertices,
//       int num_faces,
//       const std::shared_ptr<const std::vector<int>>& faces,
//       bool throw_if_invalid = false
//  );

// �����α�����Ƕ������͹�������ÿ��͹�������ǹ����
// ����ͨ�����Ӹ���Ƕ��ʹ�ָ�Ϊ����������ʱ��Ϊ͹��
// ������׶ĳһ�����и�Ϊ����������Ƭ��Ȼ�ǿ��Թ���Ϊ͹�������

// �㷨���̣�
// 1.����Raw�㼯
// 2.����Raw�㼯�γ�͹��
// 3.��͹���ƽ��������ʷֹ����͹������
// 4.����͹���������ײģ�ͣ�listed vertices��

int main()
{
#pragma region ̽ͷ1

    std::vector<_Point> detector1 =
    {
        {1,1,1},
        {1,1,-1},
        {1,-1,1},
        {-1,1,1},
        {1,-1,-1},
        {-1,1,-1},
        {-1,-1,1},
        {-1,-1,-1}
    };

    auto fcl_vertices_rect1 = Convert2FclPoints(detector1);

    FclConvex3D f3d_rect1(detector1);

    f3d_rect1.convex3D();

    auto f3s_rect1 = f3d_rect1.GenFaces();

    int num_faces_rect1 = f3d_rect1.GetNumfaces();

    const std::shared_ptr<const std::vector<fcl::Vector3<double>>>  poly_vertices_rect1 = std::make_shared<const std::vector<fcl::Vector3<double>>>(fcl_vertices_rect1);

    std::shared_ptr<const std::vector<int>> poly_faces_rect1 = std::make_shared<const std::vector<int>>(f3s_rect1);

    std::shared_ptr<fcl::Convex<double>> rect1Detour = std::make_shared<fcl::Convex<double>>(poly_vertices_rect1, num_faces_rect1, poly_faces_rect1);

#pragma endregion

#pragma region ��������

    std::vector<_Point> human_detour =
    {
        {1,1,1},
        {1,1,-1},
        {1,-1,1},
        {-1,1,1},
        {1,-1,-1},
        {-1,1,-1},
        {-1,-1,1},
        {-1,-1,-1}
    };

    auto fcl_vertices_human = Convert2FclPoints(human_detour);

    FclConvex3D f3d_human(human_detour);

    f3d_human.convex3D();

    auto f3s_human = f3d_human.GenFaces();

    int num_faces_human = f3d_human.GetNumfaces();

    const std::shared_ptr<const std::vector<fcl::Vector3<double>>>  poly_vertices_human = std::make_shared<const std::vector<fcl::Vector3<double>>>(fcl_vertices_human);

    std::shared_ptr<const std::vector<int>> poly_faces_human = std::make_shared<const std::vector<int>>(f3s_human);

    std::shared_ptr<fcl::Convex<double>> patientDetour = std::make_shared<fcl::Convex<double>>(poly_vertices_human, num_faces_human, poly_faces_human);

#pragma endregion


    // ����͹��������
    std::cout << rect1Detour->computeVolume() << std::endl;
    std::cout << patientDetour->computeVolume() << std::endl;

    // ������Ӧ����ײ����
    fcl::CollisionObject<double> obj1(rect1Detour);
    fcl::CollisionObject<double> obj2(patientDetour);

    // ����һ����ʱ����ת45��ı任
    Eigen::AngleAxisd rotation(M_PI / 4, Eigen::Vector3d::UnitZ());
    fcl::Transform3d tf(rotation);

    // ���þ��ε�λ��
    obj1.setTranslation(fcl::Vector3d(0, 0, 0)); // ��һ������λ��ԭ��

    // ������ײ���
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(&obj1, &obj2, request, result);

    // ������
    fcl::DistanceRequestd requestd;
    fcl::DistanceResultd resultd;

    fcl::distance(&obj1, &obj2, requestd, resultd);

    // �����ײ���
    if (result.isCollision()) {
        std::cout << "Collision detected!" << std::endl;
    }
    else {
        std::cout << "No collision detected." << std::endl;
        std::cout << "min_distance:" << resultd.min_distance << std::endl;
    }

    return 0;
}