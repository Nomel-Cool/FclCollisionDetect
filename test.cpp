#include <iostream>
#include <math.h>

#include "CollisionDetect3D.h"


//  Convex(
//       const std::shared_ptr<const std::vector<Vector3<S>>>& vertices,
//       int num_faces,
//       const std::shared_ptr<const std::vector<int>>& faces,
//       bool throw_if_invalid = false
//  );

// 三角形边上内嵌点仍是凸集，即该库的凸集定义是广义的
// 但是通过链接该内嵌点使分割为两个三角形时，为凸集
// 在三棱锥某一个面切割为两个三角面片仍然是可以构造为凸多面体的

// 算法流程：
// 1.给出Raw点集
// 2.处理Raw点集形成凸集
// 3.对凸点云进行三角剖分构造成凸多面体
// 4.构造凸多面体的碰撞模型（listed vertices）

int main()
{
#pragma region 探头1

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

#pragma region 人体轮廓

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


    // 计算凸多边形体积
    std::cout << rect1Detour->computeVolume() << std::endl;
    std::cout << patientDetour->computeVolume() << std::endl;

    // 创建对应的碰撞对象
    fcl::CollisionObject<double> obj1(rect1Detour);
    fcl::CollisionObject<double> obj2(patientDetour);

    // 创建一个逆时针旋转45°的变换
    Eigen::AngleAxisd rotation(M_PI / 4, Eigen::Vector3d::UnitZ());
    fcl::Transform3d tf(rotation);

    // 设置矩形的位置
    obj1.setTranslation(fcl::Vector3d(0, 0, 0)); // 第一个矩形位于原点

    // 进行碰撞检测
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    fcl::collide(&obj1, &obj2, request, result);

    // 距离检测
    fcl::DistanceRequestd requestd;
    fcl::DistanceResultd resultd;

    fcl::distance(&obj1, &obj2, requestd, resultd);

    // 输出碰撞结果
    if (result.isCollision()) {
        std::cout << "Collision detected!" << std::endl;
    }
    else {
        std::cout << "No collision detected." << std::endl;
        std::cout << "min_distance:" << resultd.min_distance << std::endl;
    }

    return 0;
}