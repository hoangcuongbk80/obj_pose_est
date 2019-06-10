#include "obj_pose_est/model_to_scene.h"

model_to_scene::model_to_scene()
{
    std::cerr << "Start model to scene!!!\n";
}

model_to_scene::~model_to_scene()
{
   if(depthImg!=NULL)
   delete depthImg;
   if(rgbImg!=NULL)
   delete rgbImg;
}

void model_to_scene::computeOBB(const pcl::PointCloud<pcl::PointXYZ> &input, boundingBBox &OBB)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(input, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(input, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(input, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    OBB.length[0] = maxPoint.x - minPoint.x; //MAX length OBB
    OBB.length[1] = maxPoint.y - minPoint.y; //MID length OBB
    OBB.length[2] = maxPoint.z - minPoint.z; //MIN length OBB

    if(OBB.length[0] < OBB.length[1])
    {
        float buf = OBB.length[0]; OBB.length[0] = OBB.length[1]; 
        OBB.length[1] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }
    if(OBB.length[0] < OBB.length[2])
    {
        float buf = OBB.length[0]; OBB.length[0] = OBB.length[2]; 
        OBB.length[2] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitY()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }
    if(OBB.length[1] < OBB.length[2])
    {
        float buf = OBB.length[1]; OBB.length[1] = OBB.length[2]; 
        OBB.length[2] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitX()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }

    pcl::getMinMax3D(*cloudPointsProjected, OBB.minPoint, OBB.maxPoint);
    OBB.toOrigin = projectionTransform;

    pcl::PointXYZ OBB_points;

    OBB.cornerPoints.push_back(OBB.minPoint); // Min Point
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);

    OBB.cornerPoints.push_back(OBB.maxPoint); //Max point
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);

    pcl::transformPointCloud(OBB.cornerPoints, OBB.cornerPoints, projectionTransform.inverse());

    OBB.center.x = 0; OBB.center.y = 0; OBB.center.z = 0;
    for (int i = 0; i < OBB.cornerPoints.size(); i++)
    {
        OBB.center.x += OBB.cornerPoints[i].x;
        OBB.center.y += OBB.cornerPoints[i].y;
        OBB.center.z += OBB.cornerPoints[i].z;
    }
    OBB.center.x = OBB.center.x / OBB.cornerPoints.size();
    OBB.center.y = OBB.center.y / OBB.cornerPoints.size();
    OBB.center.z = OBB.center.z / OBB.cornerPoints.size();
}

void model_to_scene::coarseToFineRegistration(const pcl::PointCloud<pcl::PointXYZ> &sourceCloud, 
                                  const pcl::PointCloud<pcl::PointXYZ> &targetCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::copyPointCloud(model, *targetCloud);

    boundingBBox source_OBB, target_OBB;
    computeOBB(sourceCloud, source_OBB);
    computeOBB(targetCloud, target_OBB);
    pcl::transformPointCloud(sourceCloud, *source, source_OBB.toOrigin);
    pcl::transformPointCloud(targetCloud, *target, target_OBB.toOrigin);


    for(double RX = 0; RX <= M_PI; RX+=M_PI)
    for(double RY = 0; RY <= M_PI; RY+=M_PI)
    for(double RZ = 0; RZ < M_PI; RZ+=M_PI)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);;
        
        /* Eigen::Affine3f ROT = Eigen::Affine3f::Identity();
        ROT.rotate (Eigen::AngleAxisf (RX, Eigen::Vector3f::UnitX()));
        ROT.rotate (Eigen::AngleAxisf (RY, Eigen::Vector3f::UnitY()));
        ROT.rotate (Eigen::AngleAxisf (RZ, Eigen::Vector3f::UnitZ()));                        
        pcl::transformPointCloud (*origCluster, *sourceCloud, ROT);
    
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(sourceCloud);
        icp.setInputTarget(targetCloud);
        icp.setMaximumIterations (100);
        icp.setMaxCorrespondenceDistance(0.2);
        icp.setRANSACOutlierRejectionThreshold(1);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        double overlapScore = overlapPortion(*targetCloud, Final, overlap_dst_thresh);
        std::cerr << "overlap score: " << overlapScore << "\n";
        if(overlapScore < overlap_score_thresh) continue; 

        if(icp.hasConverged() & bestScore < overlapScore)                                             
        {
            bestMat = icp.getFinalTransformation();
            myclusters[i].recognizedObj = models[k].name;
            bestCoarseMat = ROT.matrix();
            bestFineMat = icp.getFinalTransformation();
            bestModel = k;
            bestScore = overlapScore;
        } */

        /* if(icp.hasConverged() || bestScore > icp.getFitnessScore())                                             
        {
            bestMat = icp.getFinalTransformation();
            myclusters[i].recognizedObj = models[k].name;
            bestCoarseMat = ROT.matrix();
            bestFineMat = icp.getFinalTransformation();
            bestModel = k;
            bestScore = icp.getFitnessScore();
        } */
    }

}