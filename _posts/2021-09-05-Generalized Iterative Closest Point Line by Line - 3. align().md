---
layout: post
title: Generalized Iterative Closest Point Line by Line - 3. align()
subtitle: G-ICP 내부 설명 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Generalized Iterative Closest Point (G-ICP)

(바빠서 코드만 살펴보고 설명을 못 적는 중...)

https://pointclouds.org/documentation/registration_8hpp_source.html

```cpp
template <typename PointSource, typename PointTarget, typename Scalar>
 inline void
 Registration<PointSource, PointTarget, Scalar>::align(PointCloudSource& output)
 {
   align(output, Matrix4::Identity());
 }
  
 template <typename PointSource, typename PointTarget, typename Scalar>
 inline void
 Registration<PointSource, PointTarget, Scalar>::align(PointCloudSource& output,
                                                       const Matrix4& guess)
 {
    if (!initCompute())
        return;

    // Resize the output dataset
    output.resize(indices_->size());
    // Copy the header
    output.header = input_->header;
    // Check if the output will be computed for all points or only a subset
    if (indices_->size() != input_->size()) {
        output.width = indices_->size();
        output.height = 1;
    }
    else {
        output.width = static_cast<std::uint32_t>(input_->width);
        output.height = input_->height;
    }
    output.is_dense = input_->is_dense;

    // Copy the point data to output
    for (std::size_t i = 0; i < indices_->size(); ++i)
        output[i] = (*input_)[(*indices_)[i]];

    // Set the internal point representation of choice unless otherwise noted
    if (point_representation_ && !force_no_recompute_)
        tree_->setPointRepresentation(point_representation_);

    // Perform the actual transformation computation
    converged_ = false;
    final_transformation_ = transformation_ = previous_transformation_ =
        Matrix4::Identity();

    // Right before we estimate the transformation, we set all the point.data[3] values to
    // 1 to aid the rigid transformation
    for (std::size_t i = 0; i < indices_->size(); ++i)
      output[i].data[3] = 1.0;

    computeTransformation(output, guess);

    deinitCompute();
 }
```

```cpp
struct _PointXYZ
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

```

https://pointclouds.org/documentation/point__types_8hpp_source.html


```cpp
template <typename PointSource, typename PointTarget>
 inline void
 GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeTransformation(
     PointCloudSource& output, const Eigen::Matrix4f& guess)
{
    pcl::IterativeClosestPoint<PointSource, PointTarget>::initComputeReciprocal();
    // Difference between consecutive transforms
    double delta = 0;
    // Get the size of the source point cloud
    const std::size_t N = indices_->size();
    // Set the mahalanobis matrices to identity
    mahalanobis_.resize(N, Eigen::Matrix3d::Identity());
    // Compute target cloud covariance matrices
    if ((!target_covariances_) || (target_covariances_->empty())) {
        target_covariances_.reset(new MatricesVector);
        computeCovariances<PointTarget>(target_, tree_, *target_covariances_);
    }
    // Compute input cloud covariance matrices
    if ((!input_covariances_) || (input_covariances_->empty())) {
        input_covariances_.reset(new MatricesVector);
        computeCovariances<PointSource>(input_, tree_reciprocal_, *input_covariances_);
    }
  
    base_transformation_ = Eigen::Matrix4f::Identity();
    nr_iterations_ = 0;
    converged_ = false;
    double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
    pcl::Indices nn_indices(1);
    std::vector<float> nn_dists(1);
    
    pcl::transformPointCloud(output, output, guess);
  
    while (!converged_) {
        std::size_t cnt = 0;
        pcl::Indices source_indices(indices_->size());
        pcl::Indices target_indices(indices_->size());
    
        // guess corresponds to base_t and transformation_ to t
        Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero();
        for (std::size_t i = 0; i < 4; i++)
        for (std::size_t j = 0; j < 4; j++)
            for (std::size_t k = 0; k < 4; k++)
            transform_R(i, j) += double(transformation_(i, k)) * double(guess(k, j));
    
        Eigen::Matrix3d R = transform_R.topLeftCorner<3, 3>();
    
        for (std::size_t i = 0; i < N; i++) {
        PointSource query = output[i];
        query.getVector4fMap() = transformation_ * query.getVector4fMap();
    
            if (!searchForNeighbors(query, nn_indices, nn_dists)) {
                PCL_ERROR("[pcl::%s::computeTransformation] Unable to find a nearest neighbor "
                        "in the target dataset for point %d in the source!\n",
                        getClassName().c_str(),
                        (*indices_)[i]);
                return;
            }
    
        // Check if the distance to the nearest neighbor is smaller than the user imposed
        // threshold
        if (nn_dists[0] < dist_threshold) {
            Eigen::Matrix3d& C1 = (*input_covariances_)[i];
            Eigen::Matrix3d& C2 = (*target_covariances_)[nn_indices[0]];
            Eigen::Matrix3d& M = mahalanobis_[i];
            // M = R*C1
            M = R * C1;
            // temp = M*R' + C2 = R*C1*R' + C2
            Eigen::Matrix3d temp = M * R.transpose();
            temp += C2;
            // M = temp^-1
            M = temp.inverse();
            source_indices[cnt] = static_cast<int>(i);
            target_indices[cnt] = nn_indices[0];
            cnt++;
        }
        }
        // Resize to the actual number of valid correspondences
        source_indices.resize(cnt);
        target_indices.resize(cnt);
        /* optimize transformation using the current assignment and Mahalanobis metrics*/
        previous_transformation_ = transformation_;
        // optimization right here
        try {
        rigid_transformation_estimation_(
            output, source_indices, *target_, target_indices, transformation_);
            /* compute the delta from this iteration */
            delta = 0.;
            for (int k = 0; k < 4; k++) {
                for (int l = 0; l < 4; l++) {
                double ratio = 1;
                if (k < 3 && l < 3) // rotation part of the transform
                    ratio = 1. / rotation_epsilon_;
                else
                    ratio = 1. / transformation_epsilon_;
                double c_delta =
                    ratio * std::abs(previous_transformation_(k, l) - transformation_(k, l));
                if (c_delta > delta)
                    delta = c_delta;
                }
            }
        } catch (PCLException& e) {
        PCL_DEBUG("[pcl::%s::computeTransformation] Optimization issue %s\n",
                    getClassName().c_str(),
                    e.what());
        break;
        }
        nr_iterations_++;
    
        if (update_visualizer_ != 0) {
            PointCloudSourcePtr input_transformed(new PointCloudSource);
            pcl::transformPointCloud(output, *input_transformed, transformation_);
            update_visualizer_(*input_transformed, source_indices, *target_, target_indices);
        }

        // Check for convergence
        if (nr_iterations_ >= max_iterations_ || delta < 1) {
            converged_ = true;
            PCL_DEBUG("[pcl::%s::computeTransformation] Convergence reached. Number of "
                        "iterations: %d out of %d. Transformation difference: %f\n",
                        getClassName().c_str(),
                        nr_iterations_,
                        max_iterations_,
                        (transformation_ - previous_transformation_).array().abs().sum());
            previous_transformation_ = transformation_;
        }
        else
            PCL_DEBUG("[pcl::%s::computeTransformation] Convergence failed\n",
                        getClassName().c_str());
    }
    final_transformation_ = previous_transformation_ * guess;
  
    PCL_DEBUG("Transformation "
             "is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%"
             "5f\t%5f\t%5f\t%5f\n",
             final_transformation_(0, 0),
             final_transformation_(0, 1),
             final_transformation_(0, 2),
             final_transformation_(0, 3),
             final_transformation_(1, 0),
             final_transformation_(1, 1),
             final_transformation_(1, 2),
             final_transformation_(1, 3),
             final_transformation_(2, 0),
             final_transformation_(2, 1),
             final_transformation_(2, 2),
             final_transformation_(2, 3),
             final_transformation_(3, 0),
             final_transformation_(3, 1),
             final_transformation_(3, 2),
             final_transformation_(3, 3));
  
   // Transform the point cloud
   pcl::transformPointCloud(*input_, output, final_transformation_);
 }
  
 template <typename PointSource, typename PointTarget>
 void
 GeneralizedIterativeClosestPoint<PointSource, PointTarget>::applyState(
     Eigen::Matrix4f& t, const Vector6d& x) const
 {
   // Z Y X euler angles convention
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(static_cast<float>(x[5]), Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(static_cast<float>(x[4]), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(static_cast<float>(x[3]), Eigen::Vector3f::UnitX());
    t.topLeftCorner<3, 3>().matrix() = R * t.topLeftCorner<3, 3>().matrix();
    Eigen::Vector4f T(static_cast<float>(x[0]),
                        static_cast<float>(x[1]),
                        static_cast<float>(x[2]),
                        0.0f);
    t.col(3) += T;
}
```

```cpp
template <typename PointSource, typename PointTarget, typename Scalar>
 bool
 Registration<PointSource, PointTarget, Scalar>::initComputeReciprocal()
{
    if (!input_) {
        PCL_ERROR("[pcl::registration::%s::compute] No input source dataset was given!\n",
                getClassName().c_str());
        return (false);
    }

    if (source_cloud_updated_ && !force_no_recompute_reciprocal_) {
        tree_reciprocal_->setInputCloud(input_);
        source_cloud_updated_ = false;
    }
    return (true);
}
```
https://pointclouds.org/documentation/gicp_8h_source.html#l00106


```cpp
{
    min_number_correspondences_ = 4;
    reg_name_ = "GeneralizedIterativeClosestPoint";
    max_iterations_ = 200;
    transformation_epsilon_ = 5e-4;
    corr_dist_threshold_ = 5.;
    rigid_transformation_estimation_ = [this](const PointCloudSource& cloud_src,
                                            const pcl::Indices& indices_src,
                                            const PointCloudTarget& cloud_tgt,
                                            const pcl::Indices& indices_tgt,
                                            Eigen::Matrix4f& transformation_matrix) {
    estimateRigidTransformationBFGS(
        cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
    };
}
```

```cpp

    template <typename PointSource, typename PointTarget>
    void
    GeneralizedIterativeClosestPoint<PointSource, PointTarget>::
        estimateRigidTransformationBFGS(const PointCloudSource& cloud_src,
                                        const pcl::Indices& indices_src,
                                        const PointCloudTarget& cloud_tgt,
                                        const pcl::Indices& indices_tgt,
                                        Eigen::Matrix4f& transformation_matrix)
    {
    // need at least min_number_correspondences_ samples
    if (indices_src.size() < min_number_correspondences_) {
        PCL_THROW_EXCEPTION(
            NotEnoughPointsException,
            "[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need "
            "at least "
                << min_number_correspondences_
                << " points to estimate a transform! "
                "Source and target have "
                << indices_src.size() << " points!");
        return;
    }
    // Set the initial solution
    Vector6d x = Vector6d::Zero();
    // translation part
    x[0] = transformation_matrix(0, 3);
    x[1] = transformation_matrix(1, 3);
    x[2] = transformation_matrix(2, 3);
    // rotation part (Z Y X euler angles convention)
    // see: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
    x[3] = std::atan2(transformation_matrix(2, 1), transformation_matrix(2, 2));
    x[4] = asin(-transformation_matrix(2, 0));
    x[5] = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));

    // Set temporary pointers
    tmp_src_ = &cloud_src;
    tmp_tgt_ = &cloud_tgt;
    tmp_idx_src_ = &indices_src;
    tmp_idx_tgt_ = &indices_tgt;

    // Optimize using BFGS
    OptimizationFunctorWithIndices functor(this);
    BFGS<OptimizationFunctorWithIndices> bfgs(functor);
    bfgs.parameters.sigma = 0.01;
    bfgs.parameters.rho = 0.01;
    bfgs.parameters.tau1 = 9;
    bfgs.parameters.tau2 = 0.05;
    bfgs.parameters.tau3 = 0.5;
    bfgs.parameters.order = 3;

    int inner_iterations_ = 0;
    int result = bfgs.minimizeInit(x);
    result = BFGSSpace::Running;
    do {
        inner_iterations_++;
        result = bfgs.minimizeOneStep(x);
        if (result) {
        break;
        }
        result = bfgs.testGradient();
    } while (result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
    if (result == BFGSSpace::NoProgress || result == BFGSSpace::Success ||
        inner_iterations_ == max_inner_iterations_) {
        PCL_DEBUG("[pcl::registration::TransformationEstimationBFGS::"
                "estimateRigidTransformation]");
        PCL_DEBUG("BFGS solver finished with exit code %i \n", result);
        transformation_matrix.setIdentity();
        applyState(transformation_matrix, x);
    }
    else
        PCL_THROW_EXCEPTION(
            SolverDidntConvergeException,
            "[pcl::" << getClassName()
                    << "::TransformationEstimationBFGS::estimateRigidTransformation] BFGS "
                        "solver didn't converge!");
    }


template <typename PointSource, typename PointTarget>
inline double
GeneralizedIterativeClosestPoint<PointSource, PointTarget>::
    OptimizationFunctorWithIndices::operator()(const Vector6d& x)
{
Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
gicp_->applyState(transformation_matrix, x);
double f = 0;
int m = static_cast<int>(gicp_->tmp_idx_src_->size());
for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    Eigen::Vector4f p_trans_src(transformation_matrix * p_src);
    // Estimate the distance (cost function)
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                    p_trans_src[1] - p_tgt[1],
                    p_trans_src[2] - p_tgt[2]);
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // increment= d'*Md/num_matches = d'*M*d/num_matches (we postpone
    // 1/num_matches after the loop closes)
    f += double(d.transpose() * Md);
}
return f / m;
}

template <typename PointSource, typename PointTarget>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget>::
    OptimizationFunctorWithIndices::df(const Vector6d& x, Vector6d& g)
{
Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
gicp_->applyState(transformation_matrix, x);
// Zero out g
g.setZero();
// Eigen::Vector3d g_t = g.head<3> ();
// the transpose of the derivative of the cost function w.r.t rotation matrix
Eigen::Matrix3d dCost_dR_T = Eigen::Matrix3d::Zero();
int m = static_cast<int>(gicp_->tmp_idx_src_->size());
for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();

    Eigen::Vector4f p_trans_src(transformation_matrix * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                    p_trans_src[1] - p_tgt[1],
                    p_trans_src[2] - p_tgt[2]);
    // Md = M*d
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*d/num_matches (we postpone 2/num_matches after the loop
    // closes)
    g.head<3>() += Md;
    // Increment rotation gradient
    p_trans_src = gicp_->base_transformation_ * p_src;
    Eigen::Vector3d p_base_src(p_trans_src[0], p_trans_src[1], p_trans_src[2]);
    dCost_dR_T += p_base_src * Md.transpose();
}
g.head<3>() *= 2.0 / m;
dCost_dR_T *= 2.0 / m;
gicp_->computeRDerivative(x, dCost_dR_T, g);
}

template <typename PointSource, typename PointTarget>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget>::
    OptimizationFunctorWithIndices::fdf(const Vector6d& x, double& f, Vector6d& g)
{
Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
gicp_->applyState(transformation_matrix, x);
f = 0;
g.setZero();
// the transpose of the derivative of the cost function w.r.t rotation matrix
Eigen::Matrix3d dCost_dR_T = Eigen::Matrix3d::Zero();
const int m = static_cast<int>(gicp_->tmp_idx_src_->size());
for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    Eigen::Vector4f p_trans_src(transformation_matrix * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                    p_trans_src[1] - p_tgt[1],
                    p_trans_src[2] - p_tgt[2]);
    // Md = M*d
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // Increment total error
    f += double(d.transpose() * Md);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*d/num_matches (we postpone 2/num_matches after the loop
    // closes)
    g.head<3>() += Md;
    p_trans_src = gicp_->base_transformation_ * p_src;
    Eigen::Vector3d p_base_src(p_trans_src[0], p_trans_src[1], p_trans_src[2]);
    // Increment rotation gradient
    dCost_dR_T += p_base_src * Md.transpose();
}
f /= double(m);
g.head<3>() *= double(2.0 / m);
dCost_dR_T *= 2.0 / m;
gicp_->computeRDerivative(x, dCost_dR_T, g);
}

}
```


---

