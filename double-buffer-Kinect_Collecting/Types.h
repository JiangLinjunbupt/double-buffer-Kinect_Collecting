#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

using namespace std;
using namespace Eigen;

typedef std::string String;
typedef unsigned int uint;
typedef int Integer;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector3f Vector3;
typedef Eigen::VectorXf VectorN;

/// More complex matrixes
typedef Eigen::Matrix<float, 2, 3> Matrix_2x3;
typedef Eigen::Matrix<float, 1, Eigen::Dynamic> Matrix_1xN;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> Matrix_2xN;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Matrix_3xN;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2> Matrix_Nx2;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;

typedef Eigen::Hyperplane<float, 3> Plane3;
typedef Eigen::ParametrizedLine<float, 3> Ray3;
typedef Eigen::AlignedBox3f BBox3;
typedef Eigen::Vector2i Vector2i;
typedef Eigen::AlignedBox2i BBox2i;
typedef Eigen::Quaternion<float> Quaternion;

/// Nan for the default type
inline float nan() { return std::numeric_limits<float>::quiet_NaN(); }

