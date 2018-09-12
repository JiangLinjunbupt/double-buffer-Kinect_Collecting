#pragma once
#include"Types.h"

class Camera {
private:

	int _width;
	int _height;
	float _focal_length_x = nan();
	float _focal_length_y = nan();
	float centerx;
	float centery;
public:
	Camera();
public:
	int width() const { return _width; }
	int height() const { return _height; }
	float focal_length_x() const { return _focal_length_x; }
	float focal_length_y() const { return _focal_length_y; }
	/// @}

public:
	Matrix_2x3 projection_jacobian(const Vector3& p);
public:
	Vector3 world_to_depth_image(const Vector3& wrld);
	Vector3 depth_to_world(float i, float j, float depth);
};