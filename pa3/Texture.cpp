//
// Created by LEI XU on 4/27/19.
//

#include <iostream>
#include "Texture.hpp"

Eigen::Vector3f Texture::getColorBilinear(float u, float v) {
	float u_point = u * width;
	float v_point = (1 - v) * height;
	
	float u0 = std::max(0, static_cast<int>(u_point - 0.5f)) + 0.5;
	float u1 = std::min(width - 1, static_cast<int>(u_point + 0.5f)) + 0.5;
	float v0 = std::max(0, static_cast<int>(v_point - 0.5f)) + 0.5;
	float v1 = std::min(height - 1, static_cast<int>(v_point + 0.5f)) + 0.5;

	auto color00 = image_data.at<cv::Vec3b>(v0, u0);
	auto color01 = image_data.at<cv::Vec3b>(v1, u0);
	auto color10 = image_data.at<cv::Vec3b>(v0, u1);
	auto color11 = image_data.at<cv::Vec3b>(v1, u1);

	auto s = u_point - u0;
	auto t = v_point - v0;
	auto lerp0 = color00 + (color10 - color00) * s;
	auto lerp1 = color01 + (color11 - color01) * s;
	auto color = lerp0 + (lerp1 - lerp0) * t;

	return Eigen::Vector3f(color[0], color[1], color[2]);
}
