#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(const Eigen::Vector3f &axis, float angle) {
	Eigen::Vector3f norm_axis = axis.normalized();
	Eigen::Vector3f &n = norm_axis;
	angle = angle / 180 * MY_PI;

	Eigen::Matrix4f tmp_matrix[3];
	tmp_matrix[0] << Eigen::Matrix4f::Identity();
	tmp_matrix[1] << n.x() * n.x(), n.x() * n.y(), n.x() * n.z(), 0, 
					n.x() * n.y(), n.y() * n.y(), n.y() * n.z(), 0, 
					n.x() * n.z(), n.y() * n.z(), n.z() * n.z(), 0,
					0, 0, 0, 1;

	tmp_matrix[2] << 0, -n.z(), n.y(), 0,
					n.z(), 0, -n.x(), 0,
					-n.y(), n.x(), 0, 0,
					0, 0, 0, 1;

	Eigen::Matrix4f rotation = std::cos(angle) * tmp_matrix[0]
							+ (1 - std::cos(angle)) * tmp_matrix[1]
							+ std::sin(angle) * tmp_matrix[2];

	return rotation;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
//    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
		
//	float angle = rotation_angle / 180 * MY_PI;
//	model << std::cos(angle), -std::sin(angle), 0, 0, 
//		std::sin(angle), std::cos(angle), 0, 0,
//		0, 0, 1, 0,
//		0, 0, 0, 1;
	Eigen::Matrix4f model = get_rotation(Eigen::Vector3f(0, 1, 0), rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

	float t = zNear * std::tan(eye_fov / 180 * MY_PI / 2);
	float r = t * aspect_ratio;

	Eigen::Matrix4f right_to_left_hand, persp_to_ortho, ortho;
	right_to_left_hand << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, -1, 0,
						0, 0, 0, 1;
		
	persp_to_ortho << zNear, 0, 0, 0,
				   0, zNear, 0, 0, 
				   0, 0, zNear + zFar, -zNear * zFar, 
				   0, 0, 1, 0;

	ortho << 1 / r, 0, 0, 0,
		  0, 1 / t, 0, 0,
		  0, 0, 2 / (zFar - zNear), (zNear + zFar) / (zNear - zFar),
		  0, 0, 0, 1;

	projection = ortho * persp_to_ortho * right_to_left_hand;	

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default, namely anti-clockwire
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
