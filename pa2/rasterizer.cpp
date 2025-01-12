// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <limits>
#include <iostream>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
			// ???
			// Why does the same thing three times?
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }

	std::cout << "draw" << std::endl;
}

static bool insideTriangle(float x, float y, const Eigen::Vector3f *vertices) {
	int count = 0;
	Eigen::Vector3f tmp_v1, tmp_v2;
	for (int i = 0; i < 3; i++) {
		tmp_v1 = vertices[(i + 1) % 3] - vertices[i];
		tmp_v2 = Eigen::Vector3f(x, y, 0) - vertices[i];
		tmp_v1[2] = tmp_v2[2] = 0;
		count += tmp_v1.cross(tmp_v2).z() > 0 ? 1 : -1;
	}

	return std::abs(count) == 3;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
	float lx, ly, ux, uy;
	lx = ly = std::numeric_limits<float>::max();
	ux = uy = std::numeric_limits<float>::min();

	for (auto &vertex : v) {
		lx = std::min(lx, vertex.x());
		ly = std::min(ly, vertex.y());

		ux = std::max(ux, vertex.x());
		uy = std::max(uy, vertex.y());
	}

	// DEBUG
//	std::cout << "lx ux ly uy is " << lx << " " << ux << " " << ly << " " << uy << std::endl;
	// DEBUG

	std::vector<float> dx{0.25, 0.25, 0.75, 0.75}, dy{0.25, 0.75, 0.25, 0.75};
	for (int x = (int)(lx); x <= (int)(ux); x++) {
		for (int y = (int)(ly); y <= (int)(uy); y++) {
			// DEBUG
			// std::cout << "(x, y) is " << x << " " << y << ": \n";
			// DEBUG

			// DEBUG
//			int debug = 0;
//			if (x == 4 && y == 6) debug = 1;
//			else debug = 0;
			// DEBUG

			Eigen::Vector3f color_pixel{0, 0, 0};
			for (int k = 0; k < 4; k++) {
				float px = x + dx[k], py = y + dy[k];

				// index of the pixel in depth sample buffer
				int ind = get_index(x, y) * 4 + k;

				// DEBUG
//				if (debug) {
//					std::cout << "before depth_sample[" << ind << "] is " << depth_sample[ind] << std::endl;
//					std::cout << "before frame_sample[" << ind << "] is " << frame_sample[ind] << std::endl;
//				}
				// DEBUG

				if (insideTriangle(px, py, t.v)) {
					auto[alpha, beta, gamma] = computeBarycentric2D(px, py, t.v);
					float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
	
					// DEBUG
//					if (debug) std::cout << "k is " << k << " in triangle" << std::endl;
					// DEBUG

					if (z_interpolated < depth_sample[ind]) {
						// DEBUG
//						if (debug) std::cout << "update frame_sample" << std::endl;
						// DEBUG

						depth_sample[ind] = z_interpolated;
						frame_sample[ind] = t.getColor() / 4;
					}


				}

				// DEBUG
				// std::cout << "sample ind is " << ind << std::endl;
				// DEBUG
				
				// DEBUG
				// if (debug) {
//					std::cout << "after depth_sample[" << ind << "] is " << depth_sample[ind] << std::endl;
//					std::cout << "after frame_sample[" << ind << "] is " << frame_sample[ind] << std::endl;
//				}
				// DEBUG
			
				// DEBUG
				// std::cout << "frame_sample[" << ind << "] is " << frame_sample[ind] << std::endl;
				// std::cout << "depth_sample[" << ind << "] is " << depth_sample[ind] << std::endl;
				// DEBUG

				color_pixel += frame_sample[ind];
			}

			// DEBUG
			// if (debug) std::cout << "color_pixel is " << color_pixel << std::endl;
			// DEBUG

			// DEBUG
			// if (debug) std::cout << "before: frame_buf[" << get_index(x, y) << "] is " << frame_buf[get_index(x, y)] << std::endl;
			// DEBUG

			// DEBUG
//			std::cout << "(" << x << ", " << y << "): " << std::endl;
//			std::cout << "before frame_buf is " << frame_buf[get_index(x, y)] << std::endl;
			// DEBUG

			frame_buf[get_index(x, y)] = color_pixel;

			// DEBUG
//			std::cout << "after frame_buf is " << frame_buf[get_index(x, y)] << std::endl;
			// DEBUG

			// DEBUG
			// if (debug) std::cout << "after: frame_buf[" << get_index(x, y) << "] is " << frame_buf[get_index(x, y)] << std::endl;
			// if (debug) std::cout << "\n\n" << std::endl;
			// DEBUG

		}
	}

	// DEBUG
	// std::cout << "breakpoint frame_buffer assigment" << std::endl;
	// std::cout << "\n\n\n\n" << std::endl;
	std::cout << "rasterize_triangle finished" << std::endl;
	// DEBUG
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
		std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

	frame_sample.resize(w * h * 5);
	depth_sample.resize(w * h * 5);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
