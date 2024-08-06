#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    //change angle to radient
    rotation_angle = rotation_angle / 180 * MY_PI;
    float cos_angle = cos(rotation_angle);
    float sin_angle = sin(rotation_angle);
    // Then return it.
    model << cos_angle, -sin_angle, 0, 0,
             sin_angle, cos_angle, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    float t = tan(eye_fov/2/180*MY_PI) * abs(zNear);
    float r = t * aspect_ratio;
    projection << zNear / r, 0, 0, 0,
                  0, zNear / t, 0, 0,
                  0, 0, (zNear + zFar) / (zNear - zFar), 2*zNear*zFar / (zNear - zFar),
                  0, 0, -1, 0;
    // Then return it.
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    angle = angle / 180 * MY_PI;
    float cos_angle = cos(angle);
    float sin_angle = sin(angle);
    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    //using Rodrigues' rotation formula
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f dualMat;
    dualMat << 0, -z, y,
               z, 0, -x,
               -y, x, 0;
    rotation.block<3,3>(0,0) = (cos_angle*I) + (1-cos_angle)*axis*axis.transpose() + sin_angle*dualMat;
    return rotation;
}

#pragma warning(push)
#pragma warning(disable: 4819)


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 10};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //rotate by any axis
        r.set_model(get_rotation(Vector3f(0, 0, 1), angle));

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

        // rotate by any axis
        r.set_model(get_rotation(Vector3f(0, 1, 0).normalized(), angle));

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
