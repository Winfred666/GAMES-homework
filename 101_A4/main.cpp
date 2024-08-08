#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

const int CONTROL_POINTS = 5;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < CONTROL_POINTS) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // copy the control_points vector to a new vector
    std::vector<cv::Point2f> new_control_points = control_points;
    int n = new_control_points.size(); // it is a 4 point curve, cubic Bézier
    // calculate smaller set of control_points recursively
    for(int q = 0; q < n-1; q++){
        for(int i = 0; i < n-1-q; i++){
            cv::Point2f point = new_control_points[i] + t * (new_control_points[i+1] - new_control_points[i]);
            //delete the point that was used to calculate the new point
            new_control_points[i] = point;
        }
    }
    return new_control_points[0];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    const float step = 0.001;
    const float thicksqr = 5.0; // the radius of the point
    float t = 0.0;
    while (t <= 1.0) 
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        // do anti-aliasing, pixel center that close to the curve should be colored
        int base_x = std::round(point.x-0.5);
        int base_y = std::round(point.y-0.5);

        for (int i = -thicksqr; i <= thicksqr; i++) {
            for (int j = -thicksqr; j <= thicksqr; j++) {
                float dist = (base_y+i+0.5 - point.y) * (base_y+i+0.5 - point.y) + 
                                (base_x+j+0.5 - point.x) * (base_x+j+0.5 - point.x);
                if (dist < thicksqr) {
                    float weight = 1 - dist / thicksqr;
                    if(weight > 0.8) weight = 1; //the point is already close to curve
                    float old_pixel = window.at<cv::Vec3b>(base_y + i, base_x + j)[1];
                    if (old_pixel < 255 * weight)
                        window.at<cv::Vec3b>(base_y + i, base_x + j)[1] = 255 * weight; // draw the green point where the curve is
                }
            }
        }
        t += step;
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == CONTROL_POINTS) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
