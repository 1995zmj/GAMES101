#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
	auto& p_0 = points[0];
	auto& p_1 = points[1];
	auto& p_2 = points[2];
	auto& p_3 = points[3];

	int count = 0;
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
			3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;
		
		//window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
		auto x = point.y;
		auto y = point.x;

		int minx = std::floor(x);
		int maxx = std::ceil(x);
		int miny = std::floor(y);
		int maxy = std::ceil(y);

		cv::Point2f cp(x,y);
		cv::Point2f cp1(minx + 0.5f, miny + 0.5f);
		cv::Point2f cp2(maxx + 0.5f, miny + 0.5f);
		cv::Point2f cp3(minx + 0.5f, maxy + 0.5f);
		cv::Point2f cp4(maxx + 0.5f, maxy + 0.5f);

		auto r1 = (cp1 - cp);
		auto at =  1.0f - r1.dot(r1);
		at = at < 0 ? 0 : at;

		auto r2 = (cp2 - cp);
		auto bt = 1.0f - r2.dot(r2);
		bt = bt < 0 ? 0 : bt;

		auto r3 = (cp3 - cp);
		auto ct = 1.0f - r3.dot(r3);
		ct = ct < 0 ? 0 : ct;

		auto r4 = (cp4 - cp);
		auto dt = 1.0f - r4.dot(r4);
		dt = dt < 0 ? 0 : dt;

		float a = window.at<cv::Vec3b>(cp1.x, cp1.y)[2];
		float b = window.at<cv::Vec3b>(cp2.x, cp2.y)[2];
		float c = window.at<cv::Vec3b>(cp3.x, cp3.y)[2];
		float d = window.at<cv::Vec3b>(cp4.x, cp4.y)[2];

		window.at<cv::Vec3b>(cp1.x, cp1.y)[2] = std::max(a, at*255);
		window.at<cv::Vec3b>(cp2.x, cp2.y)[2] = std::max(b, bt * 255);
		window.at<cv::Vec3b>(cp3.x, cp3.y)[2] = std::max(c, ct * 255);
		window.at<cv::Vec3b>(cp4.x, cp4.y)[2] = std::max(d, dt * 255);
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
	// TODO: Implement de Casteljau's algorithm
	if (control_points.size() == 1) return control_points[0];

	std::vector<cv::Point2f> Points;
	for (int i = 0; i < control_points.size() - 1; i++)
	{
		Points.emplace_back(control_points[i] * (1 - t) + t * control_points[i + 1]);
	}
	
	return recursive_bezier(Points, t);

}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.


	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = recursive_bezier(control_points, t);
		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
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
		for (auto& point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == 4)
		{
			naive_bezier(control_points, window);
			//bezier(control_points, window);

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
