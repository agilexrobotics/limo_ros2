#include <string.h>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

enum TLState {
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
};

enum TLType {
  regular = 0,
  five_lights = 1,
  four_lights = 2,
};

int DetectColor(cv::Mat image) {
  cv::Mat hsv_img;
  cv::cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);

  cv::Mat red_thresh, red_thresh1, red_thresh2, yellow_thresh, green_thresh;

  cv::Scalar red_min(0, 5, 150);
  cv::Scalar red_max(8, 255, 255);
  cv::inRange(hsv_img, red_min, red_max, red_thresh1);

  cv::Scalar red_min2(175, 5, 150);
  cv::Scalar red_max2(180, 255, 255);
  cv::inRange(hsv_img, red_min2, red_max2, red_thresh2);

  red_thresh = red_thresh1 + red_thresh2;

  cv::Scalar yellow_min(20, 5, 150);
  cv::Scalar yellow_max(30, 255, 255);
  cv::inRange(hsv_img, yellow_min, yellow_max, yellow_thresh);

  cv::Scalar green_min(35, 5, 150);
  cv::Scalar green_max(90, 255, 255);
  cv::inRange(hsv_img, green_min, green_max, green_thresh);

  cv::Mat red_blur, yellow_blur, green_blur;
  cv::medianBlur(red_thresh, red_blur, 5);
  cv::medianBlur(yellow_thresh, yellow_blur, 5);

  int red, yellow, green;
  red = cv::countNonZero(red_blur);
  yellow = cv::countNonZero(yellow_blur);
  green = cv::countNonZero(green_blur);

  int light_color = std::max({red, yellow, green});

  int result = 0;
  if (light_color > 60) {
    if (light_color == red) {
      result = 1;
    } else if (light_color == yellow) {
      result = 2;
    } else if (light_color == green) {
      result = 3;
    }
  } else {
    result = 0;
  }
  return result;
}

cv::Mat ImgResize(cv::Mat image, float height,
                  cv::InterpolationFlags inter = cv::INTER_AREA) {
  int w = image.cols;
  int h = image.rows;
  float ratio = height / (float)h;
  cv::Mat resized;
  cv::Size dim((int)(w * ratio), height);
  cv::resize(image, resized, dim, 0, 0, inter);
  return resized;
}

int DetectState(cv::Mat image, TLType type = TLType::regular) {
  image = ImgResize(image, 200);
return 0;
  int height = image.rows;
  int width = image.cols;
  cv::Mat output = image;

  cv::Mat gray, circles;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 20, 50, 30, 15, 30);

  int overall_state = 0;
  int state_arrow = 0;
  int state_solid = 0;

  int x = circles.at<int>(0);
  int y = circles.at<int>(1);
  int radius = circles.at<int>(2);

  cv::Rect rect(x - radius, y - radius, radius * 2, radius * 2);
  cv::Mat roi(image, rect);

  int color = DetectColor(roi);

  if (color > 0) {
    state_solid = color;
  }

  if (type == 1) {
    overall_state = state_arrow + state_solid + 1;
  } else if (type == 2) {
    overall_state = state_arrow + 7;
  } else {
    overall_state = state_solid;
  }

  return overall_state;
}
int main(int argc, char* argv[]) {
  std::string path = "/opt/ros2_ws/src/drivers/limo_ros2/limo_base/images/red.jpg";
  cv::Mat img = cv::imread(path);

  cv::imshow("img", img);
  cv::waitKey(0);


  DetectState(img);

  printf("finished\n");

  return 0;
}