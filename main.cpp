#include "PMSType.h"
#include "PatchMatchStereo.h"
#include <opencv2/opencv.hpp>

void dispMatNorm(const sint32 &width, const sint32 &height, const float32 *dispMap, cv::Mat &dispMat) {
    float32 minDisp = FLT_MAX, maxDisp = FLT_MIN;
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = dispMap[i * width + j];
            if (disp != Invalid_Float) {
                minDisp = std::min(minDisp, disp);
                maxDisp = std::max(maxDisp, disp);
            }
        }
    }
    for (uint32 i = 0; i < height; i++) {
        for (uint32 j = 0; j < width; j++) {
            const float32 disp = dispMap[i * width + j];
            if (disp == Invalid_Float) {
                dispMat.data[i * width + j] = 0;
            } else {
                dispMat.data[i * width + j] = static_cast<uchar>((disp - minDisp) / (maxDisp - minDisp) * 255);
            }
        }
    }
}

int main() {
    // ··· 读取影像
    std::string pathLeft = R"(..\data\cone\im2.png)";
    std::string pathRight = R"(..\data\cone\im6.png)";

    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_GRAYSCALE);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_GRAYSCALE);

    if (imgLeft.data == nullptr || imgRight.data == nullptr) {
        std::cout << "读取影像失败！" << std::endl;
        return -1;
    }
    if (imgLeft.rows != imgRight.rows || imgLeft.cols != imgRight.cols) {
        std::cout << "左右影像尺寸不一致！" << std::endl;
        return -1;
    }

    // ··· SGM匹配
    const auto width = static_cast<sint32>(imgLeft.cols);
    const auto height = static_cast<sint32>(imgRight.rows);

    PMSOption psmOption;
    // 候选视差范围
    psmOption._minDisparity = 0;
    psmOption._maxDisparity = 64;


    PatchMatchStereo pms;
    // 初始化
    if (!pms.initialize(width, height, psmOption)) {
        std::cout << "SGM初始化失败！" << std::endl;
        return -2;
    }

    // 匹配
    auto *disLeft = new float32[width * height]();
    auto *disRight = new float32[width * height]();
    if (!pms.match(imgLeft.data, imgRight.data, disLeft, disRight)) {
        std::cout << "SGM匹配失败！" << std::endl;
        return -2;
    }

    // 显示视差图
    cv::Mat dispMatLeft = cv::Mat(height, width, CV_8UC1);
    cv::Mat dispMatRight = cv::Mat(height, width, CV_8UC1);
    cv::Mat dispMat = cv::Mat(height, width * 2, CV_8UC1);

    dispMatNorm(width, height, disLeft, dispMatLeft);
    dispMatNorm(width, height, disRight, dispMatRight);
    cv::hconcat(dispMatLeft, dispMatRight, dispMat);

    cv::imwrite("../dispMatLeft.png", dispMatLeft);
    cv::imshow("dispMatLeft", dispMatLeft);
    cv::imwrite("../dispMatRight.png", dispMatRight);
    cv::imshow("dispMatRight", dispMatRight);
    cv::imshow("dispMat", dispMat);
    cv::waitKey(0);

    delete[] disLeft;
    delete[] disRight;
    disLeft = nullptr;
    disRight = nullptr;

    return 0;
}
