#include "PMSType.h"
#include "PatchMatchStereo.h"
#include <opencv2/opencv.hpp>

void dispMatNorm(const sint32 &width, const sint32 &height, const float32 *dispMap, cv::Mat &dispMat) {
    float32 minDisp = FLT_MAX, maxDisp = FLT_MIN;
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = std::abs(dispMap[i * width + j]);
            if (disp != Invalid_Float) {
                minDisp = std::min(minDisp, disp);
                maxDisp = std::max(maxDisp, disp);
            }
        }
    }
    for (uint32 i = 0; i < height; i++) {
        for (uint32 j = 0; j < width; j++) {
            const float32 disp = std::abs(dispMap[i * width + j]);
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

    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_COLOR);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_COLOR);

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

    // 左右影像的彩色数据
    auto bytesLeft = new uint8[width * height * 3];
    auto bytesRight = new uint8[width * height * 3];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            bytesLeft[i * 3 * width + 3 * j] = imgLeft.at<cv::Vec3b>(i, j)[0];
            bytesLeft[i * 3 * width + 3 * j + 1] = imgLeft.at<cv::Vec3b>(i, j)[1];
            bytesLeft[i * 3 * width + 3 * j + 2] = imgLeft.at<cv::Vec3b>(i, j)[2];
            bytesRight[i * 3 * width + 3 * j] = imgRight.at<cv::Vec3b>(i, j)[0];
            bytesRight[i * 3 * width + 3 * j + 1] = imgRight.at<cv::Vec3b>(i, j)[1];
            bytesRight[i * 3 * width + 3 * j + 2] = imgRight.at<cv::Vec3b>(i, j)[2];
        }
    }
    printf("Done!\n");

    // PMS匹配参数设计
    PMSOption psmOption;
    // patch大小
    psmOption._patchSize = 35;
    // 候选视差范围
    psmOption._minDisparity = 0;
    psmOption._maxDisparity = 64;
    // gamma
    psmOption._gamma = 10.0f;
    // alpha
    psmOption._alpha = 0.9f;
    // t_col
    psmOption._tauCol = 10.0f;
    // t_grad
    psmOption._tauGrad = 2.0f;
    // 传播迭代次数
    psmOption._numIters = 3;
    // 前端平行窗口
    psmOption._isForceFpw = false;
    // 整数视差精度
    psmOption._isIntegerDisp = false;

    PatchMatchStereo pms;
    // 初始化
    if (!pms.initialize(width, height, psmOption)) {
        std::cout << "PMS Initialization Failure!" << std::endl;
        return -2;
    }

    // 匹配
    auto *disLeft = new float32[width * height];
    auto *disRight = new float32[width * height];
    if (!pms.match(bytesLeft, bytesRight, disLeft, disRight)) {
        std::cout << "PMS Matching Failure!" << std::endl;
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
