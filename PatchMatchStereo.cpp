//
// Created by ZZK on 2022/7/16.
//

#include "PatchMatchStereo.h"

#define SAFE_DELETE(P) if(P!=nullptr){delete[](P); (P)=nullptr;}

PatchMatchStereo::PatchMatchStereo() : _width(0), _height(0), _imgLeft(nullptr), _imgRight(nullptr),
                                       _grayLeft(nullptr), _grayRight(nullptr),
                                       _gradLeft(nullptr), _gradRight(nullptr),
                                       _costLeft(nullptr), _costRight(nullptr),
                                       _dispLeft(nullptr), _dispRight(nullptr),
                                       _planeLeft(nullptr), _planeRight(nullptr),
                                       _isInitialized(false) {

}

PatchMatchStereo::~PatchMatchStereo() {
    release();
}

bool PatchMatchStereo::initialize(const sint32 &width, const sint32 &height, const PMSOption &option) {
    // ··· 赋值

    // 影像尺寸
    _width = width;
    _height = height;
    // PMS参数
    _option = option;

    if (width <= 0 || height <= 0) {
        return false;
    }

    //··· 开辟内存空间
    const sint32 size = width * height;
    // 灰度数据
    _grayLeft = new uint8[size];
    _grayRight = new uint8[size];
    // 梯度数据
    _gradLeft = new PGradient[size];
    _gradRight = new PGradient[size];
    // 代价数据
    _costLeft = new float32[size];
    _costRight = new float32[size];
    // 视差图
    _dispLeft = new float32[size];
    _dispRight = new float32[size];
    // 平面集
    _planeLeft = new DisparityPlane[size];
    _planeRight = new DisparityPlane[size];

    _isInitialized = _grayLeft && _grayRight &&
                     _gradLeft && _gradRight &&
                     _costLeft && _costRight &&
                     _dispLeft && _dispRight &&
                     _planeLeft && _planeRight;

    return _isInitialized;
}

void PatchMatchStereo::release() {
    SAFE_DELETE(_grayLeft);
    SAFE_DELETE(_grayRight);
    SAFE_DELETE(_gradLeft);
    SAFE_DELETE(_gradRight);
    SAFE_DELETE(_costLeft);
    SAFE_DELETE(_costRight);
    SAFE_DELETE(_dispLeft);
    SAFE_DELETE(_dispRight);
    SAFE_DELETE(_planeLeft);
    SAFE_DELETE(_planeRight);
}

bool PatchMatchStereo::match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft, float32 *dispRight) {
    if (!_isInitialized) {
        return false;
    }
    if (imgLeft == nullptr || imgRight == nullptr) {
        return false;
    }

    _imgLeft = imgLeft;
    _imgRight = imgRight;

    // 随机初始化
    randomInitialization();
    // 计算灰度图
    computeGray();
    // 计算梯度图
    computeGradient();

    // 迭代传播
    propagation();

    // 平面转换成视差
    planeToDisparity();

    // 左右一致性检查
    if (_option._isCheckLR) {
        // 一致性检查
        // lrCheck();
    }
    // 视差填充
    if (_option._isFillHoles) {
        //	FillHolesInDispMap();
    }

    // 输出视差图
    if (_dispLeft && dispLeft) {
        memcpy(dispLeft, _dispLeft, _height * _width * sizeof(float32));
    }
    if (_dispRight && dispRight) {
        memcpy(dispRight, _dispRight, _height * _width * sizeof(float32));
    }

    return true;
}

void PatchMatchStereo::randomInitialization() {
    const sint32 width = _width;
    const sint32 height = _height;


    if (width <= 0 || height <= 0 ||
        _dispLeft == nullptr || _dispRight == nullptr ||
        _planeLeft == nullptr || _planeRight == nullptr) {
        return;
    }

    const PMSOption &option = _option;
    const sint32 minDisparity = option._minDisparity;
    const sint32 maxDisparity = option._maxDisparity;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float32> rand_d(static_cast<float32>(minDisparity),
                                                   static_cast<float32>(maxDisparity));
    std::uniform_real_distribution<float32> rand_n(-1.0f, 1.0f);

    for (sint32 k = 0; k < 2; k++) {
        float32 *dispPtr = k == 0 ? _dispLeft : _dispRight;
        DisparityPlane *planePtr = k == 0 ? _planeLeft : _planeRight;
        float32 sign = (k == 0) ? 1.0f : -1.0f;

        for (sint32 y = 0; y < _height; ++y) {
            for (sint32 x = 0; x < _width; ++x) {
                const sint32 p = y * width + x;;

                // 随机视差值
                float32 disp = sign * rand_d(gen);
                if (option._isIntegerDisp) {
                    disp = static_cast<float32>(round(disp));
                }
                dispPtr[p] = disp;

                // 随机法向量
                PVector3f norm;
                if (!option._isForceFpw) {
                    norm._x = rand_n(gen);
                    norm._y = rand_n(gen);
                    float32 z = rand_n(gen);
                    while (z == 0.0f) {
                        z = rand_n(gen);
                    }
                    norm._z = z;
                    norm.normalize();
                } else {
                    norm._x = 0.0f;
                    norm._y = 0.0f;
                    norm._z = 1.0f;
                }

                // 计算视差平面
                planePtr[p] = DisparityPlane(x, y, norm, disp);
            }
        }
    }
}

void PatchMatchStereo::computeGray() {
    const sint32 width = _width;
    const sint32 height = _height;
    if (width <= 0 || height <= 0 ||
        _imgLeft == nullptr || _imgRight == nullptr ||
        _grayLeft == nullptr || _grayRight == nullptr) {
        return;
    }

    // 彩色转灰度
    for (sint32 n = 0; n < 2; n++) {
        auto *color = (n == 0) ? _imgLeft : _imgRight;
        auto *gray = (n == 0) ? _grayLeft : _grayRight;
        for (sint32 i = 0; i < height; i++) {
            for (sint32 j = 0; j < width; j++) {
                const auto b = color[i * (width * 3) + (j * 3)];
                const auto g = color[i * (width * 3) + (j * 3) + 1];
                const auto r = color[i * (width * 3) + (j * 3) + 2];
                gray[i * width + j] = uint8(r * 0.299 + g * 0.587 + b * 0.114);
            }
        }
    }
}

void PatchMatchStereo::computeGradient() {
    const sint32 width = _width;
    const sint32 height = _height;
    if (width <= 0 || height <= 0 ||
        _gradLeft == nullptr || _gradRight == nullptr ||
        _grayLeft == nullptr || _grayRight == nullptr) {
        return;
    }

    // Sobel梯度算子
    for (sint32 n = 0; n < 2; n++) {
        auto *gray = (n == 0) ? _grayLeft : _grayRight;
        auto *grad = (n == 0) ? _gradLeft : _gradRight;
        for (sint32 y = 1; y < height - 1; y++) {
            for (sint32 x = 1; x < width - 1; x++) {
                const auto grad_x =
                        (-gray[(y - 1) * width + x - 1] + gray[(y - 1) * width + x + 1]) +
                        (-2 * gray[y * width + x - 1] + 2 * gray[y * width + x + 1]) +
                        (-gray[(y + 1) * width + x - 1] + gray[(y + 1) * width + x + 1]);
                const auto grad_y =
                        (
                                -gray[(y - 1) * width + x - 1]
                                - 2 * gray[(y - 1) * width + x]
                                - gray[(y - 1) * width + x + 1]
                        ) +
                        (
                                gray[(y + 1) * width + x - 1] +
                                2 * gray[(y + 1) * width + x] +
                                gray[(y + 1) * width + x + 1]
                        );
                grad[y * width + x]._x = sint16(grad_x);
                grad[y * width + x]._y = sint16(grad_y);
            }
        }
    }
}

void PatchMatchStereo::propagation() {
    const sint32 width = _width;
    const sint32 height = _height;
    if (width <= 0 || height <= 0 ||
        _imgLeft == nullptr || _imgRight == nullptr ||
        _grayLeft == nullptr || _grayRight == nullptr ||
        _dispLeft == nullptr || _dispRight == nullptr ||
        _planeLeft == nullptr || _planeRight == nullptr) {
        return;
    }

    // 左右视图匹配参数
    const auto optionLeft = _option;
    auto optionRight = _option;

    optionRight._minDisparity = -optionLeft._maxDisparity;
    optionRight._maxDisparity = -optionLeft._minDisparity;

    // 左右视图传播实例
    PMSPropagation propaLeft(optionLeft,
                             width, height,
                             _imgLeft, _imgRight,
                             _gradLeft, _gradRight,
                             _planeLeft, _planeRight,
                             _costLeft, _costRight,
                             _dispLeft);
    PMSPropagation propaRight(optionRight,
                              width, height,
                              _imgRight, _imgLeft,
                              _gradRight, _gradLeft,
                              _planeRight, _planeLeft,
                              _costRight, _costLeft,
                              _dispRight);

    // 迭代传播
    for (sint32 k = 0; k < _option._numIters; k++) {
        propaLeft.doPropagation();
        propaRight.doPropagation();
    }
}

void PatchMatchStereo::planeToDisparity() {
    const sint32 width = _width;
    const sint32 height = _height;
    if (width <= 0 || height <= 0 ||
        _dispLeft == nullptr || _dispRight == nullptr ||
        _planeLeft == nullptr || _planeRight == nullptr) {
        return;
    }
    for (int k = 0; k < 2; k++) {
        auto *planePtr = (k == 0) ? _planeLeft : _planeRight;
        auto *dispPtr = (k == 0) ? _dispLeft : _dispRight;
        for (sint32 y = 0; y < height; y++) {
            for (sint32 x = 0; x < width; x++) {
                const sint32 p = y * width + x;
                const auto &plane = planePtr[p];
                dispPtr[p] = plane.getDisparity(x, y);
            }
        }
    }
}
