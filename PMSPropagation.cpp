//
// Created by tianhe on 2022/8/10.
//

#include "PMSPropagation.h"

#define SAFE_DELETE(P) if(P!=nullptr){delete(P); (P)=nullptr;}

PMSPropagation::PMSPropagation(const PMSOption &option,
                               const sint32 &width, const sint32 &height,
                               const uint8 *imgLeft, const uint8 *imgRight,
                               const PGradient *gradLeft, const PGradient *gradRight,
                               DisparityPlane *planeLeft, DisparityPlane *planeRight,
                               float32 *costLeft, float32 *costRight,
                               float32 *disparityMap) {
    _option = option;
    _width = width;
    _height = height;
    _imgLeft = imgLeft;
    _imgRight = imgRight;
    _gradLeft = gradLeft;
    _gradRight = gradRight;
    _planeLeft = planeLeft;
    _planeRight = planeRight;
    _costLeft = costLeft;
    _costRight = costRight;
    _disparityMap = disparityMap;

    _numIter = 0;

    _costCptLeft = new CostComputerPMS(imgLeft, imgRight,
                                       gradLeft, gradRight,
                                       width, height,
                                       option._patchSize,
                                       option._minDisparity, option._maxDisparity,
                                       option._gamma, option._alpha, option._tauCol, option._tauGrad
    );

    _costCptRight = new CostComputerPMS(imgRight, imgLeft,
                                        gradRight, gradLeft,
                                        width, height,
                                        option._patchSize,
                                        -option._maxDisparity, -option._minDisparity,
                                        option._gamma, option._alpha, option._tauCol, option._tauGrad);

    // 计算初始代价数据
    computeCostData();
}

PMSPropagation::~PMSPropagation() {
    SAFE_DELETE(_costCptLeft);
    SAFE_DELETE(_costCptRight);
}

void PMSPropagation::computeCostData() {
    if (_costCptLeft == nullptr || _costCptRight == nullptr ||
        _imgLeft == nullptr || _imgRight == nullptr ||
        _gradLeft == nullptr || _gradRight == nullptr ||
        _planeLeft == nullptr || _planeRight == nullptr ||
        _costLeft == nullptr || _disparityMap == nullptr) {
        return;
    }

    auto *costCpt = dynamic_cast<CostComputerPMS *>(_costCptLeft);

    for (sint32 y = 0; y < _height; y++) {
        for (sint32 x = 0; x < _width; x++) {
            DisparityPlane planeP = _planeLeft[y * _width + x];
            _costLeft[y * _width + x] = costCpt->computeAggregation(x, y, planeP);
        }
    }
}

void PMSPropagation::doPropagation() {
    if (_costCptLeft == nullptr || _costCptRight == nullptr ||
        _imgLeft == nullptr || _imgRight == nullptr ||
        _gradLeft == nullptr || _gradRight == nullptr ||
        _planeLeft == nullptr || _planeRight == nullptr ||
        _costLeft == nullptr || _disparityMap == nullptr) {
        return;
    }

    // 偶数次迭代从左上到右下传播
    // 奇数次迭代从右下到左上传播
    const sint32 dir = (_numIter % 2 == 0) ? 1 : -1;
    sint32 y = (dir == 1) ? 0 : _height - 1;
    for (sint32 i = 0; i < _height; i++) {
        sint32 x = (dir == 1) ? 0 : _width - 1;
        for (sint32 j = 0; j < _width; j++) {

            // 空间传播
            spatialPropagation(x, y, dir);

            x += dir;
        }
        y += dir;
    }
    ++_numIter;
}

void PMSPropagation::spatialPropagation(const sint32 &x, const sint32 &y, const sint32 &direction) {
    // ---
    // 空间传播

    // 偶数次迭代从左上到右下传播
    // 奇数次迭代从右下到左上传播
    const sint32 dir = direction;

    // 获取p当前的视差平面并计算代价
    auto &planeP = _planeLeft[y * _width + x];
    auto &costP = _costLeft[y * _width + x];
    auto *costCpt = dynamic_cast<CostComputerPMS *>(_costCptLeft);

    // 获取p左(右)侧像素的视差平面，计算将平面分配给p时的代价，取较小值
    const sint32 xd = x - dir;
    if (xd >= 0 && xd < _width) {
        auto &plane = _planeLeft[y * _width + xd];
        if (plane != planeP) {
            const auto cost = costCpt->computeAggregation(x, y, plane);
            if (cost < costP) {
                planeP = plane;
                costP = cost;
            }
        }
    }

    // 获取p上(下)侧像素的视差平面，计算将平面分配给p时的代价，取较小值
    const sint32 yd = y - dir;
    if (yd >= 0 && yd < _height) {
        auto &plane = _planeLeft[yd * _width + x];
        if (plane != planeP) {
            const auto cost = costCpt->computeAggregation(x, y, plane);
            if (cost < costP) {
                planeP = plane;
                costP = cost;
            }
        }
    }
}
