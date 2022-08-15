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

    // 随机数生成器
    _randDisp = new std::uniform_real_distribution<float32>(-1.0f, 1.0f);
    _randNorm = new std::uniform_real_distribution<float32>(-1.0f, 1.0f);

    // 计算初始代价数据
    computeCostData();
}

PMSPropagation::~PMSPropagation() {
    SAFE_DELETE(_costCptLeft);
    SAFE_DELETE(_costCptRight);
    SAFE_DELETE(_randDisp);
    SAFE_DELETE(_randNorm);
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

            // 平面优化
            if (!_option._isForceFpw) {
                planeRefine(x, y);
            }

            viewPropagation(x, y);

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

void PMSPropagation::planeRefine(const sint32 &x, const sint32 &y) {
    auto minDisparity = _option._minDisparity;
    auto maxDisparity = _option._maxDisparity;

    // 像素p的平面、代价、视差、法线
    auto &planeP = _planeLeft[y * _width + x];
    auto &costP = _costLeft[y * _width + x];
    float32 dispP = planeP.getDisparity(x, y);
    PVector3f normP = planeP.getNormal();

    // 迭代条件
    float32 dispUpdate = float32(maxDisparity - minDisparity) / 2.0f;
    float32 normUpdate = 1.0f;
    const float32 stopThres = 0.1f;

    // 代价计算器
    auto *costCpt = dynamic_cast<CostComputerPMS *>(_costCptLeft);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    auto &randDisp = *_randDisp;
    auto &randNorm = *_randNorm;

    // 迭代优化
    while (dispUpdate > stopThres) {
        // 在 -disp_update ~ disp_update 范围内随机一个视差增量
        float32 dispRd = randDisp(gen) * dispUpdate;
        if (_option._isIntegerDisp) {
            dispRd = static_cast<float32>(round(dispRd));
        }

        float32 dispPNew = dispP + dispRd;
        if (dispPNew < float32(minDisparity) || dispPNew > float32(maxDisparity)) {
            dispUpdate /= 2.0f;
            normUpdate /= 2.0f;
            continue;
        }

        // 在 -norm_update ~ norm_update 范围内随机三个值作为法线增量的三个分量
        PVector3f normRd;
        if (!_option._isForceFpw) {
            normRd._x = randNorm(gen) * normUpdate;
            normRd._y = randNorm(gen) * normUpdate;
            float32 z = randNorm(gen) * normUpdate;
            while (z == 0.0f) {
                z = randNorm(gen) * normUpdate;
            }
            normRd._z = z;
        } else {
            normRd._x = 0.0f;
            normRd._y = 0.0f;
            normRd._z = 0.0f;
        }
        // 计算像素p新的法线
        auto normPNew = normP + normRd;
        normPNew.normalize();

        // 计算新的视差平面
        auto planeNew = DisparityPlane(x, y, normPNew, dispPNew);

        // 比较Cost
        if (planeNew != planeP) {
            float32 cost = costCpt->computeAggregation(x, y, planeNew);
            if (cost < costP) {
                planeP = planeNew;
                costP = cost;
                dispP = dispPNew;
                normP = normPNew;
            }
        }

        dispUpdate /= 2.0f;
        normUpdate /= 2.0f;
    }
}

void PMSPropagation::viewPropagation(const sint32 &x, const sint32 &y) {
    // --
    // 视图传播
    // 搜索p在右视图的同名点q，更新q的平面

    // 左视图匹配点p的位置及其视差平面
    const sint32 p = y * _width + x;
    const auto &planeP = _planeLeft[p];
    const float32 dispP = planeP.getDisparity(x, y);

    auto *costCpt = dynamic_cast<CostComputerPMS *>(_costCptRight);

    // 计算右视图列号
    const sint32 xr = std::lround(float32(x) - dispP);
    if (xr < 0 || xr >= _width) {
        return;
    }

    const sint32 q = y * _width + xr;
    auto &planeQ = _planeRight[q];
    auto &costQ = _costRight[q];

    // 将左视图的视差平面转换到右视图
    const auto planeP2Q = planeP.toAnotherView(x, y);
    const float32 dispQ = planeP2Q.getDisparity(xr, y);
    const auto cost = costCpt->computeAggregation(xr, y, planeP2Q);
    if (cost < costQ) {
        planeQ = planeP2Q;
        costQ = cost;
    }
}
