//
// Created by tianhe on 2022/8/10.
//

#include "PMSPropagation.h"

#define SAFE_DELETE(P) if(P!=nullptr){delete[](P); (P)=nullptr;}

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

}
