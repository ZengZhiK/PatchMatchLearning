//
// Created by tianhe on 2022/8/10.
//

#ifndef PMSPROPAGATION_H
#define PMSPROPAGATION_H


#include "PMSType.h"
#include "CostComputer.hpp"

class PMSPropagation {

public:
    PMSPropagation(const PMSOption &option,
                   const sint32 &width, const sint32 &height,
                   const uint8 *imgLeft, const uint8 *imgRight,
                   const PGradient *gradLeft, const PGradient *gradRight,
                   DisparityPlane *planeLeft, DisparityPlane *planeRight,
                   float32 *costLeft, float32 *costRight,
                   float32 *disparityMap);

    ~PMSPropagation();

    /** \brief 执行传播一次 */
    void doPropagation();

private:
    /** \brief PMS算法参数*/
    PMSOption _option;

    /** \brief 影像宽高 */
    sint32 _width;
    sint32 _height;

    /** \brief 影像数据 */
    const uint8 *_imgLeft;
    const uint8 *_imgRight;

    /** \brief 梯度数据 */
    const PGradient *_gradLeft;
    const PGradient *_gradRight;

    /** \brief 平面数据 */
    DisparityPlane *_planeLeft;
    DisparityPlane *_planeRight;

    /** \brief 代价数据	 */
    float32 *_costLeft;
    float32 *_costRight;

    /** \brief 视差数据 */
    float32 *_disparityMap;

    /** \brief 代价计算器 */
    CostComputer *_costCptLeft;
    CostComputer *_costCptRight;

    /** \brief 传播迭代次数 */
    sint32 _numIter;

    /** \brief 计算代价数据 */
    void computeCostData();

    /**
     * \brief 空间传播
     * \param x 像素x坐标
     * \param y 像素y坐标
     * \param direction 传播方向
     */
    void spatialPropagation(const sint32& x, const sint32& y, const sint32& direction);
};


#endif //PMSPROPAGATION_H
