//
// Created by tianhe on 2022/8/10.
//

#ifndef COSTCOMPUTER_HPP
#define COSTCOMPUTER_HPP

#include <cmath>

#include "PMSType.h"

#define COST_PUNISH 120.0f  // NOLINT(cppcoreguidelines-macro-usage)

class CostComputer {
public:
    /** \brief 代价计算器默认构造 */
    CostComputer() : _imgLeft(nullptr), _imgRight(nullptr), _width(0), _height(0),
                     _patchSize(0), _minDisparity(0), _maxDisparity(0) {}

    /**
     * \brief 代价计算器初始化
     * \param img_left		左影像数据
     * \param img_right		右影像数据
     * \param width			影像宽
     * \param height		影像高
     * \param patch_size	局部Patch大小
     * \param min_disp		最小视差
     * \param max_disp		最大视差
     */
    CostComputer(const uint8 *imgLeft, const uint8 *imgRight, const sint32 &width, const sint32 &height,
                 const sint32 &patchSize, const sint32 &minDisparity, const sint32 &maxDisparity) {
        _imgLeft = imgLeft;
        _imgRight = imgRight;
        _width = width;
        _height = height;
        _patchSize = patchSize;
        _minDisparity = minDisparity;
        _maxDisparity = maxDisparity;
    }

    /** \brief 代价计算器析构 */
    virtual ~CostComputer() = default;

    /**
     * \brief 计算左影像p点视差为d时的代价值
     * \param x		p点x坐标
     * \param y		p点y坐标
     * \param d		视差值
     * \return 代价值
     */
    virtual float32 compute(const sint32 &x, const sint32 &y, const float32 &d) = 0;

public:
    /** \brief 左影像数据 */
    const uint8 *_imgLeft;
    /** \brief 右影像数据 */
    const uint8 *_imgRight;

    /** \brief 影像宽 */
    sint32 _width;
    /** \brief 影像高 */
    sint32 _height;
    /** \brief 局部窗口Patch大小 */
    sint32 _patchSize;

    /** \brief 最小最大视差 */
    sint32 _minDisparity;
    sint32 _maxDisparity;
};

/**
 * \brief 代价计算器：PatchMatchStereo原文代价计算器
 */
class CostComputerPMS : public CostComputer {
public:
    /** \brief PMS代价计算器默认构造 */
    CostComputerPMS() : _gradLeft(nullptr), _gradRight(nullptr),
                        _gamma(0), _alpha(0), _tauCol(0), _tauGrad(0) {};

    /**
     * \brief PMS代价计算器带参构造
     * \param img_left		左影像数据
     * \param img_right		右影像数据
     * \param grad_left		左梯度数据
     * \param grad_right	右梯度数据
     * \param width			影像宽
     * \param height		影像高
     * \param patch_size	局部Patch大小
     * \param min_disp		最小视差
     * \param max_disp		最大视差
     * \param gamma			参数gamma值
     * \param alpha			参数alpha值
     * \param t_col			参数tau_col值
     * \param t_grad		参数tau_grad值
     */
    CostComputerPMS(const uint8 *imgLeft, const uint8 *imgRight,
                    const PGradient *gradLeft, const PGradient *gradRight,
                    const sint32 &width, const sint32 &height,
                    const sint32 &patchSize,
                    const sint32 &minDisparity, const sint32 &maxDisparity,
                    const float32 &gamma, const float32 &alpha,
                    const float32 &tauCol, const float32 tauGrad) :
            CostComputer(imgLeft, imgRight, width, height, patchSize, minDisparity, maxDisparity) {
        _gradLeft = gradLeft;
        _gradRight = gradRight;
        _gamma = gamma;
        _alpha = alpha;
        _tauCol = tauCol;
        _tauGrad = tauGrad;
    }


    /**
     * \brief 计算左影像p点视差为d时的代价值，未做边界判定
     * \param x		p点x坐标
     * \param y		p点y坐标
     * \param d		视差值
     * \return 代价值
     */
    float32 compute(const sint32 &x, const sint32 &y, const float32 &d) override {
        const float32 xr = float32(x) - d;
        if (xr < 0.0f || xr >= float32(_width)) {
            return (1.0f - _alpha) * _tauCol + _alpha * _tauGrad;
        }

        // 颜色空间距离
        const auto colQL = getColor(_imgLeft, x, y);
        const auto colQR = getColor(_imgRight, xr, y);
        const auto dc = std::min(
                std::abs(float32(colQL._b) - colQR._x) +
                std::abs(float32(colQL._g) - colQR._y) +
                std::abs(float32(colQL._r) - colQR._z),
                _tauCol
        );

        // 梯度空间距离
        const auto gradQL = getGradient(_gradLeft, x, y);
        const auto gradQR = getGradient(_gradRight, xr, y);
        const auto dg = std::min(
                std::abs(float32(gradQL._x) - gradQR._x) +
                std::abs(float32(gradQL._y) - gradQR._y),
                _tauGrad
        );

        // 代价值
        return (1 - _alpha) * dc + _alpha * dg;
    }


    /**
     * \brief 计算左影像p点视差平面为p时的聚合代价值
     * \param x		p点x坐标
     * \param y 	p点y坐标
     * \param p		平面参数
     * \return 聚合代价值
     */
    float32 computeAggregation(const sint32 &x, const sint32 &y, const DisparityPlane &p) {
        const sint32 patHalf = _patchSize / 2;
        const PColor &colP = getColor(_imgLeft, x, y);

        float32 cost = 0.0f;
        for (sint32 r = -patHalf; r <= patHalf; r++) {
            sint32 yL = y + r;
            for (sint32 c = -patHalf; c <= patHalf; c++) {
                sint32 xL = x + c;

                if (yL < 0 || yL >= _height || xL < 0 || xL >= _width) {
                    continue;
                }
                // 计算视差值
                const float32 d = p.getDisparity(xL, yL);

                if (d < float32(_minDisparity) || d > float32(_maxDisparity)) {
                    cost += COST_PUNISH;
                    continue;
                }

                const PColor &colQ = getColor(_imgLeft, xL, yL);
                const auto dc = std::abs(colP._r - colQ._r) +
                                std::abs(colP._g - colQ._g) +
                                std::abs(colP._b - colQ._b);

                const auto w = std::exp(float32(-dc) / _gamma);

                cost += w * compute(xL, yL, d);
            }
        }

        return cost;
    }

    /**
    * \brief 获取像素点的颜色值
    * \param img_data	颜色数组,3通道
    * \param x			像素x坐标
    * \param y			像素y坐标
    * \return 像素(x,y)的颜色值
    */
    PColor getColor(const uint8 *imgData, const sint32 &x, const sint32 &y) {
        const uint8 *pixel = imgData + y * (_width * 3) + (x * 3);
        return {pixel[0], pixel[1], pixel[2]};
    }

    PVector3f getColor(const uint8 *imgData, const float32 &x, const sint32 &y) {
        float32 col[3] = {0.0f, 0.0f, 0.0f};
        const auto x1 = sint32(x);
        const sint32 x2 = x1 + 1;

        const float32 ofs = x - float32(x1);

        for (sint32 n = 0; n < 3; n++) {
            const auto &col1 = imgData[y * (_width * 3) + (x1 * 3) + n];
            const auto &col2 = (x2 < _width) ? imgData[y * (_width * 3) + (x2 * 3) + n] : col1;
            col[n] = (1.0f - ofs) * float32(col1) + ofs * float32(col2);
        }

        return {col[0], col[1], col[2]};
    }

    PGradient getGradient(const PGradient *gradData, const sint32 &x, const sint32 &y) {
        return gradData[y * _width + x];
    }

    PVector2f getGradient(const PGradient *gradData, const float32 &x, const sint32 &y) {
        const auto x1 = static_cast<sint32>(x);
        const sint32 x2 = x1 + 1;
        const float32 ofs = x - float32(x1);

        const auto &grad1 = gradData[y * _width + x1];
        const auto &grad2 = (x2 < _width) ? gradData[y * _width + x2] : grad1;

        return {
                (1.0f - ofs) * float32(grad1._x) + ofs * float32(grad2._x),
                (1.0f - ofs) * float32(grad1._y) + ofs * float32(grad2._y)
        };
    }

private:
    /** \brief 左影像梯度数据 */
    const PGradient *_gradLeft;
    /** \brief 右影像梯度数据 */
    const PGradient *_gradRight;

    /** \brief 参数gamma */
    float32 _gamma;
    /** \brief 参数alpha */
    float32 _alpha;
    /** \brief 参数tau_col */
    float32 _tauCol;
    /** \brief 参数tau_grad */
    float32 _tauGrad;
};


#endif //COSTCOMPUTER_HPP
