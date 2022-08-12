//
// Created by ZZK on 2022/7/10.
//

#ifndef PMSTYPE_H
#define PMSTYPE_H

#include <cstdint>
#include <limits>
#include <vector>
#include <cmath>

/** \brief float无效值 */
constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

/**
 * \brief 基础类型别名
*/
typedef int8_t sint8;       // 有符号8位整数
typedef uint8_t uint8;      // 无符号8位整数
typedef int16_t sint16;     // 有符号16位整数
typedef uint16_t uint16;    // 无符号16位整数
typedef int32_t sint32;     // 有符号32位整数
typedef uint32_t uint32;    // 无符号32位整数
typedef int64_t sint64;     // 有符号64位整数
typedef uint64_t uint64;    // 无符号64位整数
typedef float float32;      // 单精度浮点
typedef double float64;     // 双精度浮点

/** \brief PMS参数结构体 */
struct PMSOption {
    sint32 _patchSize;              // patch尺寸，局部窗口为 patch_size*patch_size
    sint32 _minDisparity;           // 最小视差
    sint32 _maxDisparity;           // 最大视差

    float32 _gamma;                 // gamma 权值因子
    float32 _alpha;                 // alpha 相似度平衡因子
    float32 _tauCol;                // tau for color	相似度计算颜色空间的绝对差的下截断阈值
    float32 _tauGrad;               // tau for gradient 相似度计算梯度空间的绝对差下截断阈值

    sint32 _numIters;               // 传播迭代次数

    bool _isCheckLR;                // 是否检查左右一致性
    float32 _lrCheckThres;          // 左右一致性约束阈值

    bool _isFillHoles;              // 是否填充视差空洞

    bool _isForceFpw;               // 是否强制为Frontal-Parallel Window
    bool _isIntegerDisp;            // 是否为整像素视差

    PMSOption() : _patchSize(35), _minDisparity(0), _maxDisparity(64), _gamma(10.0f), _alpha(0.9f),
                  _tauCol(10.0f), _tauGrad(2.0f), _numIters(3), _isCheckLR(false), _lrCheckThres(0),
                  _isFillHoles(false), _isForceFpw(false), _isIntegerDisp(false) {}
};

/**
 * \brief 颜色结构体
 */
struct PColor {
    uint8 _r, _g, _b;

    PColor() : _r(0), _g(0), _b(0) {}

    PColor(uint8 b, uint8 g, uint8 r) : _r(r), _g(g), _b(b) {}
};

/**
 * \brief 梯度结构体
 */
struct PGradient {
    sint16 _x, _y;

    PGradient() : _x(0), _y(0) {}

    PGradient(sint16 x, sint16 y) : _x(x), _y(y) {}
};

/**
* \brief 二维矢量结构体
*/
struct PVector2f {

    float32 _x, _y;

    PVector2f() : _x(0.0f), _y(0.0f) {};

    PVector2f(const float32 &x, const float32 &y) : _x(x), _y(y) {}

    PVector2f(const sint16 &x, const sint16 &y) {
        _x = float32(x);
        _y = float32(y);
    }

    PVector2f(const PVector2f &v) {
        this->_x = v._x;
        this->_y = v._y;
    }

    // ···operators
    // operator +
    PVector2f operator+(const PVector2f &v) const {
        return {this->_x + v._x, this->_y + v._y};
    }

    // operator -
    PVector2f operator-(const PVector2f &v) const {
        return {this->_x - v._x, this->_y - v._y};
    }

    // operator -t
    PVector2f operator-() const {
        return {-this->_x, -this->_y};
    }

    // operator =
    PVector2f &operator=(const PVector2f &v) {
        if (this == &v) {
            return *this;
        } else {
            this->_x = v._x;
            this->_y = v._y;
            return *this;
        }
    }
};

/**
* \brief 三维矢量结构体
*/
struct PVector3f {

    float32 _x, _y, _z;

    PVector3f() : _x(0.0f), _y(0.0f), _z(0.0f) {};

    PVector3f(const float32 &x, const float32 &y, const float32 &z) {
        _x = x;
        _y = y;
        _z = z;
    }

    PVector3f(const uint8 &x, const uint8 &y, const uint8 &z) {
        _x = float32(x);
        _y = float32(y);
        _z = float32(z);
    }

    PVector3f(const PVector3f &v) {
        this->_x = v._x;
        this->_y = v._y;
        this->_z = v._z;
    }

    // normalize
    void normalize() {
        if (_x == 0.0f && _y == 0.0f && _z == 0.0f) {
            return;
        } else {
            const float32 sq = _x * _x + _y * _y + _z * _z;
            const auto sqf = (float32) sqrt(sq);
            _x /= sqf;
            _y /= sqf;
            _z /= sqf;
        }
    }

    // ···operators
    // operator +
    PVector3f operator+(const PVector3f &v) const {
        return {this->_x + v._x, this->_y + v._y, this->_z + v._z};
    }

    // operator -
    PVector3f operator-(const PVector3f &v) const {
        return {this->_x - v._x, this->_y - v._y, this->_z - v._z};
    }

    // operator -t
    PVector3f operator-() const {
        return {-this->_x, -this->_y, -this->_z};
    }

    // operator =
    PVector3f &operator=(const PVector3f &v) {
        if (this == &v) {
            return *this;
        } else {
            this->_x = v._x;
            this->_y = v._y;
            this->_z = v._z;
            return *this;
        }
    }

    // operator ==
    bool operator==(const PVector3f &v) const {
        return (this->_x == v._x) && (this->_y == v._y) && (this->_z == v._z);
    }

    // operator !=
    bool operator!=(const PVector3f &v) const {
        return (this->_x != v._x) || (this->_y != v._y) || (this->_z != v._z);
    }

    // dot
    float32 dot(const PVector3f &v) const {
        return this->_x * v._x + this->_y * v._y + this->_z * v._z;
    }
};

/**
 * \brief 视差平面
 */
struct DisparityPlane {
    PVector3f _p;

    DisparityPlane() = default;

    DisparityPlane(const float32 &x, const float32 &y, const float32 &z) {
        _p._x = x;
        _p._y = y;
        _p._z = z;
    }

    DisparityPlane(const sint32 &x, const sint32 &y, const PVector3f &n, const float32 &d) {
        _p._x = -n._x / n._z;
        _p._y = -n._y / n._z;
        _p._z = (n._x * float32(x) + n._y * float32(y) + n._z * d) / n._z;
    }

    /**
     * \brief 获取该平面下像素(x,y)的视差
     * \param x		像素x坐标
     * \param y		像素y坐标
     * \return 像素(x,y)的视差
     */
    float32 getDisparity(const sint32 &x, const sint32 &y) const {
        return _p.dot(PVector3f(float32(x), float32(y), 1.0f));
    }

    /** \brief 获取平面的法线 */
    PVector3f getNormal() const {
        PVector3f n(_p._x, _p._y, -1.0f);
        n.normalize();
        return n;
    }

    /**
     * \brief 将视差平面转换到另一视图
     * 假设左视图平面方程为 d = a_p*xl + b_p*yl + c_p
     * 左右视图满足：(1) xr = xl - d_p; (2) yr = yl; (3) 视差符号相反(本代码左视差为正值，右视差为负值)
     * 代入左视图视差平面方程就可得到右视图坐标系下的平面方程: d = a_p/(a_p-1)*xr + b_p/(a_p-1)*yr + c_p/(a_p-1)
     * 右至左同理
     * \param x		像素x坐标
     * \param y 	像素y坐标
     * \return 转换后的平面
     */
    DisparityPlane toAnotherView(const sint32 &x, const sint32 &y) const {
        const float32 d = getDisparity(x, y);
        return {-_p._x, -_p._y, -_p._z - _p._x * d};
    }

    // operator ==
    bool operator==(const DisparityPlane &v) const {
        return _p == v._p;
    }

    // operator !=
    bool operator!=(const DisparityPlane &v) const {
        return _p != v._p;
    }
};

#endif //PMSTYPE_H
