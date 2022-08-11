//
// Created by ZZK on 2022/7/16.
//

#ifndef PATCHMATCHSTEREO_H
#define PATCHMATCHSTEREO_H

#include "PMSPropagation.h"
#include "PMSType.h"
#include <vector>
#include <utility>
#include <ctime>
#include <random>

class PatchMatchStereo {
public:
    PatchMatchStereo();

    ~PatchMatchStereo();

    /**
    * \brief 类的初始化，完成一些内存的预分配、参数的预设置等
    * \param width		输入，核线像对影像宽
    * \param height		输入，核线像对影像高
    * \param option		输入，PatchMatchStereo参数
    */
    bool initialize(const sint32 &width, const sint32 &height, const PMSOption &option);

    /**
    * \brief 执行匹配
    * \param img_left	输入，左影像数据指针，3通道
    * \param img_right	输入，右影像数据指针，3通道
    * \param disp_left	输出，左影像视差图指针，预先分配和影像等尺寸的内存空间
    */
    bool match(const uint8 *imgLeft, const uint8 *imgRight, float32 *dispLeft, float32 *dispRight);

    /**
    * \brief 重设
    * \param width		输入，核线像对影像宽
    * \param height		输入，核线像对影像高
    * \param option		输入，SemiGlobalMatching参数
    */
    bool reset(const uint32 &width, const uint32 &height, const PMSOption &option);

private:
    /** \brief 随机初始化 */
    void randomInitialization();

    /** \brief 计算灰度数据 */
    void computeGray();

    /** \brief 计算梯度数据 */
    void computeGradient();

    /** \brief 迭代传播 */
    void propagation();

    /** \brief 一致性检查	 */
    void lrCheck();

    /** \brief 视差图填充 */
    void fillHolesInDispMap();

    /** \brief 平面转换成视差 */
    void planeToDisparity();

    /** \brief 内存释放	 */
    void release();

    /** \brief PMS参数	 */
    PMSOption _option;

    /** \brief 影像宽	 */
    sint32 _width;

    /** \brief 影像高	 */
    sint32 _height;

    /** \brief 左影像数据	 */
    const uint8 *_imgLeft;
    /** \brief 右影像数据	 */
    const uint8 *_imgRight;

    /** \brief 左影像灰度数据	 */
    uint8 *_grayLeft;
    /** \brief 右影像灰度数据	 */
    uint8 *_grayRight;

    /** \brief 左影像梯度数据	 */
    PGradient *_gradLeft;
    /** \brief 右影像梯度数据	 */
    PGradient *_gradRight;

    /** \brief 左影像聚合代价数据	 */
    float32 *_costLeft;
    /** \brief 右影像聚合代价数据	 */
    float32 *_costRight;

    /** \brief 左影像视差图	*/
    float32 *_dispLeft;
    /** \brief 右影像视差图	*/
    float32 *_dispRight;

    /** \brief 左影像平面集	*/
    DisparityPlane *_planeLeft;
    /** \brief 右影像平面集	*/
    DisparityPlane *_planeRight;

    /** \brief 是否初始化标志	*/
    bool _isInitialized;

    /** \brief 误匹配区像素集	*/
    std::vector<std::pair<sint32, sint32>> _mismatchesLeft;
    std::vector<std::pair<sint32, sint32>> _mismatchesRight;

};


#endif //PATCHMATCHSTEREO_H
