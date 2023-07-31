#pragma once
#include "adrc.h"

class Smc {
public:
	float m_fpDes;	// 控制变量目标值
	float m_fpFB;	// 控制变量反馈值
	float m_fpE;	// 本次偏差
	float m_fpU;	// 本次运算结果
	float m_fpUMax;	// 输出上限
	
	// SMC参数
	float m_b;		// 常量，系统决定，定下来后不需要修改
	float m_eps;	// 减小ε可以减缓抖振，但是会导致跟踪速度变慢
	float m_gain;	// 增大m_gain可以降低响应时间和减小跟踪误差，但是太大会导致计算错误
	float m_dead;	// 常量，系统决定，定下来后不需要修改
	Adrc_TD* m_TD; 

    Smc(float b, float eps, float gain, float dead, float output_max);

    void CalSMC(void);
};