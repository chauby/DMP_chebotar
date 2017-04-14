#ifndef _TRAJECTORY_
#define _TRAJECTORY_

#include <vector>
#include "DMPState.h"
using namespace dmp;
using namespace std;
/**
 * @brief 通过一个二维数组生成DMP的trajectory
 * @param[in] tr[][2] 输入的数据
 * @param[in] length  输入的数据的行数量
 * @param[out] trajectory 
 */
void convert_trajectory_pt(const double tr[][2],int length,vector<DMPState>& trajectory);
/**
 * @brief 载入轨迹,格式:TPVA 
 * @param file_name 文件名
 * @param trajectory 轨迹
 */
void load_trajectory_pvat(const char* file_name,vector<DMPState>& trajectory);

/**
 * @brief 载入轨迹,格式:TP
 * @param file_name 轨迹文件
 * @param trajectory 轨迹文件中只记录了位置
 * @param dt 采样周期
 */
int load_trajectory_pt(const char* file_name,vector<DMPState>& trajectory);

/**
 * @brief 载入轨迹,格式:P
 * @param file_name 轨迹文件
 * @param trajectory 轨迹文件中只记录了位置
 * @param dt 采样周期
 */
void load_trajectory_p(const char* file_name,vector<DMPState>& trajectory,double dt);

/**
 * @brief 保存轨迹
 * @param file_name
 * @param trajectory
 */
void save_trajectory(const char* file_name,vector<DMPState>& trajectory);

#endif /* ifndef _TRAJECTORY_ */
