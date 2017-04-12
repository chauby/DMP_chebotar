/**
 * @file load_trajectory.cpp
 * @brief 以文件的形式载入dmp的trajectory
 * @author kipochen
 * @version v1.0
 * @date 2016-12-08
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include "DMPState.h"

using namespace std;
using namespace dmp;


/**
 * @brief 通过一个二维数组生成DMP的trajectory
 * @param[in] tr[][2] 输入的数据
 * @param[in] length  输入的数据的行数量
 * @param[out] trajectory 
 */
void convert_trajectory_pt(const double tr[][2],int length,vector<DMPState>& trajectory)
{

	DMPState state;
	double x,xd,xdd,time;

	//先输入所有的x
	for (int i = 0; i < length; ++i) 
	{
		time = tr[i][0];
		x = tr[i][1];
		state.setX(x);
		state.setTime(time);
		trajectory.push_back(state);
	}

	//只针对中间点进行操作,计算速度与加速度
	double v_pre;
	double v_after;
	double dt_pre ;
	double dt_after ;

	std::vector<DMPState>::iterator iter = trajectory.begin();
	for(;iter!=trajectory.end();++iter)
	{
		if(iter!=trajectory.begin() && iter!=(trajectory.end()-1))
		{
			dt_pre = iter->getTime() - (iter-1)->getTime(); //当前时刻减去前一个时刻
			dt_after = (iter+1)->getTime() - iter->getTime(); //下一个时刻减去当前时刻
			xd = ((iter+1)->getX() - (iter-1)->getX())/(dt_pre+dt_after);
			v_pre = (iter->getX() - (iter-1)->getX())/dt_pre;
			v_after = ((iter+1)->getX() - iter->getX())/dt_after;
			xdd = 2*(v_after - v_pre)/(dt_pre+dt_after);
			iter->setXd(xd);
			iter->setXdd(xdd);
		}
	}

	//只针对开始点与结束点进行操作,计算速度与加速度
	iter =  trajectory.begin();
	iter->setXd((iter+1)->getXd());
	iter->setXdd((iter+1)->getXdd());

	iter =  trajectory.end()-1;
	iter->setXd((iter-1)->getXd());
	iter->setXdd((iter-1)->getXdd());
}
/**
 * @brief 载入轨迹,格式TPVA:time position velocity acc 
 * @param[in] file_name 文件名
 * @param[out] trajectory 轨迹
 */
void load_trajectory_pvat(const char* file_name,vector<DMPState>& trajectory)
{
	FILE* f = fopen(file_name, "rt");
	if(f==NULL)
	{
		printf("文件不存在\n");
	}
	char line[1000];
	int i = 0;
	DMPState state;
	double x,xd,xdd,time;
	while(fgets(line, 1000, f) != NULL)
	{
		sscanf(line, "%lf %lf %lf %lf", &time,&x,&xd,&xdd);
		state.setX(x);
		state.setXd(xd);
		state.setXdd(xdd);
		state.setTime(time);
		trajectory.push_back(state);
		i++;
	}
}

/**
 * @brief 载入轨迹,格式TP:Time Position
 * @param[in] file_name 轨迹文件
 * @param[out] trajectory 轨迹文件中只记录了位置
 * @param[in] dt 采样周期
 */
void load_trajectory_pt(const char* file_name,vector<DMPState>& trajectory)
{
	FILE* f = fopen(file_name, "rt");

	if(f==NULL)
	{
		printf("文件不存在\n");
	}

	DMPState state;
	double x,xd,xdd,time;
	char line[1000];
	int i = 0;

	//先输入所有的x
	while(fgets(line, 1000, f) != NULL)
	{
		sscanf(line, "%lf %lf", &time,&x);
		state.setX(x);
		state.setTime(time);
		trajectory.push_back(state);
		i++;
	}

	//只针对中间点进行操作,计算速度与加速度
	double v_pre;
	double v_after;
	double dt_pre ;
	double dt_after ;

	std::vector<DMPState>::iterator iter = trajectory.begin();
	for(;iter!=trajectory.end();++iter)
	{
		if(iter!=trajectory.begin() && iter!=(trajectory.end()-1))
		{
			dt_pre = iter->getTime() - (iter-1)->getTime(); //当前时刻减去前一个时刻
			dt_after = (iter+1)->getTime() - iter->getTime(); //下一个时刻减去当前时刻
			xd = ((iter+1)->getX() - (iter-1)->getX())/(dt_pre+dt_after);
			v_pre = (iter->getX() - (iter-1)->getX())/dt_pre;
			v_after = ((iter+1)->getX() - iter->getX())/dt_after;
			xdd = 2*(v_after - v_pre)/(dt_pre+dt_after);
			iter->setXd(xd);
			iter->setXdd(xdd);
		}
	}

	//只针对开始点与结束点进行操作,计算速度与加速度
	iter =  trajectory.begin();
	iter->setXd((iter+1)->getXd());
	iter->setXdd((iter+1)->getXdd());

	iter =  trajectory.end()-1;
	iter->setXd((iter-1)->getXd());
	iter->setXdd((iter-1)->getXdd());
}

/**
 * @brief 载入轨迹,格式P:position
 * @param file_name 轨迹文件
 * @param trajectory 轨迹文件中只记录了位置
 * @param dt 采样周期
 */
void load_trajectory_p(const char* file_name,vector<DMPState>& trajectory,double dt)
{
	FILE* f = fopen(file_name, "rt");
	if(f==NULL)
	{
		printf("文件不存在\n");
	}

	DMPState state;
	double x,xd,xdd,time;
	char line[1000];
	int i = 0;
	//先输入所有的x
	while(fgets(line, 1000, f) != NULL)
	{
		time = dt*i;
		sscanf(line, "%lf", &x);
		state.setX(x);
		state.setTime(time);
		trajectory.push_back(state);
		i++;
	}

	//只针对中间点进行操作,计算速度与加速度
	double v_pre;
	double v_after;
	std::vector<DMPState>::iterator iter = trajectory.begin();
	for(;iter!=trajectory.end();++iter)
	{
		if(iter!=trajectory.begin() && iter!=(trajectory.end()-1))
		{
			xd = ((iter+1)->getX() - (iter-1)->getX())/(2*dt);
			v_pre = (iter->getX() - (iter-1)->getX())/dt;
			v_after = ((iter+1)->getX() - iter->getX())/dt;
			xdd = (v_after - v_pre)/dt;
			iter->setXd(xd);
			iter->setXdd(xdd);
		}
	}

	//只针对开始点与结束点进行操作,计算速度与加速度
	iter =  trajectory.begin();
	iter->setXd((iter+1)->getXd());
	iter->setXdd((iter+1)->getXdd());

	iter =  trajectory.end()-1;
	iter->setXd((iter-1)->getXd());
	iter->setXdd((iter-1)->getXdd());
}

/**
 * @brief 保存轨迹
 * @param file_name
 * @param trajectory
 */
void save_trajectory(const char* file_name,vector<DMPState>& trajectory)
{
	FILE* f = fopen(file_name, "w+");
	if(f==NULL)
	{
		printf("无法创建文件");
	}
	std::vector<DMPState>::iterator iter = trajectory.begin();
	for(;iter!=trajectory.end();++iter)
	{
            fprintf(f, "%.7f %.7f %.7f %.7f\n", iter->getTime(),
                    iter->getX(), iter->getXd(), iter->getXdd());
	}
}
