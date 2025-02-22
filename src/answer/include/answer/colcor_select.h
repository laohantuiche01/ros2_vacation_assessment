#ifndef COLCOR_SELECT_H
#define COLCOR_SELECT_H

#include<opencv2/opencv.hpp>

namespace colcor_select {
    std::vector<std::vector<int> > myColcor={
        {29,198,113},//绿色传送门 0
        {193,97,212},//紫色传送门 1
        {60,85,107},//补给 2
        {170,120,151},//基地 3
        {94,101,255},//密码 4
        {89,170,240},//自己 5
        {255,104,104},//敌方机器人 6
        {175,175,175},//背景 7
        {58,58,58}//墙 8
    };
}

#endif //COLCOR_SELECT_H
