#pragma once
#include "const.h"
/*
    实现图相关的算法
*/

// 基于stack实现的广度优先搜索
void getDistByBfs() {
    for (int i = 0; i < berth_num; i++) {
        int step = 0; // 从 berth 出发到达当前 point 的帧数
        queue<Point> que;
        Berth berth = berths[i];

        for (int x = 0; x < N; x++) { // 初始化未访问过
            for (int y = 0; y < N; y++) {
                dists[i][x][y] = INT16_MAX;
            }
        }

        for (int x = 0; x < 4; x++) { // 初始化 berth 内部距离为 0
            for (int y = 0; y < 4; y++) {
                dists[i][berth.x + x][berth.y + y] = step;
                que.push(make_pair(berth.x + x, berth.y + y));
            }
        }

        int counter = que.size(); // 临时变量记录 que 中 point 的数量
        while (!que.empty()) {
            ++step;
            for (int j = 0; j < counter; j++) {
                Point cur_point = que.front();
                que.pop();                
                if (cur_point.second + 1 < n) { // right
                    // 没有访问过的坐标，并且该坐标为空地('.'表示)
                    if (dists[i][cur_point.first][cur_point.second + 1] > step && ch[cur_point.first][cur_point.second + 1] == '.') {
                        dists[i][cur_point.first][cur_point.second + 1] = step;
                        que.push(Point(cur_point.first, cur_point.second + 1));
                    }
                }
                if (cur_point.second > 0) { // left
                    if (dists[i][cur_point.first][cur_point.second - 1] > step && ch[cur_point.first][cur_point.second - 1] == '.') {
                        dists[i][cur_point.first][cur_point.second - 1] = step;
                        que.push(Point(cur_point.first, cur_point.second - 1));
                    }
                }
                if (cur_point.first > 0) { // upper
                    if (dists[i][cur_point.first - 1][cur_point.second] > step && ch[cur_point.first - 1][cur_point.second] == '.') {
                        dists[i][cur_point.first - 1][cur_point.second] = step;
                        que.push(Point(cur_point.first - 1, cur_point.second));
                    }
                }
                if (cur_point.first + 1 < n) { // down
                    if (dists[i][cur_point.first + 1][cur_point.second] > step && ch[cur_point.first + 1][cur_point.second] == '.') {
                        dists[i][cur_point.first + 1][cur_point.second] = step;
                        que.push(Point(cur_point.first + 1, cur_point.second));
                    }
                }
            }
            counter = que.size();
        }
    }
}

int calcEulerDist2(int x1, int y1, int x2, int y2) {
    return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

int calcManhattanDist(int x1, int y1, int x2, int y2) {
    return abs(x2 - x1) + abs(y2 - y1);
}