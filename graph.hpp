#include "const.h"
/*
    实现图相关的算法
*/

extern const int berth_num;
extern const int N;
extern const int n;

// 基于stack实现的广度优先搜索
void get_dist_by_bfs() {
    for (int i = 0; i < berth_num; i++) {
        int step = 0; // 从 berth 出发到达当前 point 的帧数
        stack<Point> stk;
        Berth berth = berths[i];
        int counter = 1; // 临时变量记录 stk 中 point 的数量
        Point berth_start_point{berth.x, berth.y};
        stk.push(berth_start_point);

        for (int x = 0; x < N; x++) { // 初始化未访问过
            for (int y = 0; y < N; y++) {
                dists[i][x][y] = -1;
            }
        }

        dists[i][berth_start_point.x][berth_start_point.y] = step;
        while (!stk.empty()) {
            ++step;
            for (int j = 0; j < counter; j++) {
                Point cur_point = stk.top();
                stk.pop();
                if (cur_point.y + 1 < n) { // right
                    // 没有访问过的坐标，并且该坐标为空地('.'表示)
                    if (dists[i][cur_point.x][cur_point.y + 1] < 0 && ch[cur_point.x][cur_point.y + 1] == '.') {
                        dists[i][cur_point.x][cur_point.y + 1] = step;
                        stk.push(Point{cur_point.x, cur_point.y + 1});
                    }
                }
                if (cur_point.y > 0) { // left
                    if (dists[i][cur_point.x][cur_point.y - 1] < 0 && ch[cur_point.x][cur_point.y + 1] == '.') {
                        dists[i][cur_point.x][cur_point.y - 1] = step;
                        stk.push(Point{cur_point.x, cur_point.y - 1});
                    }
                }
                if (cur_point.x > 0) { // upper
                    if (dists[i][cur_point.x - 1][cur_point.y] < 0 && ch[cur_point.x][cur_point.y + 1] == '.') {
                        dists[i][cur_point.x - 1][cur_point.y] = step;
                        stk.push(Point{cur_point.x - 1, cur_point.y});
                    }
                }
                if (cur_point.x + 1 < n) { // down
                    if (dists[i][cur_point.x + 1][cur_point.y] < 0 && ch[cur_point.x][cur_point.y + 1] == '.') {
                        dists[i][cur_point.x + 1][cur_point.y] = step;
                        stk.push(Point{cur_point.x + 1, cur_point.y});
                    }
                }
            }
            counter = stk.size();
        }
    }
}