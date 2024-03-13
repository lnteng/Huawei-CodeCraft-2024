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

int calcEulerDist2(int x1, int y1, int x2, int y2) { //欧式距离
    return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

int calcManhattanDist(int x1, int y1, int x2, int y2) { //曼哈顿距离
    return abs(x2 - x1) + abs(y2 - y1);
}

void printMoreDebugINfo() {
    for (int i = 0; i < berth_num; i++) { // debug
        for (int x = 0; x < N; x++) {
            std::ostringstream oss;
            oss << "[";
            for (int y = 0; y < N; y++) {
                oss << dists[i][x][y] << "\t";
            }
            oss << "]";
            logger.log(INFO, oss.str());
        }
    }
}


// 定位点属于港口区域
int locateBelongBerth(Point point) { 
    // 返回值：返回地图点位所属的固定港口区域的id
    int best_berth_id = 0;
    for (int i = 1; i < select_berth_num; i++) {
        best_berth_id = dists[best_berth_id][point.first][point.second] < dists[i][point.first][point.second] ? best_berth_id : i;
    }
    return best_berth_id;
}

// 初始化地图点位所属泊位区域
void initBelongBerth() {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            berth_field[i][j] = locateBelongBerth(make_pair(i, j));
        }
    }
}

// 定义节点
struct Node {
    int x, y; // 坐标
    int g, h; // 实际代价和启发式代价
    Node(int _x, int _y, int _g, int _h) : x(_x), y(_y), g(_g), h(_h) {}
};

// A* 算法
vector<Direct> AStar(Point p1, Point p2) {
    // 定义优先队列，按照f值（g+h）从小到大排序
    auto cmp = [](Node& a, Node& b) {
        return a.g + a.h > b.g + b.h;
    };
    priority_queue<Node, vector<Node>, decltype(cmp)> heap(cmp);

    // 初始化起点
    Node start(p1.first, p1.second, 0, calcManhattanDist(p1.first, p1.second, p2.first, p2.second));
    heap.push(start);

    // 定义访问数组，记录每个位置是否已经访问过
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    visited[p1.first][p1.second] = true; //visited[p1.first][p2.second] = true;

    // 定义父节点数组，记录每个节点的父节点
    vector<vector<Point>> parent(N, vector<Point>(N, {-1, -1}));

    while (!heap.empty()) {
        Node cur = heap.top();
        heap.pop();

        // 到达终点
        if (cur.x == p2.first && cur.y == p2.second) {
            vector<Direct> path;
            while (cur.x != p1.first || cur.y != p1.second) {
                Point p = parent[cur.x][cur.y]; //这里是找父节点，所以是终点到起点，所以是反向的
                if (cur.x == p.first) {
                    if (cur.y > p.second) {
                        path.push_back(Direct::right); // 左
                    } else {
                        path.push_back(Direct::left); // 右
                    }
                } else {
                    if (cur.x > p.first) {
                        path.push_back(Direct::down); // 上
                    } else {
                        path.push_back(Direct::upper); // 下
                    }
                }
                cur.x = p.first;
                cur.y = p.second;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // 遍历四个方向
        for (int i = 0; i < 4; i++) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            // 判断新位置是否有效
            if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited[nx][ny] && (ch[nx][ny] == '.' || ch[nx][ny] == 'B')) {
                visited[nx][ny] = true;
                parent[nx][ny] = {cur.x, cur.y};
                Node next(nx, ny, cur.g + 1, calcManhattanDist(nx, ny, p2.first, p2.second));
                heap.push(next);
            }
        }
    }

    // 无法到达终点
    return {};
}
