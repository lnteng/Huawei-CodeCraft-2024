#include "../const.h"
using namespace std;

// 定义地图大小
// const int N = 200;

// 定义地图
// char ch[N][N]; //注意这里和原来的地图大小不一样

// 定义方向
const int dx[4] = {0, 0, -1, 1};
const int dy[4] = {1, -1, 0, 0};


// 定义节点
struct Node {
    int x, y; // 坐标
    double g, h; // 实际代价和启发式代价
    Node(int _x, int _y, double _g, double _h) : x(_x), y(_y), g(_g), h(_h) {}
};

// 计算两点之间的欧式距离
double cost(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// A*算法
vector<int> AStar(int sx, int sy, int ex, int ey) {
    // 定义优先队列，按照f值（g+h）从小到大排序
    auto cmp = [](Node& a, Node& b) {
        return a.g + a.h > b.g + b.h;
    };
    priority_queue<Node, vector<Node>, decltype(cmp)> q(cmp);

    // 初始化起点
    Node start(sx, sy, 0, cost(sx, sy, ex, ey));
    q.push(start);

    // 定义访问数组，记录每个位置是否已经访问过
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    visited[sx][sy] = true;

    // 定义父节点数组，记录每个节点的父节点
    vector<vector<pair<int, int>>> parent(N, vector<pair<int, int>>(N, {-1, -1}));

    while (!q.empty()) {
        Node cur = q.top();
        q.pop();

        // 到达终点
        if (cur.x == ex && cur.y == ey) {
            vector<int> path;
            while (cur.x != sx || cur.y != sy) {
                pair<int, int> p = parent[cur.x][cur.y]; //这里是找父节点，所以是终点到起点，所以是反向的
                if (cur.x == p.first) {
                    if (cur.y > p.second) {
                        path.push_back(0); // 左
                    } else {
                        path.push_back(1); // 右
                    }
                } else {
                    if (cur.x > p.first) {
                        path.push_back(3); // 上
                    } else {
                        path.push_back(2); // 下
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
            if (nx >= 0 && nx < N && ny >= 0 && ny < N && !visited[nx][ny] && ch[nx][ny] == '.') {
                visited[nx][ny] = true;
                parent[nx][ny] = {cur.x, cur.y};
                Node next(nx, ny, cur.g + 1, cost(nx, ny, ex, ey));
                q.push(next);
            }
        }
    }

    // 无法到达终点
    return {};
}

// int main() {
//     // 读取地图和起点、终点坐标
//     ifstream fin("/home/ys/project/huaweiruantiao/Huawei-CodeCraft-2024/maps/map-3.8.txt");
//     for (int i = 0; i < N; i++) {
//         for (int j = 0; j < N; j++) {
//             fin >> ch[i][j];
//         }
//     }
//     int sx, sy, ex, ey;
//     cin >> sx >> sy >> ex >> ey;

//     vector<int> path = AStar(sx, sy, ex, ey);
//     // // 计时
//     // clock_t start, end;
//     // start = clock();

//     // // 打印100次
//     // for (int i = 0; i < 100; i++) {
//     //     vector<int> path = AStar(sx, sy, ex, ey);
//     // }

//     // end = clock();
//     // cout << "time: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

//     // 输出路径
//     for (const auto& p : path) {
//         cout << p << " ";
//     }
//     cout << endl;

//     return 0;
// }
