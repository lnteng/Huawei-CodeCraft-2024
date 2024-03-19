#pragma once
#include "const.h"
/*
    实现图相关的算法
*/

/**
 * @brief 使用基于stack实现的广度优先搜索 (BFS) 算法计算每个泊位到网格上每个点的距离。
 *
 * 该函数将每个泊位与网格上每个点之间的距离初始化为最大限值。
 * 然后它将每个泊位内的点之间的距离设置为 0 并将它们添加到队列中。
 * 然后，该函数在网格上执行 BFS 遍历，在探索每个可到达点时更新距离。
 * 最终距离存储在“dists”数组（已在const.hpp中初始化）中。
 */
void getDistByBfs()
{
    for (int i = 0; i < berth_num; i++)
    {
        int step = 0; // 从 berth 出发到达当前 point 的帧数
        queue<Point> que;
        Berth berth = berths[i];

        for (int x = 0; x < N; x++)
        { // 初始化未访问过
            for (int y = 0; y < N; y++)
            {
                dists[i][x][y] = MAX_LIMIT;
            }
        }

        for (int x = 0; x < 4; x++)
        { // 初始化 berth 内部距离为 0
            for (int y = 0; y < 4; y++)
            {
                dists[i][berth.x + x][berth.y + y] = step;
                que.push(make_pair(berth.x + x, berth.y + y));
            }
        }

        int counter = que.size(); // 临时变量记录 que 中 point 的数量
        while (!que.empty())
        {
            ++step;
            for (int j = 0; j < counter; j++)
            {
                Point cur_point = que.front();
                que.pop();
                if (cur_point.second + 1 < n)
                { // right
                    // 没有访问过的坐标，并且该坐标为机器人可到达的坐标
                    if (dists[i][cur_point.first][cur_point.second + 1] > step && isRobotAccessible(cur_point.first, cur_point.second + 1))
                    {
                        dists[i][cur_point.first][cur_point.second + 1] = step;
                        que.push(Point(cur_point.first, cur_point.second + 1));
                    }
                }
                if (cur_point.second > 0)
                { // left
                    if (dists[i][cur_point.first][cur_point.second - 1] > step && isRobotAccessible(cur_point.first, cur_point.second - 1))
                    {
                        dists[i][cur_point.first][cur_point.second - 1] = step;
                        que.push(Point(cur_point.first, cur_point.second - 1));
                    }
                }
                if (cur_point.first > 0)
                { // upper
                    if (dists[i][cur_point.first - 1][cur_point.second] > step && isRobotAccessible(cur_point.first - 1, cur_point.second))
                    {
                        dists[i][cur_point.first - 1][cur_point.second] = step;
                        que.push(Point(cur_point.first - 1, cur_point.second));
                    }
                }
                if (cur_point.first + 1 < n)
                { // down
                    if (dists[i][cur_point.first + 1][cur_point.second] > step && isRobotAccessible(cur_point.first + 1, cur_point.second))
                    {
                        dists[i][cur_point.first + 1][cur_point.second] = step;
                        que.push(Point(cur_point.first + 1, cur_point.second));
                    }
                }
            }
            counter = que.size();
        }
    }
}

/**
 * @brief 计算栅格地图两点之间的欧式几何距离的平方。
 * @return 两点之间的欧式几何距离平方。
 */
int calcEulerDist2(int x1, int y1, int x2, int y2)
{ // 欧式几何距离
    return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

/**
 * @brief 计算栅格地图两点之间的曼哈顿距离。
 *
 * @return 两点之间的曼哈顿距离。
 */
int calcManhattanDist(int x1, int y1, int x2, int y2)
{ // 曼哈顿距离
    return abs(x2 - x1) + abs(y2 - y1);
}

/**
 * @brief 打印给定泊位索引的距离地图，地图上的值对应该点到泊位的曼哈顿距离。
 *
 * @param berth_index 泊位索引。
 */
void printMoreDebugINfo(int berth_index)
{
    for (int x = 0; x < N; x++)
    {
        std::ostringstream oss;
        oss << "[";
        for (int y = 0; y < N; y++)
        {
            oss << dists[4][x][y] << "\t";
        }
        oss << "]";
        logger.log(INFO, oss.str());
    }
}

/**
 * @brief 返回给定地图点所属的固定泊位（selected_berth）区域的ID。
 *
 * @param point 要定位的地图点。
 * @return 该点所属固定端口区的ID。 如果该点位于不可访问的区域，则返回 -1。
 */
int locateBelongBerth(Point point)
{
    // 返回值：返回地图点位所属的固定泊位区域的id
    int best_selected_berth_id = -1; // 不可达区域
    if (!isRobotAccessible(point.first, point.second))
    {
        return best_selected_berth_id;
    }
    best_selected_berth_id = 0;
    for (int i = 1; i < select_berth_num; i++)
    {
        best_selected_berth_id = getDistByPoint(selected_berth[best_selected_berth_id], point) < getDistByPoint(selected_berth[i], point) ? best_selected_berth_id : i;
    }
    return best_selected_berth_id;
}

// A*定义节点
struct Node
{
    int x, y; // 坐标
    int g, h; // 实际代价和启发式代价
    Node(int _x, int _y, int _g, int _h) : x(_x), y(_y), g(_g), h(_h) {}
};


/**
 * @brief A* 算法:执行 A* 算法来查找网格上两点之间的最短路径。
 *
 * @param p1 起点。
 * @param p2 目标点。
 * @param version 算法版本：版本0为普通A*算法，版本1为A*算法加入了拥堵度的启发式函数。
 * @return 表示存放从 p1 到 p2 最短路径的方向向量的路径。
 * 如果没有路径，则返回空向量。
 */
vector<Direct> AStar(Point p1, Point p2,int version=0)
{
    if (!isRobotAccessible(p1.first, p1.second) || !isRobotAccessible(p2.first, p2.second))
    {
        logger.log(WARNING, formatString("AStar: p1({},{}) or p2({},{}) is not accessible", p1.first, p1.second, p2.first, p2.second)); // 该区域没有可获取货物
        return {};
    }
    if (version != 0 && version != 1)
    {
        logger.log(WARNING, formatString("AStar: version({}) is not valid", version)); // 该区域没有可获取货物
        return AStar(p1, p2, 0); // 默认使用普通A*算法
    }
    // 定义优先队列，按照f值（g+h）从小到大排序
    auto cmp = [](Node &a, Node &b)
    {
        return a.g + a.h > b.g + b.h;
    };
    priority_queue<Node, vector<Node>, decltype(cmp)> heap(cmp);

    // 初始化起点
    Node start(p1.first, p1.second, 0, calcManhattanDist(p1.first, p1.second, p2.first, p2.second));
    heap.push(start);

    // 定义访问数组，记录每个位置是否已经访问过
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    visited[p1.first][p1.second] = true; // visited[p1.first][p2.second] = true;

    // 定义父节点数组，记录每个节点的父节点
    vector<vector<Point>> parent(N, vector<Point>(N, {-1, -1}));

    while (!heap.empty())
    {
        Node cur = heap.top();
        heap.pop();

        // 到达终点
        if (cur.x == p2.first && cur.y == p2.second)
        {
            vector<Direct> path;
            while (cur.x != p1.first || cur.y != p1.second)
            {
                Point p = parent[cur.x][cur.y]; // 这里是找父节点，所以是终点到起点，所以是反向的
                if (cur.x == p.first)
                {
                    if (cur.y > p.second)
                    {
                        path.push_back(Direct::right); // 左
                    }
                    else
                    {
                        path.push_back(Direct::left); // 右
                    }
                }
                else
                {
                    if (cur.x > p.first)
                    {
                        path.push_back(Direct::down); // 上
                    }
                    else
                    {
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
        for (int i = 0; i < 4; i++)
        {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            // 判断新位置是否有效
            if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited[nx][ny] && isRobotAccessible(nx, ny))
            {
                visited[nx][ny] = true;
                parent[nx][ny] = {cur.x, cur.y};
                if (version == 1 && congestion[nx][ny].first >= High_congestion)
                {
                    int extra_cost = high_congestion_cost; // 高拥堵度代价
                    Node next(nx, ny, cur.g + 1 + extra_cost, calcManhattanDist(nx, ny, p2.first, p2.second));
                    heap.push(next);
                }else {
                    Node next(nx, ny, cur.g + 1, calcManhattanDist(nx, ny, p2.first, p2.second));
                    heap.push(next);
                }                
            }
        }
    }

    // 无法到达终点
    return {};
}

/**
 * @brief * 算法: 基于 bfs 的结果计算返回指定港口的最短路径。
 *
 * @param p1 起点（货物）。
 * @param p2 目标港口。
 * @return 表示存放从 p1 到 berthIdx 港口的最短路径的方向向量的路径。
 * 如果没有路径，则返回空向量，按理不可能为空。
 */
vector<Direct> bfsPaths(Point p1, int berthIdx)
{
    vector<Direct> paths;
    Point curPoint = p1;

    // // 打乱数组顺序
    // vector<int> nums = {0, 1, 2, 3};
    // std::random_device rd;
    // std::mt19937 g(rd());
    // std::shuffle(nums.begin(), nums.end(), g);
    while (true)
    {
        for (int dir = 0; dir < 4; dir++)
        {
            if (isVaild(curPoint.first, curPoint.second, (Direct)dir) && dists[berthIdx][curPoint.first + dx[dir]][curPoint.second + dy[dir]] < getDistByPoint(berthIdx, curPoint))
            {
                curPoint.first += dx[dir];
                curPoint.second += dy[dir];
                paths.push_back((Direct)dir);
                if (getDistByPoint(berthIdx, curPoint) == 0)
                {
                    return paths;
                }
            }
        }
    }
    return {};
}



/**
  * 
  * @brief 判断一个泊位距离为1和2的可达点，用于判断泊位附近是否开阔
  *
  * @param x 位置的 x 坐标。
  * @param y 位置的 y 坐标。
  * @param berth_id 泊位ID。
  * @return 如果位置宽敞则为 true，否则为 false。
  * 
  * @note 该函数用于初始化固定船舶位置
  */
std::pair<int, int> berthSpaciousness(int berth_id) {
    int reachable1 = 0, reachable2 = 0;
    for (int dx = -2; dx < 6; ++dx) {
        for (int dy = -2; dy < 6; ++dy) {
            int nx = berths[berth_id].x + dx, ny = berths[berth_id].y + dy;
            if (isVaild(nx,ny,pause) && isRobotAccessible(nx,ny)) {
                if (dists[berth_id][nx][ny] == 1) {
                    ++reachable1;
                } else if (dists[berth_id][nx][ny] == 2)
                {
                    ++reachable2;
                }
            }
        }
    }
    std::pair<int, int> res = std::make_pair(reachable1, reachable2);
    return res;
}