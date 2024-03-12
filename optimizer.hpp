#pragma once
#include "const.h"
/*
    实现最优化相关的算法：选择港口、船舶、物品等
*/

// 选择固定港口，在初始化BFS后调用
vector<int> selectBerth() {
    //全局变量 dists[berth_num][N][N]; 记录任一点到港口i的最短距离
    //全局变量 berths[berth_num + 10]
    // 函数实现：现在要求按照哈夫曼树的思路选择港口，即选择最小的两个港口，然后合并成一个新的港口组，直到只剩下五个港口
    // 然后，每个港口组选择装载速度最大的港口
    // 返回值：返回选择的港口的id的vector
    vector<int> res;
    // 使用优先队列存储港口组，队列中的元素是一个pair，第一个元素是港口组的最小距离，第二个元素是港口组中的港口id
    priority_queue<pair<int, vector<int>>, vector<pair<int, vector<int>>>, greater<pair<int, vector<int>>>> pq;
    // 初始化，每个港口都是一个港口组
    for (int i = 0; i < berth_num ; i++) {
        pq.push({dists[i][berths[i].x][berths[i].y], {i}});
    }
    // 合并港口组，直到只剩下五个港口组
    while (pq.size() > 5) {
        // 取出距离最小的两个港口组
        auto group1 = pq.top(); pq.pop();
        auto group2 = pq.top(); pq.pop();
        // 合并两个港口组
        group1.second.insert(group1.second.end(), group2.second.begin(), group2.second.end());
        // 计算新的港口组的最小距离
        int min_dist = INT_MAX;
        for (int i : group1.second) {
            for (int j : group1.second) {
                if (i != j) {
                    min_dist = min(min_dist, dists[j][berths[i].x][berths[i].y]);
                }
            }
        }
        // 将新的港口组放入队列
        pq.push({min_dist, group1.second});
    }
    // 选择每个港口组的代表港口
    while (!pq.empty()) {
        auto group = pq.top(); pq.pop();
        int min_time = INT_MAX;
        int best_berth = -1;
        for (int i : group.second) {
            if (berths[i].transport_time < min_time) {
                min_time = berths[i].transport_time;
                best_berth = i;
            }
        }
        // 将代表港口的id加入结果
        res.push_back(best_berth);
    }
    for (int i = 0; i < 5; i++) { // 或者考虑用指针数组
        selected_berth[i] = res[i]; 
    }
    return res;

    
}