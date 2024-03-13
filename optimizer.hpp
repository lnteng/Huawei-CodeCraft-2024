#pragma once
#include "const.h"
/*
    实现最优化和选择相关的算法：选择港口、船舶、物品等
*/

// 选择固定港口，在初始化BFS后调用
void InitselectBerth() {
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
    return;
}

int shipBackBerth (int boatId) { 
    // 选择权重最大的港口，在接近船舶容量的情况，权重= 船舶剩余货物/运送时间
    //船舶Id用于决定港口货物都小于船舶容量的情况
    if (boats[boatId].status != 0 && boats[boatId].pos != -1) {
        return -1; // 船舶不能前往港口
    }
    int best_berth = selected_berth[boatId]; // 设置默认港口，用于初始船舶位置
    for (int i = 0; i < boat_num; i++) {
        if (berths[selected_berth[i]].remain_goods_num > boat_capacity * boat_return_weight) {
            best_berth = berths[selected_berth[i]].remain_goods_num/berths[selected_berth[i]].transport_time
            > berths[best_berth].remain_goods_num/berths[best_berth].transport_time ? selected_berth[i] : best_berth;
        }
    }
    return best_berth;
}


// TODO : 添加物品属性，物品出现的时候确定固定港口id

// TODO 选出区域内的最佳物品，参数需要包含泊位ID和机器人位置
