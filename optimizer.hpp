#pragma once
#include "const.h"
/*
    实现最优化和选择相关的算法：选择港口、船舶、物品等
*/

// 选择货物
Point pickGood(int bIdx, int zhenId) {
    Point p;
    int maxPriority = 0;
    for (const auto& gd : gds) {
        // good 不需要考虑 good 超时删除问题
        if (gd.second.end_time < zhenId) {
            // gds_flag[p] = true;
            continue;
        }
        if (gds_flag[gd.first] == true) {
            continue;
        }
        int dist = dists[bIdx][gd.first.first][gd.first.second];
        int newPriority = gd.second.value / dist;
        if (1 * dist + zhenId > gd.second.end_time) { // 当前机器人来不及处理该 good
            continue;
        }
        if (newPriority > maxPriority) {
            p = gd.first;
            maxPriority = newPriority;
        }
    }
    gds_flag[p] = true;
    return p;
}

// 选择港口
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
    // 初始化地图点位所属泊位区域
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            berth_field[i][j] = locateBelongBerth(make_pair(i, j));
        }
    }
    // for (int x = 0; x < n; x++) {
    //     std::ostringstream oss;
    //     oss << "[";
    //     for (int y = 0; y < n; y++) {
    //         oss << berth_field[x][y] << "\t";
    //     }
    //     oss << "]";
    //     logger.log(INFO, oss.str());
    // }
    
    return;
}

// 初始化机器人所属区域和路径 //TODO 测试
void BFSPathSearch (int robotIdx, int selected_berthIdx) { // 机器人前往区域路径，进入区域后，机器人会自动选择货物目标
    Robot& robot = robots[robotIdx]; 
    Point pRobut = make_pair(robot.x, robot.y); // 模拟机器人位置
    vector<Direct> paths;
    vector<int> nums = {0, 1, 2, 3};
    // 随机数生成器
    std::random_device rd;
    std::mt19937 g(rd());
    // 打乱数组顺序
    std::shuffle(nums.begin(), nums.end(), g);
    do{
        for (int dir: nums) {
            if (isVaild(pRobut.first, pRobut.second, (Direct)dir) 
                && dists[selected_berth[selected_berthIdx]][pRobut.first + dx[dir]][pRobut.second + dy[dir]] 
                    < dists[selected_berth[selected_berthIdx]][pRobut.first][pRobut.second]) { 
                paths.push_back((Direct)dir); 
                if (berth_field[pRobut.first][pRobut.second] == selected_berthIdx) {
                    reverse(paths.begin(), paths.end()); // 出栈顺序恢复为路径
                    robot.newPath(paths);
                    // logger.log(INFO, formatString("paths size:{}", paths.size()));
                    return;
                }
                pRobut.first += dx[dir]; //预测机器人移动后的位置，会在边界后多走几步
                pRobut.second += dy[dir];
            }   
        }
    }while(berth_field[pRobut.first][pRobut.second] == selected_berthIdx);
    
    


}
void InitRobot() {
    // 初始化机器人所属区域
    // 思路：从dists[berth_num][N][N]获取任意点到任意港口的最短距离，现在希望将10个机器人，对10个机器人分配5个港口
    // 现在要求用开销小且简洁的代码实现机器人分配到港口，要求每个港口至少有一个机器人，所有机器人移动的总距离最小
    // 返回值：返回机器人分配的港口的id的vector

    // 计算每个港口到机器人的最大距离,距离最大的港口先分配机器人
    // 创建pair<int,int>数组存储每个港口到机器人的最大距离
    pair<int, int> max_dist[select_berth_num];
    for (int select_berth_id = 0; select_berth_id < select_berth_num; select_berth_id++) {
        max_dist[select_berth_id] = make_pair(select_berth_id, 0);
        for (int robot_id = 0; robot_id < robot_num; robot_id++) {
            if (dists[selected_berth[select_berth_id]][robots[robot_id].x][robots[robot_id].y] == INT16_MAX) { //和BFS默认的不可达距离保持一致
                continue;
            }
            max_dist[select_berth_id].second = max(max_dist[select_berth_id].second, dists[selected_berth[select_berth_id]][robots[robot_id].x][robots[robot_id].y]);
            logger.log(formatString("max_dist[select_berth_id].second:{}",max_dist[select_berth_id].second));
        }
    }
    sort(max_dist, max_dist + select_berth_num, [](pair<int, int> a, pair<int, int> b) {
        return a.second > b.second;
    });
    for (int i = 0; i < select_berth_num; i++) { // 选五个机器人
        // 港口选择距离最小的机器人
        int min_dist = INT_MAX;
        int best_robot = -1;
        for (int j = 0; j < robot_num; j++) { 
            if (dists[selected_berth[max_dist[i].first]][robots[j].x][robots[j].y] < min_dist
            && robots[j].path.size() == 0) { // TODO:确认机器人和港口位置不会重叠
                min_dist = dists[selected_berth[max_dist[i].first]][robots[j].x][robots[j].y];
                best_robot = j;            
            }
        }
        logger.log(INFO,formatString("robot {} ->berth {} :{}", best_robot, max_dist[i].first,dists[selected_berth[max_dist[i].first]][robots[best_robot].x][robots[best_robot].y]));
        BFSPathSearch(best_robot, max_dist[i].first); //设置机器人初始路径
    }
    // TODO：剩下五个机器人的处理，初步考虑就近找船舶
    

    // 机器人分配到港口
        

}

int nearBerth(Point curPoint) {
    int bIdx = 0;
    int minDist = INT16_MAX;
    for (int i: selected_berth) {
        int newDist = dists[i][curPoint.first][curPoint.second];
        if (newDist < minDist) {
            bIdx = i;
            minDist = newDist;
        }
    }
    return bIdx; //返回对应港口的dist下标
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

// TODO 选出区域内的最佳物品，参数需要包含泊位ID和机器人位置
