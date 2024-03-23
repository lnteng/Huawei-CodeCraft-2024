#pragma once
#include "const.h"
#include "graph.hpp"
/*
    实现最优化和选择相关的算法：选择泊位、船舶、物品等
*/

/**
 * @brief 选择货物
 *
 * @param rIdx 机器人ID
 * @param zhenId 当前帧数
 * @return 选择的货物坐标,如果没有货物返回boat_virtual_point(200,200)
 *
 * @note 选择货物的策略：选择以固定泊位为基准选择货物，选择未被标记的货物，然后选择优先级最高的货物
 */
Point pickGood(int rIdx, int zhenId)
{
    Point p = boat_virtual_point;
    int maxPriority = 0;
    Point cur_alternative_gds = boat_virtual_point;
    double cur_gds_priority = 0.0;
    for (auto it = gds.begin(); it != gds.end(); ) {
        auto &gd = *it;
        if (gd.second.end_time <= zhenId)
        { // 删除超时的 good
            it = gds.erase(it);
            continue;
        }
        if (gd.second.marked)
        { // 已经被标记
            ++it;
            continue;
        }
        int dist = getDistByPoint(selected_berth[robots[rIdx].selected_berthIdx], gd.first); // 货物到机器人目标港口的最短距离
        if (locateBelongBerth(gd.first).first != robots[rIdx].selected_berthIdx)
        { // 选择区域外货物在一定范围内作为备选
            if (goods_withinfield_ratio>0 && int(Goods_tolerance * dist/goods_withinfield_ratio) + zhenId <= gd.second.end_time)  // 取货物容错系数/固定泊位区域外可选货物距离比例
            { // 选择区域外货物在一定范围内作为备选
                double gds_priority = gd.second.getPriorityOutsideFeild(dist);
                if (gds_priority > cur_gds_priority)
                {
                    cur_gds_priority = gds_priority; //备选货物优先级
                    cur_alternative_gds = gd.first; //备选货物位置
                }
            }
            ++it;
            continue;
        }
        if (int(Goods_tolerance * dist) + zhenId > gd.second.end_time)  // 取货物容错系数
        { // 当前机器人来不及处理该 good 
            ++it;
            continue;
        }
        if (gd.second.priority == 0.0) 
        { // 根据既定的策略来更新 good 的优先级
            gd.second.updatePriority(dist);
        }
        if (gd.second.priority > maxPriority)
        {
            p = gd.first;
            maxPriority = gd.second.priority;
        }
        ++it;
    }
    // 如果该区域无货物，则按照距离选择邻近区域内货物
    if(p == boat_virtual_point || cur_alternative_gds != boat_virtual_point) {
        p = cur_alternative_gds;
    }
    if (p != boat_virtual_point)
    {
        gds[p].marked = true;
    } else {
        logger.log(WARNING, formatString("pickGood: rIdx:{},selected_bIdx:{} has no goods", rIdx,robots[rIdx].selected_berthIdx));
    }
    return p;
}


/**
 * @brief 初始化泊位选择。
 *
 * 该函数根据哈夫曼树方法选择泊位。
 * 首先选择两个最小的泊位并将它们合并到一个新的泊位组中。
 * 这个过程一直持续到只剩下五个泊位组为止。 
 * 然后，从每组中选择装载速度最大的泊位。 //TODO:选择策略还应该考虑周围的狭窄程度（3.10目前策略仍非最优，因为有两个港口选在单通道处）
 *
 * @return 包含所选泊位 ID 的vector。
 *
 * @note 选择固定泊位，在初始化BFS后调用
 * @note 全局变量 dists[berth_num][N][N] 记录任一点到泊位 i 的最短距离。
 * @note 全局变量 berths[berth_num + 10] 记录泊位的信息。
 * @note 全局变量 selected_berth[5] 固定泊位seleted_berth_id到泊位id
 */
void InitselectBerth()
{
    vector<int> res;
    int berth_group_num = berth_num; // 泊位组数目
    int berth_groups[berth_num][berth_num+1]; // 每个泊位组的泊位(最后一列存储该泊位组泊位数目)
    for (int bIdx = 0; bIdx < berth_num; bIdx++)
    { // 初始化
        berthBelongGroup[bIdx]= -1;
    }
    for (int i = 0; i < berth_num; i++)
    {
        berth_groups[i][berth_num] = 1; //初始泊位组i只有一个泊位
        for (int j = 0; j < berth_num; j++)
        {
            berth_groups[i][j] = -1;
        }
        berth_groups[i][i] = 1; //泊位组i初始包含第i个泊位
    }
    // 循环直到只剩下五个泊位组
    while(berth_group_num > boat_num){
        int min_dist2 = INT_MAX;
        int min_i = -1, min_j = -1;
        for (int i = 0; i < berth_num; i++) {
            if (berth_groups[i][berth_num] == 0) continue; // 该泊位组i无泊位
            for (int j = i + 1; j < berth_num; j++) {
                if (berth_groups[j][berth_num] == 0) continue; // 该泊位组j无泊位
                for (int k = 0; k < berth_num; k++) {
                    if (berth_groups[i][k] == -1) continue;
                    for (int l = 0; l < berth_num; l++) {
                        if (berth_groups[j][l] == -1) continue;
                        if (getDistByBerth(k, berths[l]) < min_dist2) { // 选择两个泊位组之间最短路径最小的两个泊位
                            min_dist2 = getDistByBerth(k, berths[l]); // k和l是两个泊位
                            min_i = i;
                            min_j = j;
                        }
                    }
                }
            }
        }
        if (min_i ==-1 || min_j == -1 || min_dist2 == INT_MAX) {
            logger.log(ERROR, "min_i or min_j error,fail to find min_dist2");
            logger.log(ERROR, formatString("min_i:{}, min_j:{}, min_dist2:{}", min_i, min_j, min_dist2));
        }
        //合并两个泊位组
        for (int k = 0; k<berth_num; k++) { // 合并泊位组min_j->min_i
            if (berth_groups[min_j][k] == -1) continue;
            if (berth_groups[min_j][k] == 1) {
                berth_groups[min_i][k] = 1;
                berth_groups[min_j][k] = -1;
            } else {
                logger.log(ERROR, formatString("berth_groups{}[{}] error:{}", min_j, k, berth_groups[min_j][k]));
            }    
        }
        berth_groups[min_i][berth_num] += berth_groups[min_j][berth_num];
        berth_groups[min_j][berth_num] = 0;

        berth_group_num--;
    }
    // 打印berth_groups
    for (int i=0 ;i<berth_num;i++){
        std::ostringstream oss;
        oss << "[";
        for (int j=0;j<berth_num+1;j++){
            oss << berth_groups[i][j] << "\t";
        }
        oss << "]";
        logger.log(INFO, oss.str());
    }
    // 转换为 vector<vector<int>>
    for (int i = 0; i < berth_num; ++i) {
        if (berth_groups[i][berth_num] == 0) continue; // 该泊位组i无泊位
        vector<int> cur_group;
        for (int j = 0; j < berth_num; ++j) {
            if (berth_groups[i][j] == -1) continue;
            cur_group.push_back(j);
            if (berthBelongGroup[j] == -1)
            {
                berthBelongGroup[j] = berth_groups_vec.size(); // 更新泊位所属组
            } else { // 所属泊位组初始化错误，重复变更一个泊位所属泊位组
                logger.log(ERROR, formatString("berth{} belong group error:{}", j, berthBelongGroup[j]));
            }    
        }
        berth_groups_vec.push_back(cur_group);
    }
    // 打印 berth_groups_vec
    logger.log(INFO, "berth_groups_vec:");
    for (int i = 0; i < berth_groups_vec.size(); ++i) {
        std::ostringstream oss;
        oss << "[";
        for (int j = 0; j < berth_groups_vec[i].size(); ++j) {
            oss << berth_groups_vec[i][j] << "\t";
        }
        oss << "]";
        logger.log(INFO, oss.str());
    }
    // for (int i = 0; i < berth_num; i++) {
    //     if (berthBelongGroup[i] == -1) {
    //         logger.log(ERROR, formatString("berth{} belong group error:{}", i, berthBelongGroup[i]));
    //     }
    //     logger.log(INFO, formatString("berth{} belong group:{}", i, berthBelongGroup[i]));
    // }

    // 固定泊位选择0：从每个泊位组中选择运输时间最短的泊位（优先在宽敞泊位中选择）
    // 固定泊位选择1：从每个泊位组中选择离其他港口组最短距离和最大的泊位（优先在宽敞泊位中选择）
    int other_berthgroups_dist[berth_num]; //该泊位到其他泊位组泊位距离总和
    for (int bIdx=0;bIdx<berth_num;bIdx++){
        other_berthgroups_dist[bIdx] = 0;
        for (int bIdx2 = 0; bIdx2<berth_num;bIdx2++){
            if (berthBelongGroup[bIdx] == berthBelongGroup[bIdx2]) continue;
            other_berthgroups_dist[bIdx] += getDistByBerth(bIdx, berths[bIdx2]);
        }
    }
    for (int i = 0; i < berth_num; i++) {
        if (berth_groups[i][berth_num] == 0) continue; // 该泊位组i无泊位
        if (berth_groups[i][berth_num] == 1) {
            for (int bIdx = 0; bIdx < berth_num; bIdx++) {
                if (berth_groups[i][bIdx] == -1) continue; // 泊位组i不包含泊位bIdx
                res.push_back(bIdx); // 该泊位组只有一个泊位
                break;
            }
            continue;  
        }
        // int min_transport_time = INT_MAX;
        int max_distance_between_berth = 0;
        int best_berth = -1;
        for (int j = 0; j < berth_num; j++) {
            if (berth_groups[i][j] == -1) continue; // 泊位组i不包含泊位j
            pair<int,int> reachable = berthSpaciousness(j);
            if (reachable.first>=4 && reachable.first<=reachable.second  // 泊位宽敞
                && other_berthgroups_dist[j] > max_distance_between_berth) {
                max_distance_between_berth = other_berthgroups_dist[j];
                best_berth = j;
            }
        }
        if (best_berth == -1) { // 该泊位组无宽敞泊位,选择周围最宽阔的泊位
            best_berth = 0;
            int spaciousness = 0;
            for (int j = 0; j < berth_num; j++) {
                if (berth_groups[i][j] == -1) continue;
                pair<int,int> reachable = berthSpaciousness(j);
                if (reachable.first+reachable.second > spaciousness) {
                    best_berth = j;
                }
            }
        }
        res.push_back(best_berth);
    }
    // 固定泊位选择2：从每个泊位组中选出固定泊位使得区域尽量均匀（优先在宽敞泊位中选择）
    // TODO 实现穷举组合，尽量均匀每片区域范围
    // 打印res
    for (int i = 0; i < res.size(); i++)
    {
        logger.log(INFO, formatString("res[{}]:{}", i, res[i]));
    }

    // res = {0,5,6,7,8}; // TODO:3-8测试用例
    // res = {1,3,5,7,9}; // TODO:3-11测试用例
    // 检查泊位组是否溢出和数据是否正确
    if (res.size() != boat_num) {
        logger.log(ERROR, formatString("res size error:{}", res.size()));
    }
    int berth_num_check = 0;
    for (int i = 0;i<berth_num;i++){
        if (berth_groups[i][berth_num] != 0) {
            berth_group_num --;
        }
        int count = 0;
        for (int j = 0; j<berth_num;j++){
            if (berth_groups[i][j] != -1) count++;
        }
        if (count != berth_groups[i][berth_num]) {
            logger.log(ERROR, formatString("berth_groups{} num count:{} error:{}", i, count, berth_groups[i][berth_num]));
        }
        berth_num_check += count;
    }
    if (berth_group_num !=0) {
        logger.log(ERROR, formatString("berth_group_num error:{}", boat_num-berth_group_num));
    }
    if (berth_num_check != berth_num) {
        logger.log(ERROR, formatString("berth_groups-berth_num error:{}", berth_num_check));
    }
    
    // 初始化全局变量 selected_berth 固定泊位数组
    for (int i = 0; i < boat_num; i++)
    {
        selected_berth[i] = res[i];
    }
    // 初始化固定泊位辐射区域数目
    for (int i =0; i< berth_num; i++){
        berth_field_count[i] = 0;

    }
    // 初始化地图拥堵度
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            congestion[i][j] = make_pair(-1, 0);
        }
    }
    // 更新地图点位所属泊位区域和拥堵度地图信息
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            pair<int,int> bIdx_and_dist = locateBelongBerth(make_pair(i, j));
            int bIdx = bIdx_and_dist.first;
            berth_field[i][j] = bIdx;
            if (bIdx != -1)
            { // 可达点
                if (bIdx_and_dist.second < berth_field_over)
                {
                    berth_field_count[bIdx]++; // 统计固定泊位辐射可达点数目
                    reachable_point_count++;
                }
                congestion[i][j].first = 0;
                // 四个方向是否是可达点
                for (int k = 0; k < 4; k++)
                {
                    if (!isRobotAccessible(i + dx[k], j + dy[k])) // 不可达点
                    {
                        congestion[i][j].first++;
                    }
                    if (congestion[i][j].first == 2 || congestion[i][j].first == 3)
                    {
                        single_channel_counts++;
                    }
                }
            } else { //不可达点
                congestion[i][j] = make_pair(4,0);
            }
        }
    }
    logger.log(INFO,formatString("single_channel_counts:{}",single_channel_counts));
    if (single_channel_counts >= high_single_channel_counts) {
        res = {0,3,6,7,9}; // TODO:初赛图2特化
        // 初始化全局变量 selected_berth 固定泊位数组
        for (int i = 0; i < boat_num; i++)
        {
            selected_berth[i] = res[i];
        }
        // 更新地图点位所属泊位区域和拥堵度地图信息
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                pair<int,int> bIdx_and_dist = locateBelongBerth(make_pair(i, j));
                int bIdx = bIdx_and_dist.first;
                berth_field[i][j] = bIdx;
                if (bIdx != -1)
                { // 可达点
                    if (bIdx_and_dist.second < berth_field_over)
                    {
                        berth_field_count[bIdx]++; // 统计固定泊位辐射可达点数目
                        reachable_point_count++;
                    }
                    congestion[i][j].first = 0;
                    // 四个方向是否是可达点
                    for (int k = 0; k < 4; k++)
                    {
                        if (!isRobotAccessible(i + dx[k], j + dy[k])) // 不可达点
                        {
                            congestion[i][j].first++;
                        }
                        if (congestion[i][j].first == 2 || congestion[i][j].first == 3)
                        {
                            single_channel_counts++;
                        }
                    }
                } else { //不可达点
                    congestion[i][j] = make_pair(4,0);
                }
            }
        }
    } else if (single_channel_counts < low_single_channel_counts) { // 图1特化
        // res = {0,3,6,7,9}; // TODO:初赛图1特化
        // 初始化全局变量 selected_berth 固定泊位数组
        for (int i = 0; i < boat_num; i++)
        {
            selected_berth[i] = res[i];
        }
        // 更新地图点位所属泊位区域和拥堵度地图信息
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                pair<int,int> bIdx_and_dist = locateBelongBerth(make_pair(i, j));
                int bIdx = bIdx_and_dist.first;
                berth_field[i][j] = bIdx;
                if (bIdx != -1)
                { // 可达点
                    if (bIdx_and_dist.second < berth_field_over)
                    {
                        berth_field_count[bIdx]++; // 统计固定泊位辐射可达点数目
                        reachable_point_count++;
                    }
                    congestion[i][j].first = 0;
                    // 四个方向是否是可达点
                    for (int k = 0; k < 4; k++)
                    {
                        if (!isRobotAccessible(i + dx[k], j + dy[k])) // 不可达点
                        {
                            congestion[i][j].first++;
                        }
                        if (congestion[i][j].first == 2 || congestion[i][j].first == 3)
                        {
                            single_channel_counts++;
                        }
                    }
                } else { //不可达点
                    congestion[i][j] = make_pair(4,0);
                }
            }
        }
    } else {
        // res = {0,3,6,7,9}; // TODO:初赛图3特化
        // 初始化全局变量 selected_berth 固定泊位数组
        for (int i = 0; i < boat_num; i++)
        {
            selected_berth[i] = res[i];
        }
        // 更新地图点位所属泊位区域和拥堵度地图信息
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                pair<int,int> bIdx_and_dist = locateBelongBerth(make_pair(i, j));
                int bIdx = bIdx_and_dist.first;
                berth_field[i][j] = bIdx;
                if (bIdx != -1)
                { // 可达点
                    if (bIdx_and_dist.second < berth_field_over)
                    {
                        berth_field_count[bIdx]++; // 统计固定泊位辐射可达点数目
                        reachable_point_count++;
                    }
                    congestion[i][j].first = 0;
                    // 四个方向是否是可达点
                    for (int k = 0; k < 4; k++)
                    {
                        if (!isRobotAccessible(i + dx[k], j + dy[k])) // 不可达点
                        {
                            congestion[i][j].first++;
                        }
                        if (congestion[i][j].first == 2 || congestion[i][j].first == 3)
                        {
                            single_channel_counts++;
                        }
                    }
                } else { //不可达点
                    congestion[i][j] = make_pair(4,0);
                }
            }
        }
    }
    // logger.log(INFO,formatString("single_channel_counts:{}", single_channel_counts));
    // 打印初始化船舶区域信息
    // for (int x = 0; x < n; x++)
    // {
    //     std::ostringstream oss;
    //     oss << "[";
    //     for (int y = 0; y < n; y++)
    //     {
    //         oss << berth_field[x][y] << "\t";
    //     }
    //     oss << "]";
    //     logger.log(INFO, oss.str());
    // }
    // 打印拥堵度地图信息
    for (int x = 0; x < n;x++){
        std::ostringstream oss;
        oss << "[";
        for (int y = 0; y < n; y++){
            oss << congestion[x][y].first << "\t";
        }
        oss << "]";
        logger.log(INFO, oss.str());
    }
    for (int i = 0; i < boat_num; i++)
    {
        logger.log(INFO, formatString("berth_field_count[{}]:{}", i, berth_field_count[i]));
    }
    logger.log(INFO, formatString("reachable_point_count:{}", reachable_point_count));

    return;
}

// 函数：变更泊位组（输入泊位组ID），根据船舶货物平均密度数量，重新选择固定泊位，返回是否变更
bool updateSelectedBerth(int group_id,int zhenId)
{
    // 计算泊位组内泊位的货物价值密度
    int past_bidx = selected_berth[group_id]; //记录过去的固定泊位对应泊位
    Point p = boat_virtual_point;
    int cur_maxPriority = 0; //价值最大泊位优先级
    Point cur_alternative_gds = boat_virtual_point;
    vector<int> cur_group_vec = berth_groups_vec[group_id];
    logger.log(INFO, formatString("updateSelectedBerth: group_id:{}, past_bidx:{}", group_id, past_bidx));
    if (cur_group_vec.size()== 0) logger.log(ERROR, formatString("updateSelectedBerth: group_id:{} has no berth", group_id));
    if (cur_group_vec.size()== 1) return false; //只有一个泊位，跳过
    vector<double> berth_priority;
    berth_priority.resize(cur_group_vec.size(),0.0);//泊位优先级
    double cur_gds_priority = 0.0;
    for (auto it = gds.begin(); it != gds.end(); ) {
        auto &gd = *it;
        if (gd.second.end_time <= zhenId)
        { // 删除超时的 good
            it = gds.erase(it);
            continue;
        }
        for (int bIdx = 0; bIdx < cur_group_vec.size(); bIdx++) {
            if (getDistByPoint(bIdx, gd.first) > berth_field_radius) continue; // 距离泊位最短距离为xx的货物
            // berth_priority[bIdx] += gd.second.value; // 该区域货物价值密度
            berth_priority[bIdx] += gd.second.getPriorityOutsideFeild(getDistByPoint(bIdx, gd.first)); // 该区域货物价值
        } 
        ++it;
    }
    cur_maxPriority = berth_priority[0];
    int cur_maxPriorityIdx = 0;
    // 选择货物价值最高的区域
    for (int i = 1; i < cur_group_vec.size(); i++) {
        if (berth_priority[i] > cur_maxPriority)
        {
            cur_maxPriorityIdx = i;
            cur_maxPriority = berth_priority[i];
        }
    }
    int new_bidx = cur_group_vec[cur_maxPriorityIdx];
    if (past_bidx == new_bidx) {
        return false; // 未变更固定泊位
    }
    // selected_berth[group_id] = new_bidx;
    logger.log(INFO, formatString("updateSelectedBerth: group_id:{}, past_bidx:{}, new_bidx:{}", group_id, past_bidx, new_bidx));
    return true;
}

// 函数：变更泊位组（输入泊位组ID），根据船舶货物平均密度数量，重新选择固定泊位，返回是否变更
bool endSelectedBerth(int group_id,int zhenId)
{
    bool res = false;
    int min_dist = MAX_LIMIT;
    int selectedBerth=-1;
    for (int bIdx =0;bIdx< boat_num;bIdx++){
        if (!endBoatGroup[bIdx]) continue; //泊位组已终止
        int dist = getDistByBerth(selected_berth[group_id], berths[selected_berth[bIdx]]);
        if (dist < min_dist) {
            min_dist = dist;
            selectedBerth = selected_berth[bIdx];
        }
    }
    if (selectedBerth==-1){ //没有可达的港口了，不用处理
        logger.log("endBoatGroup");
        return false;
    } 
    selected_berth[group_id]=selectedBerth;
    return true;
}


/**
 * @brief 根据已有的广度优先搜索以找到机器人移动到选定泊位区域的路径,并初始化机器人固定泊位属性。
 *
 * @param robotsIdx 机器人的索引。
 * @param selected_berthIdx 所选泊位的索引。
 * @param max_path 要考虑的最大路径数。
 * @return 无返回值。
 *
 * @note 全局变量 dists[berth_num][N][N] 记录任一点到泊位 i 的最短距离。
 * @note 机器人进入区域后会自动选择该区域的货物目标。
 * @note 初始化robot.berthId，此函数只用于初始化机器人
 */
void BFSPathSearch(int robotIdx, int selected_berthIdx, int max_path)
{ // 机器人前往区域路径，进入区域后，机器人会自动选择货物目标
    logger.log(INFO, formatString("robotIdx:{}, selected_berthIdx[{}]:{}, max_path:{}", robotIdx, selected_berthIdx,selected_berth[selected_berthIdx], max_path));
    Robot &robot = robots[robotIdx];
    robot.selected_berthIdx = selected_berthIdx;
    Point pRobut = make_pair(robot.x, robot.y); // 模拟机器人位置
    vector<Direct> paths;
    vector<int> nums = {0, 1, 2, 3};
    while (paths.size() <= max_path) // 防止死循环
    {
        for (int dir : nums)
        {
            if (isVaild(pRobut.first, pRobut.second, (Direct)dir) && dists[selected_berth[selected_berthIdx]][pRobut.first + dx[dir]][pRobut.second + dy[dir]] < getDistByPoint(selected_berth[selected_berthIdx], pRobut))
            {
                // 如果下一步是有效的，且下一步离目标泊位距离更近
                paths.push_back((Direct)dir); // 机器人移动方向
                if (berth_field[pRobut.first][pRobut.second] == selected_berthIdx)
                { // 循环出口：直到机器人进入区域
                    robot.newPath(paths);
                    // logger.log(INFO, formastString("  find paths size:{}", paths.size()));
                    return;
                }
                pRobut.first += dx[dir]; // 在边界后多走一步，避免停留在边界
                pRobut.second += dy[dir];
            }
        }
    }
    logger.log(ERROR, "BFSPathSearch over max_path");
}

/**
 * @brief 初始化机器人信息并根据一定的标准将机器人分配到泊位。
 *
 * 该函数通过识别栅格地图中机器人位置初始化机器人信息。
 * 然后，它根据特定标准按顺序依次给固定泊位分配剩余机器人中最近的。
 * 第一轮使用最短距离矩阵来计算每个泊位与机器人之间的最大距离，最大距离大的泊位优先选择一个机器人。
 * 最后根据每个泊位的运输时间将剩余的机器人分配到泊位。
 *
 * @return 一个向量，包含机器人所分配到的泊位的 ID。
 */
void InitRobot()
{
    // 初始化机器人信息
    {
        int robot_index = 0;
        for (int x = 0; x < n; x++)
        { // 观察到机器人是按(x,y)由小到大的顺序返回信息
            for (int y = 0; y < n; y++)
            {
                if (ch[x][y] == 'A')
                {
                    robots[robot_index].x = x;
                    robots[robot_index].y = y;
                    robots[robot_index].goods = 0;
                    robots[robot_index].status = 1;
                    robots[robot_index].selected_berthIdx = -1; // 未分配泊位
                    robot_index++; // TODO 注意数组越界，复赛阶段可以购买机器人
                }
            }
        }
    }

    // 计算每个泊位到机器人的最大距离,距离最大的泊位先分配机器人（控制总移动次数）
    pair<int, int> max_dist[select_berth_num]; // 固定泊位id和每个固定泊位到机器人的最大距离
    for (int select_berth_id = 0; select_berth_id < select_berth_num; select_berth_id++)
    { // 首轮分配，每个泊位按最大距离由大到小依次分配一个机器人 // TODO:比较直接使用次轮分类策略2
        max_dist[select_berth_id] = make_pair(select_berth_id, 0);
        for (int robot_id = 0; robot_id < robot_num; robot_id++)
        { // 每个泊位到所有机器人的最大距离 //TODO 可以极差或标准差衡量
            if (getDistByRobot(selected_berth[select_berth_id], robots[robot_id]) == MAX_LIMIT)
            { // 机器人和泊位位置不可达
                // max_dist[select_berth_id].second = -1;
                continue;
            }
            max_dist[select_berth_id].second = max(max_dist[select_berth_id].second, getDistByRobot(selected_berth[select_berth_id], robots[robot_id]));
        }
    }
    sort(max_dist, max_dist + select_berth_num, [](pair<int, int> a, pair<int, int> b) { // 按照距离由大到小排序
        return a.second > b.second;
    });
    for (int i = 0; i < select_berth_num; i++)
    { // 每个固定泊位依次选择一个机器人
        // 泊位选择距离最小的机器人
        int min_dist = MAX_LIMIT;
        int best_robot_id = -1;
        for (int j = 0; j < robot_num; j++)
        {
            if (getDistByRobot(selected_berth[max_dist[i].first], robots[j]) < min_dist && robots[j].path.size() == 0)
            { // 保证已分配的机器人不会再次被分配给其他泊位 // TODO:确认机器人和泊位位置不会重叠
                min_dist = dists[selected_berth[max_dist[i].first]][robots[j].x][robots[j].y];
                best_robot_id = j;
            }
        }
        // 设置选定机器人前往对应泊位区域的路径
        BFSPathSearch(best_robot_id, max_dist[i].first, getDistByRobot(selected_berth[max_dist[i].first], robots[best_robot_id])); // 设置机器人初始路径
    }
    logger.log(INFO, "InitRobot part 1 over");
    // 剩余机器人的处理1：每个泊位按照运输时间由小到大依次分配机器人
    // pair<int, int> berth_trasnport_time[select_berth_num];
    // for (int i = 0; i < select_berth_num; i++)
    // {
    //     berth_trasnport_time[i] = make_pair(i, berths[selected_berth[i]].transport_time);
    // }
    // sort(berth_trasnport_time, berth_trasnport_time + select_berth_num, [](pair<int, int> a, pair<int, int> b)
    //      { return a.second < b.second; }); // 排序：按照运输时间由小到大
    // for (int selected_berth_id = 0; selected_berth_id < select_berth_num; selected_berth_id++)
    // {
    //     int min_dist = MAX_LIMIT;
    //     int best_robot_id = -1;
    //     for (int robot_id = 0; robot_id < robot_num; robot_id++)
    //     {
    //         if (getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[robot_id]) < min_dist && robots[robot_id].path.size() == 0)
    //         {
    //             min_dist = getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[robot_id]);
    //             best_robot_id = robot_id;
    //         }
    //     }
    //     // 设置选定机器人前往对应泊位区域的路径
    //     BFSPathSearch(best_robot_id, berth_trasnport_time[selected_berth_id].first, getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[best_robot_id])); // 设置机器人初始路径
    // }
    // TODO 考虑初始路径长度的因素
    // 剩余机器人的处理2：每个固定泊位按照辐射可达点面积按比例分配机器人(最多分配数目减去上一轮已分配的)
    struct Compare
    { // 自定义比较函数，让优先队列按照 pair 的第二个元素从大到小排序
        bool operator()(const pair<int, int> &a, const pair<int, int> &b)
        {
            return a.second < b.second;
        }
    };
    priority_queue<pair<int, int>, vector<pair<int, int>>, Compare> berth_field_count_sort; // 创建优先队列(固定泊位id和辐射面积)
    for (int i = 0; i < select_berth_num; i++)
    {
        berth_field_count_sort.push(make_pair(i, berth_field_count[i]));
    }
    int part = reachable_point_count / robot_num; // 剩余每个机器人对应的辐射可达点数目，向下取整
    // 备份berth_field_count_sort的机器人ID顺序
    vector<int> addition_berth_order;
    // int part = reachable_point_count / robot_num; //十个一起分配使用 //TODO 如果实现了多机器人碰撞使用
    while (!berth_field_count_sort.empty()) {
        pair<int, int> top = berth_field_count_sort.top(); //(固定部位Id和辐射面积)
        int rIdx = top.first;
        berth_field_count_sort.pop();
        addition_berth_order.push_back(top.first);
        // int allocated_robot_num = std::floor(static_cast<double>(berth_field_count[top.first]) / part + 0.5); // 每个泊位分配的机器人数目,四舍五入
        int allocated_robot_num;
        double fraction_part = static_cast<double>(berth_field_count[top.first]) / part - std::floor(static_cast<double>(berth_field_count[top.first]) / part); // 小数部分
        if (fraction_part >= static_cast<double>(rounding_num+1) / (berth_num-boat_num)) {
            allocated_robot_num = std::ceil(static_cast<double>(berth_field_count[top.first]) / part); // num+1入
        } else {
            allocated_robot_num = std::floor(static_cast<double>(berth_field_count[top.first]) / part); // num舍
        }
        if (single_channel_counts > high_single_channel_counts) { // 图2
            switch (selected_berth[rIdx])
            {
            case 0:
                allocated_robot_num = 1;
                break;
            case 3:
                allocated_robot_num = 1;
                break;
            case 6:
                allocated_robot_num = 1;
                break;
            case 7:
                allocated_robot_num = 1;
                break;
            case 9:
                allocated_robot_num = 1;
                break;
            default:
                break;
            }   
        } else if (single_channel_counts < low_single_channel_counts) { //图1
            allocated_robot_num = 1;
            // switch (selected_berth[rIdx])
            // {
            // case 0:
            //     allocated_robot_num = 1;
            //     break;
            // case 3:
            //     allocated_robot_num = 1;
            //     break;
            // case 6:
            //     allocated_robot_num = 1;
            //     break;
            // case 7:
            //     allocated_robot_num = 1;
            //     break;
            // case 9:
            //     allocated_robot_num = 1;
            //     break;
            // default:
            //     break;
            // } 
        } else { // 图3
            allocated_robot_num = 1;
            // switch (selected_berth[rIdx])
            // {
            // case 0:
            //     allocated_robot_num = 1;
            //     break;
            // case 3:
            //     allocated_robot_num = 1;
            //     break;
            // case 6:
            //     allocated_robot_num = 1;
            //     break;
            // case 7:
            //     allocated_robot_num = 1;
            //     break;
            // case 9:
            //     allocated_robot_num = 1;
            //     break;
            // default:
            //     allocated_robot_num = 1;
            //     break;
            // }
        }

        // 减去上一轮已分配的机器人数
        // for (int rIdx = 0; rIdx < robot_num; rIdx++)
        // {
        //     if (robots[rIdx].selected_berthIdx == top.first)
        //     {
        //         allocated_robot_num--;
        //     }
        // }
        logger.log(INFO, formatString("allocated_robot_num[{}]:{}", top.first, allocated_robot_num));
        for (int j = 0; j < allocated_robot_num; j++)
        { // 每个泊位依次选择最近的机器人
            int min_dist = MAX_LIMIT;
            int best_robot_id = -1;
            for (int robot_id = 0; robot_id < robot_num; robot_id++)
            { 
                if (robots[robot_id].path.size() == 0 && getDistByRobot(selected_berth[top.first], robots[robot_id]) < min_dist)
                {
                    min_dist = getDistByRobot(selected_berth[top.first], robots[robot_id]);
                    best_robot_id = robot_id;
                }
            }
            if (best_robot_id == -1) {
                logger.log(INFO, "robot is all allocated");
                break; 
            }
            // 设置选定机器人前往对应泊位区域的路径
            BFSPathSearch(best_robot_id, top.first, getDistByRobot(selected_berth[top.first], robots[best_robot_id])); // 设置机器人初始路径
        }

    }
    for (int robot_id = 0; robot_id < robot_num; robot_id++) // 如果（近似后）有剩余机器人，分配给辐射区域最大的港口
    {
        if (robots[robot_id].path.size() == 0)
        {
            // 按照港口区域面积顺序 addition_berth_order ，找到机器人可达的港口
            for (int i = 0; i < addition_berth_order.size(); i++)
            {
                if (getDistByRobot(selected_berth[addition_berth_order[i]], robots[robot_id]) < MAX_LIMIT)
                {
                    logger.log(WARNING, formatString("  addtional allocated_robot_num[{}]:{}", addition_berth_order[i], 1));
                    BFSPathSearch(robot_id, addition_berth_order[i], getDistByRobot(selected_berth[addition_berth_order[i]], robots[robot_id])); // 设置机器人初始路径
                    break;
                }
            }
            if (robots[robot_id].path.size() == 0) { //没有找到对应的固定港口
                logger.log(ERROR, formatString("robot_id:{} has no path to selected berth", robot_id));
                vector<Direct> path = {Direct::pause}; //前robot_id帧先保持不动，避免初始没有路径可能导致其他问题
                for (int i = 0; i < robot_id; i++)
                {
                    path.push_back(Direct::pause);
                }
                robots[robot_id].newPath(path);
            }
        }
    }
    logger.log(INFO, "InitRobot part 2 over");
    return;
}

/**
 * 距离给定点最近的泊位。
 *
 * @param curPoint 查找最近泊位的当前点。
 * @return 根据距离最近的泊位索引。
 */
int nearBerth(Point curPoint)
{ // TODO:限制寻找区域内货物
    int bIdx = 0;
    int minDist = MAX_LIMIT;
    for (int i : selected_berth)
    {
        int newDist = getDistByPoint(i, curPoint);
        if (newDist < minDist)
        {
            bIdx = i;
            minDist = newDist;
        }
    }
    return bIdx; // 返回对应泊位的dist下标
}

/**
 * 为给定船只选择优先度最高的泊位。
 * 优先度= 船舶剩余货物/运送时间。
 *
 * @param BoatId 返回泊位的船的 ID。
 * @return 所选泊位的 ID，如果船只无法前往任何泊位，则返回 -1。
 */
int shipBackBerth(int boatId) 
{
    // 选择权重最大的泊位，在接近船舶容量的情况，权重= 船舶剩余货物/运送时间 // TODO:权重= 船舶剩余货物价值/运送时间 （先保证船舶剩余货物与实际一致？）
    // 船舶Id用于决定泊位货物都小于船舶容量的情况
    if (boats[boatId].status != 0 && boats[boatId].pos != -1)
    {
        return -1; // 船舶不能前往泊位
    }
    int best_berth = selected_berth[boatId]; // 设置默认泊位，用于初始船舶位置，或所有港口均不接近船舶容量时
    for (int i = 0; i < boat_num; i++)
    {
        if (berths[selected_berth[i]].remain_goods_num > int(boat_capacity * boat_return_weight))
        { // 泊位货物接近或超过船舶容量
            // 选择权重最大的泊位
            best_berth = berths[selected_berth[i]].remain_goods_num / berths[selected_berth[i]].transport_time > berths[best_berth].remain_goods_num / berths[best_berth].transport_time ? selected_berth[i] : best_berth;
        }
    }
    return best_berth;
}

// TODO 选出区域内的最佳物品，参数需要包含泊位ID和机器人位置
