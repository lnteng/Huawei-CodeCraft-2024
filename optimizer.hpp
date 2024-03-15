#pragma once
#include "const.h"
/*
    实现最优化和选择相关的算法：选择泊位、船舶、物品等
*/

/**
 * @brief 选择货物
 *
 * @param bIdx 泊位ID
 * @param zhenId 当前帧数
 * @return 选择的货物坐标,如果没有货物返回boat_virtual_point(200,200)
 *
 */
Point pickGood(int bIdx, int zhenId)
{
    Point p = boat_virtual_point;
    int maxPriority = 0;
    for (auto it = gds.begin(); it != gds.end(); ) {
        auto &gd = *it;
        if (selected_berth[locateBelongBerth(gd.first)] != bIdx)
        { // 选择区域内货物
            ++it;
            continue;
        }
        // good 不需要考虑 good 超时删除问题 // TODO 为什么？
        if (gd.second.end_time <= zhenId)
        {
            if (gd.second.end_time == 0)
            {
                logger.log(INFO, formatString("pickGood: good end_time:{} <= zhenId:{}", gd.second.end_time, zhenId));
                logger.log(ERROR,formatString("good end_time:{} goods {} ", gd.second.end_time, gd.second.value));
                logger.log(INFO,formatString("gd.x,y{},{}",gd.first.first,gd.first.second));
            }else{
                // logger.log(ERROR,formatString("#good end_time:{} goods {} ", gd.second.end_time, gd.second.value));
                it = gds.erase(it);
                continue;
            }
        }
        if (gd.second.marked)
        { // 已经被标记
            ++it;
            continue;
        }
        int dist = getDistByPoint(bIdx, gd.first);
        if (int(Goods_tolerance * dist) + zhenId > gd.second.end_time)  // 取货物容错系数
        { // 当前机器人来不及处理该 good 
            ++it;
            continue;
        }
        if (gd.second.priority == 0.0) {
            gd.second.updatePriority(dist);
        }
        if (gd.second.priority > maxPriority)
        {
            p = gd.first;
            maxPriority = gd.second.priority;
        }
        ++it;
    }
    if (p != boat_virtual_point)
    {
        gds[p].marked = true;
    }
    return p;
}

// 选择泊位
// 选择固定泊位，在初始化BFS后调用
/**
 * @brief 初始化泊位选择。
 *
 * 该函数根据哈夫曼树方法选择泊位。
 * 首先选择两个最小的泊位并将它们合并到一个新的泊位组中。
 * 这个过程一直持续到只剩下五个泊位组为止。 然后，从每组中选择装载速度最大的泊位。
 *
 * @return 包含所选泊位 ID 的vector。
 *
 * @note 全局变量 dists[berth_num][N][N] 记录任一点到泊位 i 的最短距离。
 * @note 全局变量 berths[berth_num + 10] 记录泊位的信息。
 * @note 全局变量 selected_berth[5] 固定泊位seleted_berth_id到泊位id
 */
void InitselectBerth()
{
    vector<int> res;
    // 使用优先队列存储泊位组，队列中的元素是一个pair，第一个元素是泊位组的最小距离，第二个元素是泊位组中的泊位id
    priority_queue<pair<int, vector<int>>, vector<pair<int, vector<int>>>, greater<pair<int, vector<int>>>> pq;
    for (int i = 0; i < berth_num; i++)
    { // 初始化，每个泊位都是一个泊位组
        pq.push({getDistByBerth(i, berths[i]), {i}});
    }
    while (pq.size() > 5)
    { // 合并泊位组，直到只剩下五个泊位组
        // 取出距离最小的两个泊位组
        auto group1 = pq.top();
        pq.pop();
        auto group2 = pq.top();
        pq.pop();
        // 合并两个泊位组
        group1.second.insert(group1.second.end(), group2.second.begin(), group2.second.end());
        // 计算新的泊位组的最小距离
        int min_dist = MAX_LIMIT;
        for (int i : group1.second)
        {
            for (int j : group1.second)
            {
                if (i != j)
                {
                    min_dist = min(min_dist, getDistByBerth(j, berths[i]));
                }
            }
        }
        // 将新的泊位组放入队列
        pq.push({min_dist, group1.second});
    }
    // 选择每个泊位组的代表泊位
    while (!pq.empty())
    {
        auto group = pq.top();
        pq.pop();
        int min_time = INT_MAX;
        int best_berth = -1;
        for (int i : group.second)
        {
            // logger.log(INFO, formatString("berths[i].transport_time:{}", berths[i].transport_time));
            if (berths[i].transport_time < min_time)
            {
                min_time = berths[i].transport_time;
                best_berth = i;
            }
        }
        // TODO 选择最中间的代表泊位或第一帧修改
        // 将代表泊位的id加入结果
        res.push_back(best_berth);
    }
    // 初始化全局变量 selected_berth 固定泊位数组
    for (int i = 0; i < 5; i++)
    {
        selected_berth[i] = res[i];
    }
    // 初始化地图点位所属泊位区域
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            berth_field[i][j] = locateBelongBerth(make_pair(i, j));
        }
    }
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

    return;
}

/**
 * @brief 根据已有的广度优先搜索以找到机器人移动到选定泊位区域的路径。
 *
 * @param robotsIdx 机器人的索引。
 * @param selected_berthIdx 所选泊位的索引。
 * @param max_path 要考虑的最大路径数。
 * @return 无返回值。
 *
 * @note 全局变量 dists[berth_num][N][N] 记录任一点到泊位 i 的最短距离。
 * @note 机器人进入区域后会自动选择该区域的货物目标。
 */
void BFSPathSearch(int robotIdx, int selected_berthIdx, int max_path)
{ // 机器人前往区域路径，进入区域后，机器人会自动选择货物目标
    logger.log(INFO, formatString("robotIdx:{}, selected_berthIdx[{}]:{}, max_path:{}", robotIdx, selected_berthIdx,selected_berth[selected_berthIdx], max_path));
    Robot &robot = robots[robotIdx];
    Point pRobut = make_pair(robot.x, robot.y); // 模拟机器人位置
    vector<Direct> paths;
    vector<int> nums = {0, 1, 2, 3};
    // 随机数生成器
    std::random_device rd;
    std::mt19937 g(rd());
    // 打乱数组顺序
    std::shuffle(nums.begin(), nums.end(), g);
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
                    // logger.log(INFO, formatString("find paths size:{}", paths.size()));
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
                    robot_index++; // TODO 注意数组越界，复赛阶段可以购买机器人
                }
            }
        }
    }

    // 计算每个泊位到机器人的最大距离,距离最大的泊位先分配机器人（控制总移动次数）
    pair<int, int> max_dist[select_berth_num]; // 固定泊位id和每个固定泊位到机器人的最大距离
    for (int select_berth_id = 0; select_berth_id < select_berth_num; select_berth_id++)
    { // 首轮分配，每个泊位按最大距离由大到小依次分配一个机器人
        max_dist[select_berth_id] = make_pair(select_berth_id, 0);
        for (int robot_id = 0; robot_id < robot_num; robot_id++)
        { // 每个泊位到所有机器人的最大距离 //TODO 可以极差或标准差衡量
            if (getDistByRobot(selected_berth[select_berth_id], robots[robot_id]) == MAX_LIMIT)
            { // 机器人和泊位位置不可达
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
    // 剩余机器人的处理：每个泊位按照运输时间由小到大依次分配机器人
    pair<int, int> berth_trasnport_time[select_berth_num];
    for (int i = 0; i < select_berth_num; i++)
    {
        berth_trasnport_time[i] = make_pair(i, berths[selected_berth[i]].transport_time);
    }
    sort(berth_trasnport_time, berth_trasnport_time + select_berth_num, [](pair<int, int> a, pair<int, int> b)
         { return a.second < b.second; }); // 排序：按照运输时间由小到大
    for (int selected_berth_id = 0; selected_berth_id < select_berth_num; selected_berth_id++)
    {
        int min_dist = MAX_LIMIT;
        int best_robot_id = -1;
        for (int robot_id = 0; robot_id < robot_num; robot_id++)
        {
            if (getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[robot_id]) < min_dist && robots[robot_id].path.size() == 0)
            {
                min_dist = getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[robot_id]);
                best_robot_id = robot_id;
            }
        }
        // 设置选定机器人前往对应泊位区域的路径
        BFSPathSearch(best_robot_id, berth_trasnport_time[selected_berth_id].first, getDistByRobot(selected_berth[berth_trasnport_time[selected_berth_id].first], robots[best_robot_id])); // 设置机器人初始路径
    }
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
