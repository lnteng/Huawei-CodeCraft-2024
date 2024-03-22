#include <ctime>
#include <iterator>
#include <cstdlib>
#include "move.hpp"
#include "graph.hpp"
#include "optimizer.hpp"
#include "collisionDetection.hpp"

void Init()
{
    for (int i = 0; i < n; i++)
    { // 从 ch[0][0] -> ch[n-1][n-1]
        scanf("%s", ch[i]);
    }
    for (int i = 0; i < berth_num; i++)
    {
        int berth_id; //传入船舶ID，对齐用
        scanf("%d", &berth_id);
        scanf("%d%d%d%d", &berths[berth_id].x, &berths[berth_id].y, &berths[berth_id].transport_time, &berths[berth_id].loading_speed);
        logger.log(INFO, formatString("berth {}: ({},{}),tTime: {},LoadSpeed: {}",berth_id, berths[berth_id].x, berths[berth_id].y, berths[berth_id].transport_time, berths[berth_id].loading_speed));
    }
    for (int i = 0; i < robot_num; i++) {
        robots[i].robotId = i;
    }
    scanf("%d", &boat_capacity);
    logger.log(INFO, formatString("boat_capacity: {}", boat_capacity));
    char okk[100];
    scanf("%s", okk);
    getDistByBfs();    // 使用基于stack实现的广度优先搜索 (BFS) 算法计算每个泊位到网格上每个点的距离
    InitselectBerth(); // 设置泊位初始位置，确定固定泊位和初始化地图点位所属泊位区域，并初始化每个点的拥堵度
    InitRobot();       // 初始化机器人路径，实现固定泊位区域分配
    Ok();
    logger.log(INFO, "Init OK");
    fflush(stdout);
}

/**
 * 将机器人沿给定方向移动到指定索引处。
 *
 * @param idx 要移动的机器人的索引。
 * @param dir 机器人移动的方向。
 */
inline void robotMove(int idx, Direct dir)
{
    robots[idx].x += dx[dir];
    robots[idx].y += dy[dir];
    switch (dir)
    {
    case Direct::right:
        robotRight(idx);
        break;
    case Direct::left:
        robotLeft(idx);
        break;
    case Direct::down:
        robotDown(idx);
        break;
    case Direct::upper:
        robotUpper(idx);
        break;
    default:
        break;
    }
}

int Input()
{
    scanf("%d%d", &id, &money); // 帧序号 当前金钱数
    int num;                    // 新增货物数
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {                  // 新增货物
        int x, y, val; // 货物坐标 货物价值
        scanf("%d%d%d", &x, &y, &val);
        Point point = make_pair(x, y);
        gds[point] = GoodsProperty(val, id); // TODO：计算优先级
    }
    for (int i = 0; i < robot_num; i++)
    { // 机器人状态
        int x, y;
        scanf("%d%d%d%d", &robots[i].goods, &x, &y, &robots[i].status);
        if (x != robots[i].x || y != robots[i].y || robots[i].status == 0) {
            robots[i].x = x;
            robots[i].y = y;
            logger.log(WARNING, formatString("{} :robot {} failed to move, robot collision in ({}, {})", id, i, robots[i].x, robots[i].y));
            if (robots[i].pid > 0 && robots[i].pid + 1 < robots[i].path.size()) {
                for (int idx = robots[i].pid - 1; idx <= robots[i].pid + 1; idx++) {
                    logger.log(to_string(robots[i].path[idx]));
                }
            }
            if (!robots[i].hasPath()) {
                logger.log(formatString("has no path {}", robots[i].path[robots[i].pid - 1]));
            }
            // // robots[i].rollBack();
        }
    }
    for (int i = 0; i < 5; i++)
    { // 船状态
        scanf("%d%d\n", &boats[i].status, &boats[i].pos);
    }
    char okk[100];
    scanf("%s", okk);
    return id; // 返回帧序号
}

void Output(int zhenId)
{
    // logger.log(INFO, formatString("zhenId: {}", zhenId));
    // TODO ：差错检测
    // TODO : 机器人碰撞处理
    vector<int> sortedRobots = collisionAvoid(zhenId);
    for (auto robotIdx: sortedRobots)
    {
        // 
        
        Robot &robot = robots[robotIdx];
        Point pRobut = make_pair(robot.x, robot.y);
        if (robot.status == 0)
        { // 碰撞后的恢复状态
            robot_recover_count++;
            continue;
        }
        if (robot.goods == 0)
        { // 未携带货物
            if (!robot.hasPath())
            { // 机器人没有路径或路径已走完
                if (gds.count(pRobut) > 0 && gds[pRobut].end_time > zhenId) // TODO 测试等于时是否会消失
                { // 当前位置有货物且货物没有消失
                    robotGet(robotIdx); // TODO 差错检测：如果货物已经消失，会影响货物数量和价值统计
                    robot.goodValue = gds[pRobut].value;
                    logger.log(INFO, formatString("{}: get {},{}", zhenId, robot.x, robot.y));
                    gds.erase(pRobut);
                
                    int berthIdx = selected_berth[berth_field[robot.x][robot.y]];
                    vector<Direct> paths = bfsPaths(pRobut, berthIdx); // 拿到 good 的同时，规划好回到目标 berth 的路线
                    robot.newPath(paths);
                    robotMove(robotIdx, robot.path[robot.pid]);
                    robot.incrementPid();
                }
                else
                { // 机器人当前位置没有货物, 则选择本区域优先度最高的一个货物, 并使用A*算法计算最短路径
                    int berthIdx = selected_berth[berth_field[robot.x][robot.y]];
                    Point pGood = pickGood(robot.robotId, zhenId);
                    if (gds.empty()||pGood == boat_virtual_point)
                    {
                        logger.log(INFO, formatString("{} :without gds,gds is empty, robot: {} ", zhenId, robotIdx));
                        without_goods_time_count[berthIdx]++; // 统计泊位区域无货物的帧数*机器人数
                        continue;
                    }
                    vector<Direct> paths = AStar(pRobut, pGood,1); // 计算最短路径
                    logger.log(INFO,formatString("{} :choose goods,robot {},{} ->pickGood: {},{}:{}", zhenId, robot.x, robot.y, pGood.first, pGood.second, paths.size()));
                    robot.newPath(paths);
                    continue;
                }
            }
            else
            {
                // 根据计算出来的最短路径移动
                robotMove(robotIdx, robot.path[robot.pid]);
                robot.incrementPid();
            }
        }
        else
        { // 携带有货物，根据最短距离数组判断返回港口的下一步
            if (!robot.hasPath())
            {
                int berthIdx = selected_berth[berth_field[robot.x][robot.y]]; // 根据区域选择泊位最近货物
                if (getDistByRobot(berthIdx, robot) != 0)
                { // TODO 路径走完却没有到达 berth 异常处理
                    logger.log(WARNING, formatString("{} :paths has been traveled but has not reach berth {}", zhenId, berthIdx));
                    vector<Direct> paths = bfsPaths(pRobut, berthIdx); // 重新规划好回到目标 berth 的路线
                    robot.newPath(paths);
                } 
                else 
                {
                    Berth &berth = berths[berthIdx];
                    robotPull(robotIdx);
                    berth.remain_goods_num += 1;
                    berth.remain_goods_value.push(robot.goodValue);
                    logger.log(INFO, formatString("{}: pull {},{}", zhenId, robot.x, robot.y));

                    Point pGood = pickGood(robot.robotId, zhenId); // 放下 good 的同时，选择一个新的 good
                    if (gds.empty()||pGood == boat_virtual_point)
                    {
                        logger.log(INFO, formatString("{} :after pulling,gds is empty, robot: {} ", zhenId, robotIdx));
                        without_goods_time_count[berthIdx]++; // 统计泊位区域无货物的帧数*机器人数
                        continue;
                    }
                    vector<Direct> paths = AStar(make_pair(robot.x, robot.y), pGood,1);
                    logger.log(formatString("{} :robot {},{} ->pickGood: {},{}:{}", zhenId, robot.x, robot.y, pGood.first, pGood.second, paths.size()));
                    robot.newPath(paths);
                }
            }
            else
            {
                robotMove(robotIdx, robot.path[robot.pid]);
                robot.incrementPid();
            }
        }
    }

    // 处理船舶
    for (int i = 0; i < boat_num; i++)
    {
        if (boats[i].status == 0)
        { // 船舶状态为 0,运输中
            boats[i].num = 0;
            continue;
        }
        if (boats[i].status == 2)
        { // TODO：船舶状态为 2，添加判定是否已有船舶在泊位上
            logger.log(INFO, formatString("berth {} :boats[{}].status: 1", boats[i].pos, i));
            for (int boat_id = 0; boat_id < boat_num; boat_id++)
            { // TODO 是否可以由一个泊位直接跑到另一个泊位„
                if (i!= boat_id && boats[i].pos == boats[boat_id].pos && boats[boat_id].status == 1)
                { // 仍然由船舶停靠在岸
                    logger.log(INFO, formatString("berth {} :boats{} waiting", boats[i].pos, i));
                    continue;
                }
            }
            logger.log(INFO, formatString("berth {} :boats{} reach", boats[i].pos, i));
            boatShip(i, boats[i].pos);
            continue;
        }
        if (boats[i].pos == -1)
        { // 在虚拟点，前往泊位
            int berth_id = shipBackBerth(i);
            logger.log(INFO, "boatShip " + to_string(i) + " to " + to_string(berth_id));
            boatShip(i, berth_id);
            continue;
        }
        Berth &berth = berths[boats[i].pos];
        // 统计在泊位停靠的时间
        boat_berthing_time_count[boats[i].pos]++;
        // 注意避免重复指令导致刷新运送时间
        bool end_flag = (berth.transport_time + zhenId + Fault_tolerance > zhen_total);
        if (berth.remain_goods_num >= berth.loading_speed && !end_flag && boats[i].num + berth.loading_speed <= boat_capacity)
        {
            // 船舶装载货物空间充足且泊位剩余货物充足且未临近结束时间
            berth.remain_goods_num -= berth.loading_speed;
            berth.popRemainGoods(berth.loading_speed);
            boats[i].num += berth.loading_speed;
            if (boats[i].num == boat_capacity)
            { // 船舶装载满
                logger.log(INFO, formatString("{}:full boat {} boatGo,berth {} remain: {}", zhenId,i,boats[i].pos,berth.remain_goods_num));
                boatGo(i);
                // TODO 重新选择固定船舶位置
                // bool flag = updateSelectedBerth(berthBelongGroup[boats[i].pos], zhenId);
                // if (flag)
                // {
                //     // 修改本区域机器人目标港口，删除本区域机器人此步后（碰撞避免）以后的路径
                //     for (int j = 0; j < robot_num; j++)
                //     {
                //         if (berth_field[robots[j].x][robots[j].y] == berthBelongGroup[boats[i].pos])
                //         {
                //             if (robots[j].goods) { // 有货物的重新计算路径去新港口
                //                 vector<Direct> paths = {};
                //                 robots[j].newPath(paths);
                //             }
                //             // 机器人绑定固定船舶ID不用更新
                //         }
                //     }
                //     // TODO 更新固定区域
                // }
            }
        }
        else
        {
            if ((berth.remain_goods_num > 0 || boats[i].num + berth.loading_speed > boat_capacity) && !end_flag)
            {
                if (berth.remain_goods_num > boat_capacity - boats[i].num)
                { // 船舶装载满
                    berth.remain_goods_num -= boat_capacity - boats[i].num;
                    berth.popRemainGoods(boat_capacity - boats[i].num);
                    boats[i].num = boat_capacity;
                    logger.log(INFO, formatString("{}:full boat {} boatGo,berth {} remain: {}", zhenId,i,boats[i].pos,berth.remain_goods_num));
                    boatGo(i);
            
                    bool flag = updateSelectedBerth(berthBelongGroup[boats[i].pos], zhenId);

                }
                else
                { // 港口货物装载空
                    boats[i].num += berth.remain_goods_num;
                    berth.remain_goods_num = 0;
                    berth.popRemainGoods(berth.remain_goods_value.size());
                }
            }
            else
            { // 船舶装载完毕或船舶剩余货物未空或临近结束时间
                if (boats[i].pos != -1 && (end_flag || boats[i].num == boat_capacity))
                { // 船舶装载完毕或船舶剩余货物未空
                    logger.log(INFO, formatString("{}:deadline boat {} boatGo,berth {} remain: {}", zhenId,i,boats[i].pos,berth.remain_goods_num));
                    boatGo(i);
                    // TODO 单独的策略(如果有可达的港口)，应该从其他泊位组选择
                    endBoatGroup[berthBelongGroup[boats[i].pos]] = 0;
                    bool flag = endSelectedBerth(berthBelongGroup[boats[i].pos], zhenId);
                    if (flag) //如果存在可达的剩余港口
                    {
                         for (int j = 0; j < robot_num; j++)
                        {
                            if (berth_field[robots[j].x][robots[j].y] == berthBelongGroup[boats[i].pos])
                            {
                                if (robots[j].goods) { // 有货物的重新计算路径去新港口
                                    vector<Direct> paths = {};
                                    robots[j].newPath(paths);
                                }
                                // 机器人绑定固定船舶ID不用更新
                            }
                        }
                    }// 否则已经没有可达的港口了
                }
            }
        }
    }
}

int main()
{
    Init();
    // printMoreDebugINfo(4);
    for (int zhen = 1; zhen <= zhen_total; zhen++)
    {
        int zhenId = Input();
        if (zhen == 1 and id > 1)
        {
            logger.log(ERROR, "初始化超时: " + to_string(id - zhen) + " 帧 ");
        }
        Output(zhenId);
        Ok();
        fflush(stdout);
        if (zhenId == zhen_total)
        {
            summary(zhen,zhenId); // 最后一帧结算信息
            zhen = zhenId; //提前结束
            std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待3秒
        }  
    }
    return 0;
}
