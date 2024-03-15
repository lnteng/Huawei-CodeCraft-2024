#include <ctime>
#include <iterator>
#include <cstdlib>
#include "move.hpp"
#include "graph.hpp"
#include "optimizer.hpp"

void Init()
{
    for (int i = 0; i < n; i++)
    { // 从 ch[0][0] -> ch[n-1][n-1]
        scanf("%s", ch[i]);
    }
    for (int i = 0; i < berth_num; i++)
    {
        int boat_id; //传入船舶ID，对齐用
        scanf("%d", &boat_id);
        scanf("%d%d%d%d", &berths[boat_id].x, &berths[boat_id].y, &berths[boat_id].transport_time, &berths[boat_id].loading_speed);
        logger.log(INFO, formatString("berth {}: ({},{}),tTime: {},LoadTime: {}",boat_id, berths[boat_id].x, berths[boat_id].y, berths[boat_id].transport_time, berths[id].loading_speed));
    }
    scanf("%d", &boat_capacity);
    logger.log(INFO, formatString("boat_capacity: {}", boat_capacity));
    char okk[100];
    scanf("%s", okk);
    getDistByBfs();    // 使用基于stack实现的广度优先搜索 (BFS) 算法计算每个泊位到网格上每个点的距离
    InitselectBerth(); // 设置泊位初始位置，确定固定泊位和初始化地图点位所属泊位区域
    InitRobot();       // 初始化机器人路径，实现固定泊位区域分配
    Ok();
    // logger.log(INFO, "Init OK");
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
        scanf("%d%d%d%d", &robots[i].goods, &robots[i].x, &robots[i].y, &robots[i].status);
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
    // TODO : 机器人碰撞处理
    for (int robotIdx = 0; robotIdx < robot_num; robotIdx++)
    {
        Robot &robot = robots[robotIdx];
        Point pRobut = make_pair(robot.x, robot.y);
        if (robot.status == 0)
        { // 碰撞后的恢复状态
            continue;
        }
        if (robot.goods == 0)
        { // 未携带货物
            if (!robot.hasPath())
            { // 机器人没有路径或路径已走完
                if (gds.count(pRobut) > 0)
                { // 当前位置有货物
                    robotGet(robotIdx);
                    logger.log(INFO, formatString("{}: get {},{}", zhenId, robot.x, robot.y));
                    gds.erase(pRobut);
                }
                else
                { // 当前位置没有货物, 随机选择一个货物, 并使用A*算法计算最短路径
                    if (gds.empty())
                    {
                        logger.log(ERROR, formatString("{} :gds is empty, robot: {} ", zhenId, robotIdx));
                        continue;
                    }
                    // Point pGood;
                    // int randomIndex = std::rand() % gds.size();
                    // auto it = gds.begin();
                    // std::advance(it, randomIndex++);
                    // pGood = it->first;
                    // if (randomIndex >= gds.size() || randomIndex < 0)
                    // {
                    //     continue;
                    // }
                    int berthIdx = selected_berth[berth_field[robot.x][robot.y]];
                    Point pGood = pickGood(berthIdx, zhenId);
                    vector<Direct> paths = AStar(pRobut, pGood); // 计算最短路径
                    logger.log(formatString("if :{} :robot {},{} ->pickGood: {},{}:{}", zhenId, robot.x, robot.y, pGood.first, pGood.second, paths.size()));
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
        {                                                          // 携带有货物，根据最短距离数组判断返回港口的下一步
            int berthIdx = selected_berth[berth_field[robot.x][robot.y]]; // 根据区域选择泊位最近货物
            Berth &berth = berths[berthIdx];
            vector<int> nums = {0, 1, 2, 3};
            std::random_device rd;
            std::mt19937 g(rd());
            // 打乱数组顺序
            std::shuffle(nums.begin(), nums.end(), g);
            for (int dir : nums)
            {
                if (isVaild(robot.x, robot.y, (Direct)dir) && dists[berthIdx][robot.x + dx[dir]][robot.y + dy[dir]] < getDistByRobot(berthIdx, robot))
                {
                    robotMove(robotIdx, (Direct)dir);
                    if (getDistByRobot(berthIdx, robot) == 0)
                    {
                        robotPull(robotIdx);
                        berth.remain_goods_num += 1;
                        logger.log(INFO, formatString("{}: pull {},{}", zhenId, robot.x, robot.y));
                        Point pGood = pickGood(berthIdx, zhenId);
                        vector<Direct> paths = AStar(make_pair(robot.x, robot.y), pGood);
                        logger.log(formatString("{} :robot {},{} ->pickGood: {},{}:{}", zhenId, robot.x, robot.y, pGood.first, pGood.second, paths.size()));
                        robot.newPath(paths);
                    }
                }
            }
        }
    }

    // 处理船舶
    for (int i = 0; i < 5; i++)
    {
        if (boats[i].status == 0)
        { // 船舶状态为 0,运输中
            boats[i].num = 0;
            continue;
        }
        if (boats[i].status == 2)
        { // TODO：船舶状态为 2，添加判定是否已有船舶在泊位上
            logger.log(INFO, formatString("berth {} :boats[{}].status: 1", boats[i].pos, i));
            for (int boat_id = 0; i < boat_num; i++)
            { // TODO 是否可以由一个泊位直接跑到另一个泊位
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
        // 注意避免重复指令导致刷新运送时间
        bool end_flag = (berth.transport_time + zhenId + Fault_tolerance > zhen_total);
        if (berth.remain_goods_num >= berth.loading_speed && !end_flag && boats[i].num + berth.loading_speed <= boat_capacity)
        {
            // 船舶装载货物空间充足且泊位剩余货物充足且未临近结束时间
            berth.remain_goods_num -= berth.loading_speed;
            boats[i].num += berth.loading_speed;
            if (boats[i].num == boat_capacity)
            { // 船舶装载满
                logger.log(INFO, "boatGo " + to_string(i));
                boatGo(i);
            }
        }
        else
        {
            if ((berth.remain_goods_num > 0 || boats[i].num + berth.loading_speed > boat_capacity) && !end_flag)
            {
                if (berth.remain_goods_num > boat_capacity - boats[i].num)
                { // 船舶装载满
                    berth.remain_goods_num = boat_capacity - (boat_capacity - boats[i].num);
                    boats[i].num = boat_capacity;
                    logger.log(INFO, "boatGo " + to_string(i));
                    boatGo(i);
                }
                else
                { // 港口货物装载空
                    boats[i].num += berth.remain_goods_num;
                    berth.remain_goods_num = 0;
                }
            }
            else
            { // 船舶装载完毕或船舶剩余货物未空或临近结束时间
                if (boats->pos != -1 && (end_flag || boats[i].num == boat_capacity))
                { // 船舶装载完毕或船舶剩余货物未空
                    logger.log(INFO, "boatGo " + to_string(i));
                    boatGo(i);
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
