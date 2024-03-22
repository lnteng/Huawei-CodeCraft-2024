#pragma once
#include "const.h"
#include "graph.hpp"
/*
    实现碰撞检测和碰撞避免
    改进1：后续让携带货物的机器人先走，无货物的机器人停止，优先级：携带高价值货物的机器人>携带低价值货物的机器人>不携带货物的机器人
    改进2：考虑机器人性价比，携带货物价值/机器人到目的地距离（无论是到货品的距离还是到船舶的距离），越大，机器人优先级越高
*/
// 机器人性价比
double priority_robot(Robot robotx)
{
    logger.log("priority_robot");
    double priority = 9.9;
    if ((robotx.path.size() - robotx.pid) == 0)
    {
        return 0;
    }
    priority = robotx.goods / (robotx.path.size() - robotx.pid);
    return priority;
}

/**
  * @brief 检测给定机器人ID移动序列是否会发生碰撞
  *
  * @param robots_order 机器人移动顺序
  * @return 存储发生碰撞的机器人ID的vector
  * 
  * @note //TODO 需要测试被碰撞的机器人的下一步位置是否在之后的顺序中空出;目前问题：顺序，可能是机器人同时移动？
  */
std::vector<int> checkCollisions(std::vector<int>& robots_order,int zhenId) {
    std::map<Point, int> positionsBefore;  // 记录移动前的位置
    std::map<Point, int> positionsAfter;  // 记录移动后的位置
    std::vector<int> collisionRobots;  // 记录发生碰撞的机器人ID

    // 记录所有机器人移动前的位置
    for (const auto& rIdx : robots_order) {
        positionsBefore[make_pair(robots[rIdx].x,robots[rIdx].y)] = robots[rIdx].robotId;
    }
    // 记录所有机器人移动后的预测位置
    for (const auto& rIdx : robots_order)
    {   
        int nextX = robots[rIdx].x + dx[robots[rIdx].nextDirect()];
        int nextY = robots[rIdx].y + dy[robots[rIdx].nextDirect()];
        if (robots[rIdx].status == 0 || robots[rIdx].nextDirect() == Direct::pause){
            // 检测原位置是否已经存在，是否发生碰撞
            if (positionsAfter.find(make_pair(robots[rIdx].x, robots[rIdx].y)) != positionsAfter.end())
            { //发生碰撞
                int rIdx2= positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)];
                collisionRobots.push_back(robots[rIdx].robotId);
                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
                // 处理机器人2，退回原位置
                collisionRobots.push_back(robots[rIdx2].robotId);
                positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId;

                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
            } else {
                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
            }
            continue;
        }
        // 检测positionsAfter[make_pair(nextX, nextY)]是否已经存在
        if (positionsAfter.find(make_pair(nextX, nextY)) != positionsAfter.end())
        {
            // 发生碰撞，如果被碰撞的机器人没有移动，设置下一步为所在机器人为-1，表示已经存在机器人在该店碰撞
            int rIdx2 = positionsAfter[make_pair(nextX, nextY)];
            collisionRobots.push_back(robots[rIdx].robotId);
            if (rIdx2 != -1) {
                // 处理robot2
                if (robots[rIdx2].hasPath()&&robots[rIdx2].nextDirect() != Direct::pause&&robots[rIdx2].status !=0)
                { // 如果发生了移动则复位
                    positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId; // 实际位置应该未移动时坐标
                    positionsAfter[make_pair(nextX, nextY)] = -1; //碰撞点也不可前往
                }
                collisionRobots.push_back(robots[rIdx2].robotId); 
            } else {
                collisionRobots.push_back(-1); 
            }
            // 处理robot1
            positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId; // 实际位置应该未移动时坐标
        } else { 
            //处理交换位置
            if (positionsBefore.find(make_pair(nextX, nextY)) != positionsBefore.end())
            {
                int rIdx2 = positionsBefore[make_pair(nextX, nextY)] ;
                int nextX2 = robots[rIdx2].x + dx[robots[rIdx2].nextDirect()];
                int nextY2 = robots[rIdx2].y + dy[robots[rIdx2].nextDirect()];
                if (positionsBefore.find(make_pair(nextX2,nextY2)) != positionsBefore.end())
                { // 发生相向碰撞
                    // 处理robot
                    collisionRobots.push_back(robots[rIdx].robotId);
                    positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
                    // 处理robot2
                    collisionRobots.push_back(robots[rIdx2].robotId);
                    positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId;
                    continue;
                } else {
                    positionsAfter[make_pair(nextX, nextY)] = robots[rIdx].robotId;
                }
            } else {
                positionsAfter[make_pair(nextX, nextY)] = robots[rIdx].robotId;
            }
        }
    }

    // if (collisionRobots.size()>0) {
    //     for (int i = 0; i < robots_order.size(); i++)
    //     {
    //         logger.log("  robots_order:"+to_string(robots_order[i]));
    //     }
    // }
    // // 去重
    // std::sort(collisionRobots.begin(), collisionRobots.end());
    // collisionRobots.erase(std::unique(collisionRobots.begin(), collisionRobots.end()), collisionRobots.end());
    // if (collisionRobots.size()>0) {
    //     for (int i = 0; i < robots_order.size(); i++)
    //     {
    //         logger.log("  robots_order_res:"+to_string(robots_order[i]));
    //     }
    // }
    return collisionRobots;
}




vector<int> topologicalSort(const vector<pair<int, int>> &priority_order, int num_nodes)
{
    // 构建邻接表和入度数组
    vector<int> in_degree(num_nodes, 0);
    vector<vector<int>> adj_list(num_nodes);
    for (const auto &edge : priority_order)
    {
        int from = edge.first;
        int to = edge.second;
        adj_list[from].push_back(to);
        in_degree[to]++;
    }

    // 使用队列进行拓扑排序
    queue<int> q;
    for (int i = 0; i < num_nodes; ++i)
    {
        if (in_degree[i] == 0)
        {
            q.push(i);
        }
    }

    vector<int> sorted_order;
    while (!q.empty())
    {
        int node = q.front();
        q.pop();
        sorted_order.push_back(node);
        for (int neighbor : adj_list[node])
        {
            if (--in_degree[neighbor] == 0)
            {
                q.push(neighbor);
            }
        }
    }

    // 检查是否有环
    if (sorted_order.size() != num_nodes)
    {
        sorted_order.clear(); // 清空排序结果
    }

    return sorted_order;
}

inline Direct baseDirect(Direct dir1, int dir2)
{
    if (dir2 == 0)
    {
        return Direct(leftside[int(dir1)]);
    }
    else
    {
        return Direct(rightside[int(dir1)]);
    }
}

// 通过遍历所有机器人检测与robot1可能相撞的所有情况，为避免碰撞，返回一个新的机器人排序数组sortedRobots[10]
vector<int> collisionAvoid(int zhenId)
{
    // vector<int> unsortedRobots;
    // for (int i = 0; i < robot_num; i++)
    // {
    //     unsortedRobots.push_back(i);
    // }
    // if (checkCollisions(unsortedRobots).size() == 0)
    // {
    //     return unsortedRobots;
    // } 
    vector<pair<int, int>> priorityOrder;
    for (int robotIdx1 = 0; robotIdx1 < robot_num; robotIdx1++)
    {
        Robot &robot1 = robots[robotIdx1];
        for (int robotIdx2 = robotIdx1 + 1; robotIdx2 < robot_num; robotIdx2++)
        {
            Robot &robot2 = robots[robotIdx2];
            if (!robot1.hasPath())
            {
                if (calcManhattanDist(robot1.x, robot1.y, robot2.x, robot2.y) < 2)
                {
                    if (robot2.nextDirect() == Direct::pause)
                    {
                        continue;
                    }
                    logger.log("robot2 insert pause");
                    robot2.insertDirect(Direct::pause);
                    continue;
                }
                continue;
            }
            if (!robot2.hasPath())
            {
                if (calcManhattanDist(robot1.x, robot1.y, robot2.x, robot2.y) < 2)
                {
                    if (robot1.nextDirect() == Direct::pause)
                    {
                        continue;
                    }
                    logger.log("robot1 insert pause");
                    robot1.insertDirect(Direct::pause);
                    continue;
                }
                continue;
            }
            if (robot1.x + dx[robot1.nextDirect()] == robot2.x + dx[robot2.nextDirect()] && robot1.y + dy[robot1.nextDirect()] == robot2.y + dy[robot2.nextDirect()])
            { // case 1: 争抢同一个地方，只需要其中之一暂停一步，就会变成 case 2
                logger.log("case 1");
                if (priority_robot(robot1) > priority_robot(robot2))
                {
                    robot2.insertDirect(Direct::pause);
                    for (int idx = robot2.pid - 1; idx <= robot2.pid + 1; idx++)
                    {
                        logger.log(to_string(robot2.path[idx]));
                    }
                }
                else
                {
                    robot1.insertDirect(Direct::pause);
                    for (int idx = robot1.pid - 1; idx <= robot1.pid + 1; idx++)
                    {
                        logger.log(to_string(robot1.path[idx]));
                    }
                }
            }
            else if (robot1.x + dx[robot1.nextDirect()] == robot2.x && robot1.y + dy[robot1.nextDirect()] == robot2.y)
            { // case 2: 相邻，默认 robot1 先移动，走到 robot2 当前位置的情况
                logger.log("case 2");
                if (robot1.x == robot2.x + dx[robot2.nextDirect()] && robot1.y == robot2.y + dy[robot2.nextDirect()])
                { // robot2 也想往 robot1 的当前位置移动，也就是相向运动，只能选择一个绕路
                    if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]) &&
                        isRobotAccessible(robot2.x + dx[baseDirect(robot1.nextDirect(), 0)], robot2.y + dy[baseDirect(robot1.nextDirect(), 0)])) // TODO
                    {                                                                                                                            // robot1 行走方向的左侧
                        if (priority_robot(robot1) > priority_robot(robot2))
                        { // robot2 绕路
                            robot2.insertDirectAfter(baseDirect(robot1.nextDirect(), 1));
                            robot2.insertDirect(baseDirect(robot1.nextDirect(), 0));
                        }
                        else
                        { // robot1 绕路
                            robot1.insertDirectAfter(baseDirect(robot1.nextDirect(), 1));
                            robot1.insertDirect(baseDirect(robot1.nextDirect(), 0));
                        }
                    }
                    else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]) &&
                             isRobotAccessible(robot2.x + dx[baseDirect(robot1.nextDirect(), 1)], robot2.y + dy[baseDirect(robot1.nextDirect(), 1)])) // TODO
                    {                                                                                                                                 // robot1 行走方向的右侧
                        if (priority_robot(robot1) > priority_robot(robot2))
                        { // robot2 绕路
                            robot2.insertDirectAfter(baseDirect(robot1.nextDirect(), 0));
                            robot2.insertDirect(baseDirect(robot1.nextDirect(), 1));
                        }
                        else
                        { // robot1 绕路
                            robot1.insertDirectAfter(baseDirect(robot1.nextDirect(), 0));
                            robot1.insertDirect(baseDirect(robot1.nextDirect(), 1));
                        }
                    }
                    else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]))
                    { // 只能让 robot1 左绕路
                        robot1.insertDirect(baseDirect(robot1.nextDirect(), 1));
                        robot1.insertDirect(baseDirect(robot1.nextDirect(), 0));
                    }
                    else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]))
                    { // 只能让 robot1 右绕路
                        robot1.insertDirect(baseDirect(robot1.nextDirect(), 0));
                        robot1.insertDirect(baseDirect(robot1.nextDirect(), 1));
                    }
                    else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 0)], robot2.y + dy[baseDirect(robot2.nextDirect(), 0)]))
                    { // 只能让 robot2 左绕路 TODO 修改
                        robot2.insertDirect(baseDirect(robot1.nextDirect(), 1));
                        robot2.insertDirect(baseDirect(robot1.nextDirect(), 0));
                    }
                    else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 1)], robot2.y + dy[baseDirect(robot2.nextDirect(), 1)]))
                    { // 只能让 robot2 右绕路
                        robot2.insertDirect(baseDirect(robot1.nextDirect(), 0));
                        robot2.insertDirect(baseDirect(robot1.nextDirect(), 1));
                    }
                    else
                    { // TODO: 无路可走的情况，应该只剩下单通道
                    }
                }
                else
                { // robot2 不会向 robot1 当前位置移动，只需要让 robot2 先走，给 robot1 让路
                    priorityOrder.push_back(make_pair(robotIdx2, robotIdx1));
                }
            }
            else if (robot1.x == robot2.x + dx[robot2.nextDirect()] && robot1.y == robot2.y + dy[robot2.nextDirect()])
            { // case 3: 相邻，robot1先移动不碰撞，robot2 先移动，走到 robot1 当前位置的情况
                logger.log("case 3");
                priorityOrder.push_back(make_pair(robotIdx1, robotIdx2));
            }
        }
    }
    vector<int> sortedRobots;
    if (!priorityOrder.empty())
    {
        sortedRobots = topologicalSort(priorityOrder, robot_num);
    }
    else
    {
        for (int i = 0; i < robot_num; i++)
        {
            sortedRobots.push_back(i);
        }
    }
    if (sortedRobots.empty())
    {
        logger.log(WARNING,"collisionAvoid error: priorityOrder is empty"); 
    }
    vector<int> collisionRobots = checkCollisions(sortedRobots,zhenId); //会发生碰撞的机器人 
    // TODO 测试
    if (collisionRobots.size() > 0) 
    {   // TODO 使一部分发生碰撞的机器人暂停一步
        logger.log(WARNING,"collisionAvoid handle");
        // 遍历发生碰撞的顺序
        // std::sort(collisionRobots.begin(), collisionRobots.end());
        vector<int> tempOrder;
        vector<int> bestOrder=collisionRobots;
        // 打印collisionRobots
        for (int i = 0; i < collisionRobots.size(); i++)
        {
            logger.log("collisionRobots include:"+to_string(collisionRobots[i]));
        }
    
        // do
        // {
        //     tempOrder = checkCollisions(collisionRobots);
        //     if (tempOrder.size()==0) {
        //         bestOrder=collisionRobots;
        //         break;
        //     } else if (tempOrder.size()<bestOrder.size()) {
        //         bestOrder=tempOrder;
        //     }
        // } while (std::next_permutation(collisionRobots.begin(), collisionRobots.end()));
        // // 调整sortedRobots中bestOrder对应顺序
        // if (bestOrder.size()!=0){
        //     logger.log(ERROR,"collisionAvoid handle fail");
        // }
    }
    return sortedRobots;
}
