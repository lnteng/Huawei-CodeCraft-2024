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
std::vector<pair<int,int>> checkCollisions(int zhenId) {
    std::map<Point, int> positionsBefore;  // 记录移动前的位置
    std::map<Point, int> positionsAfter;  // 记录移动后的位置
    // std::vector<int> collisionRobots;  // 记录发生碰撞的机器人ID
    std::vector<pair<int,int>> collisionRobots;
    std::map<Point,pair<int,int>> freePoint; //记录碰撞后空出的点位置

    // 记录所有机器人移动前的位置
    for (int rIdx = 0;rIdx < robot_num; rIdx++) {
        positionsBefore[make_pair(robots[rIdx].x,robots[rIdx].y)] = robots[rIdx].robotId;
    }
    // 记录所有机器人移动后的预测位置
    for (int rIdx = 0;rIdx < robot_num; rIdx++)
    {   
        int nextX = robots[rIdx].x + dx[robots[rIdx].nextDirect()];
        int nextY = robots[rIdx].y + dy[robots[rIdx].nextDirect()];
        if (robots[rIdx].status == 0 || robots[rIdx].nextDirect() == Direct::pause){
            // 检测原位置是否已经存在，是否发生碰撞
            if (positionsAfter.find(make_pair(robots[rIdx].x, robots[rIdx].y)) != positionsAfter.end())
            { //发生碰撞
                int rIdx2= positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)];
                if (rIdx2 == -1) {
                    collisionRobots.push_back(make_pair(robots[rIdx].robotId,-1)); 
                    continue;
                }
                // collisionRobots.push_back(robots[rIdx].robotId);
                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
                // 处理机器人2，退回原位置
                // collisionRobots.push_back(robots[rIdx2].robotId);
                collisionRobots.push_back(make_pair(robots[rIdx].robotId,robots[rIdx2].robotId));
                positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId;

                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
            } else {
                logger.log(formatString("robot{} pause {},{}",rIdx,robots[rIdx].x,robots[rIdx].y));
                positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
            }
            continue;
        }
        // 检测positionsAfter[make_pair(nextX, nextY)]是否已经存在
        if (positionsAfter.find(make_pair(nextX, nextY)) != positionsAfter.end())
        {
            // 发生碰撞，如果被碰撞的机器人没有移动，设置下一步为所在机器人为-1，表示已经存在机器人在该店碰撞
            int rIdx2 = positionsAfter[make_pair(nextX, nextY)];
            logger.log("下一步碰撞");
            // collisionRobots.push_back(robots[rIdx].robotId);
            if (rIdx2 != -1) {
                // 处理robot2
                if (robots[rIdx2].hasPath()&&robots[rIdx2].nextDirect() != Direct::pause&&robots[rIdx2].status !=0)
                { // 如果发生了移动则复位
                    positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId; // 实际位置应该未移动时坐标
                    positionsAfter[make_pair(nextX, nextY)] = -1; //两个机器人退回原位，碰撞点也不可前往
                    freePoint[make_pair(robots[rIdx].x,robots[rIdx].y)] = make_pair(robots[rIdx].robotId,rIdx2);
                }
                // collisionRobots.push_back(robots[rIdx2].robotId); 
                collisionRobots.push_back(make_pair(robots[rIdx].robotId,robots[rIdx2].robotId));
            } else {
                // collisionRobots.push_back(-1);
                collisionRobots.push_back(make_pair(robots[rIdx].robotId,-1)); 
            }
            // 处理robot1
            positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId; // 实际位置应该未移动时坐标
        } else { 
            //处理交换位置
            if (positionsBefore.find(make_pair(nextX, nextY)) != positionsBefore.end())
            {
                int rIdx2 = positionsBefore[make_pair(nextX, nextY)] ;
                logger.log(formatString("交换位置 robot{} next:{},{}->{}",rIdx,nextX,nextY,rIdx2));
                int nextX2 = robots[rIdx2].x + dx[robots[rIdx2].nextDirect()];
                int nextY2 = robots[rIdx2].y + dy[robots[rIdx2].nextDirect()];
                if (positionsBefore.find(make_pair(nextX2,nextY2)) != positionsBefore.end()
                    && positionsBefore[make_pair(nextX2,nextY2)] == robots[rIdx].robotId
                    )
                { // 发生交换位置
                    
                    // 处理robot
                    // collisionRobots.push_back(robots[rIdx].robotId);
                    positionsAfter[make_pair(robots[rIdx].x, robots[rIdx].y)] = robots[rIdx].robotId;
                    // 处理robot2
                    // collisionRobots.push_back(robots[rIdx2].robotId);
                    collisionRobots.push_back(make_pair(robots[rIdx].robotId,robots[rIdx2].robotId));
                    positionsAfter[make_pair(robots[rIdx2].x, robots[rIdx2].y)] = robots[rIdx2].robotId;
                    continue;
                } else {
                    positionsAfter[make_pair(nextX, nextY)] = robots[rIdx].robotId;
                }
            } else { // 正常情况
                positionsAfter[make_pair(nextX, nextY)] = robots[rIdx].robotId;
            }
        }
    }
    // 将collisionRobots中的-1分别替换为实际的两个机器人ID
    std::vector<pair<int,int>> res;
    for (int i = 0; i < collisionRobots.size(); i++)
    {
        if (collisionRobots[i].second == -1)
        { //将-1替换为原本会发生碰撞的两个机器人
            Point p = make_pair(robots[collisionRobots[i].first].x,robots[collisionRobots[i].first].y);
            res.push_back(make_pair(collisionRobots[i].first,freePoint[p].first));
            res.push_back(make_pair(collisionRobots[i].first,freePoint[p].second));
        } else{
            res.push_back(collisionRobots[i]);
        }
    }

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

pair<int, int> detectCollision() {
    // unordered_map<Point, int, hash_pair> cur_positions;
    // unordered_map<Point, int, hash_pair> next_posisions;
    // for (int i = 0; i < robot_num; i++) {
    //     Robot& robot = robots[i];
    //     Point cur_pos = make_pair(robot.x, robot.y);
    //     cur_positions[cur_pos] = i;
    //     Point next_pos = make_pair(robot.x + dx[robot.nextDirect()], robot.y + dy[robot.nextDirect()]);
    //     if (next_posisions.count(next_pos) > 0) {
    //         return make_pair(i, next_posisions[next_pos]);
    //     }
    //     next_posisions[next_pos] = i;
        
    //     if (cur_positions.count(next_pos) > 0) {
    //         int j = cur_positions[next_pos];
    //         if (next_posisions.count(cur_pos) > 0 && next_posisions[cur_pos] == j) {
    //             return make_pair(i, j);
    //         }
    //     }
    // }
    vector<Point> cur_positions;
    vector<Point> next_posisions;
    for (int i = 0; i < robot_num; i++) {
        Robot& robot = robots[i];
        cur_positions.push_back(make_pair(robot.x, robot.y));
        next_posisions.push_back(make_pair(robot.x + dx[robot.nextDirect()], robot.y + dy[robot.nextDirect()]));
    }
    for (int i = 0; i < robot_num; i++) {
        for (int j = i + 1; j < robot_num; j++) {
            if (next_posisions[i] == next_posisions[j]) {
                return make_pair(i, j);
            }
            if (cur_positions[i] == next_posisions[j] && next_posisions[i] == cur_positions[j]) {
                return make_pair(i, j);
            }
        }
    }
    return boat_virtual_point;
}

void collisionAvoid() {
    for (int i = 0; i < robot_num; i++) {
        Robot& robot = robots[i];
        if (congestion[robot.x][robot.y].first != 2) {
            robot.rollback = 0;
        }
    }
    auto collision = detectCollision();
    int i = 0;
    pair<int,int> temp;
    int while_limit = 100;
    while (collision != boat_virtual_point) {
        while_limit--;
        Robot& robot1 = robots[collision.first];
        Robot& robot2 = robots[collision.second];
        if (calcManhattanDist(robot1.x, robot1.y, robot2.x, robot2.y) == 1) {
            if (congestion[robot1.x][robot1.y].first == 2 && congestion[robot2.x][robot2.y].first == 2) {
                // if (robot1.rollback > robot2.rollback) {
                if (robot1.robotId > robot2.robotId) {
                    for (int dir = 0; dir < 4; dir++) {
                        if (isRobotAccessible(robot1.x + dx[dir], robot1.y + dy[dir]) && (robot1.x + dx[dir] != robot2.x || robot1.y + dy[dir] != robot2.y)) {
                            // logger.log("oneway");
                            // logger.log(formatString("{} {}", robot1.x, robot1.y));
                            // logger.log(formatString("{} {}", robot2.x, robot2.y));
                            // logger.log(formatString("{} {}", robot1.nextDirect(), robot2.nextDirect()));
                            robot1.insertDirect(Direct(dir ^ 1));
                            robot1.insertDirect(Direct(dir));
                            ++robot1.rollback;
                            // logger.log(formatString("{} {}", robot1.nextDirect(), robot1.path[robot1.pid+1]));
                        }
                    }
                }
                else
                {
                    for (int dir = 0; dir < 4; dir++) {
                        if (isRobotAccessible(robot2.x + dx[dir], robot2.y + dy[dir]) && (robot2.x + dx[dir] != robot1.x || robot2.y + dy[dir] != robot1.y)) {
                            // logger.log("oneway");
                            // logger.log(formatString("{} {}", robot2.x, robot2.y));
                            // logger.log(formatString("{} {}", robot2.x, robot2.y));
                            // logger.log(formatString("{} {}", robot1.nextDirect(), robot2.nextDirect()));
                            robot2.insertDirect(Direct(dir ^ 1));
                            robot2.insertDirect(Direct(dir));
                            ++robot2.rollback;
                            // logger.log(formatString("{} {}", robot2.nextDirect(), robot2.path[robot2.pid+1]));
                        }
                    }
                }
            } 
            else if (congestion[robot1.x][robot1.y].first == 2) {
                if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 0)], robot2.y + dy[baseDirect(robot2.nextDirect(), 0)]) && robot2.nextDirect() != Direct::pause && (robot2.x + dx[baseDirect(robot2.nextDirect(), 1)] != robot2.mbx || robot2.y + dy[baseDirect(robot2.nextDirect(), 1)] != robot2.mby))
                { // 只能让 robot2 左绕路
                    Direct next = robot2.nextDirect();
                    robot2.insertDirect(baseDirect(next, 1));
                    robot2.insertDirect(baseDirect(next, 0));
                    // logger.log("只能让 robot2 左绕路");
                }
                else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 1)], robot2.y + dy[baseDirect(robot2.nextDirect(), 1)]) && robot2.nextDirect() != Direct::pause && (robot2.x + dx[baseDirect(robot2.nextDirect(), 0)] != robot2.mbx || robot2.y + dy[baseDirect(robot2.nextDirect(), 0)] != robot2.mby))
                { // 只能让 robot2 右绕路
                    Direct next = robot2.nextDirect();
                    robot2.insertDirect(baseDirect(next, 0));
                    robot2.insertDirect(baseDirect(next, 1));
                    // logger.log("只能让 robot2 右绕路");
                } 
                else {
                    int dir = int(robot2.nextDirect()) ^ 1;
                    if (isRobotAccessible(robot2.x + dx[dir], robot2.y + dy[dir])) {
                        robot2.insertDirect(Direct(dir^1));
                        robot2.insertDirect(Direct(dir));
                    }
                }
                // else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 0)], robot2.y + dy[baseDirect(robot2.nextDirect(), 0)]) && robot2.nextDirect() != Direct::pause)
                // { // 只能让 robot2 左绕路
                //     Direct next = robot2.nextDirect();
                //     robot2.insertDirect(baseDirect(next, 1));
                //     robot2.insertDirect(baseDirect(next, 0));
                //     // logger.log("只能让 robot2 左绕路");
                // }
                // else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 1)], robot2.y + dy[baseDirect(robot2.nextDirect(), 1)]) && robot2.nextDirect() != Direct::pause)
                // { // 只能让 robot2 右绕路
                //     Direct next = robot2.nextDirect();
                //     robot2.insertDirect(baseDirect(next, 0));
                //     robot2.insertDirect(baseDirect(next, 1));
                //     // logger.log("只能让 robot2 右绕路");
                // }
            } else if (congestion[robot2.x][robot2.y].first == 2) {
                if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]) && robot1.nextDirect() != Direct::pause && (robot1.x + dx[baseDirect(robot1.nextDirect(), 1)] != robot1.mbx || robot1.y + dy[baseDirect(robot1.nextDirect(), 1)] != robot1.mby))
                { // 只能让 robot1 右绕路
                    Direct next = robot1.nextDirect();
                    robot1.insertDirect(baseDirect(next, 0));
                    robot1.insertDirect(baseDirect(next, 1));
                    // logger.log("只能让 robot1 右绕路");
                }
                else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]) && robot1.nextDirect() != Direct::pause && (robot1.x + dx[baseDirect(robot1.nextDirect(), 0)] != robot1.mbx || robot1.y + dy[baseDirect(robot1.nextDirect(), 0)] != robot1.mby))
                { // 只能让 robot1 左绕路
                    Direct next = robot1.nextDirect();
                    robot1.insertDirect(baseDirect(next, 1));
                    robot1.insertDirect(baseDirect(next, 0));
                    // logger.log("只能让 robot1 左绕路");
                }
                else {
                    int dir = int(robot1.nextDirect()) ^ 1;
                    if (isRobotAccessible(robot1.x + dx[dir], robot1.y + dy[dir])) {
                        robot1.insertDirect(Direct(dir^1));
                        robot1.insertDirect(Direct(dir));
                    }
                }
                // else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]) && robot1.nextDirect() != Direct::pause)
                // { // 只能让 robot1 右绕路
                //     Direct next = robot1.nextDirect();
                //     robot1.insertDirect(baseDirect(next, 0));
                //     robot1.insertDirect(baseDirect(next, 1));
                //     // logger.log("只能让 robot1 右绕路");
                // }
                // else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]) && robot1.nextDirect() != Direct::pause)
                // { // 只能让 robot1 左绕路
                //     Direct next = robot1.nextDirect();
                //     robot1.insertDirect(baseDirect(next, 1));
                //     robot1.insertDirect(baseDirect(next, 0));
                //     // logger.log("只能让 robot1 左绕路");
                // }
            }
            else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]) && robot1.nextDirect() != Direct::pause && (robot1.x + dx[baseDirect(robot1.nextDirect(), 1)] != robot1.mbx || robot1.y + dy[baseDirect(robot1.nextDirect(), 1)] != robot1.mby))
            { // 只能让 robot1 右绕路
                Direct next = robot1.nextDirect();
                robot1.insertDirect(baseDirect(next, 0));
                robot1.insertDirect(baseDirect(next, 1));
                // logger.log("只能让 robot1 右绕路");
            }
            else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]) && robot1.nextDirect() != Direct::pause && (robot1.x + dx[baseDirect(robot1.nextDirect(), 0)] != robot1.mbx || robot1.y + dy[baseDirect(robot1.nextDirect(), 0)] != robot1.mby))
            { // 只能让 robot1 左绕路
                Direct next = robot1.nextDirect();
                robot1.insertDirect(baseDirect(next, 1));
                robot1.insertDirect(baseDirect(next, 0));
                // logger.log("只能让 robot1 左绕路");
            }
            else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 0)], robot2.y + dy[baseDirect(robot2.nextDirect(), 0)]) && robot2.nextDirect() != Direct::pause && (robot2.x + dx[baseDirect(robot2.nextDirect(), 1)] != robot2.mbx || robot2.y + dy[baseDirect(robot2.nextDirect(), 1)] != robot2.mby))
            { // 只能让 robot2 左绕路
                Direct next = robot2.nextDirect();
                robot2.insertDirect(baseDirect(next, 1));
                robot2.insertDirect(baseDirect(next, 0));
                // logger.log("只能让 robot2 左绕路");
            }
            else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 1)], robot2.y + dy[baseDirect(robot2.nextDirect(), 1)]) && robot2.nextDirect() != Direct::pause && (robot2.x + dx[baseDirect(robot2.nextDirect(), 0)] != robot2.mbx || robot2.y + dy[baseDirect(robot2.nextDirect(), 0)] != robot2.mby))
            { // 只能让 robot2 右绕路
                Direct next = robot2.nextDirect();
                robot2.insertDirect(baseDirect(next, 0));
                robot2.insertDirect(baseDirect(next, 1));
                // logger.log("只能让 robot2 右绕路");
            }
            else
            {
                if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 1)], robot1.y + dy[baseDirect(robot1.nextDirect(), 1)]) && robot1.nextDirect() != Direct::pause)
                { // 只能让 robot1 右绕路
                    Direct next = robot1.nextDirect();
                    robot1.insertDirect(baseDirect(next, 0));
                    robot1.insertDirect(baseDirect(next, 1));
                    // logger.log("只能让 robot1 右绕路");
                }
                else if (isRobotAccessible(robot1.x + dx[baseDirect(robot1.nextDirect(), 0)], robot1.y + dy[baseDirect(robot1.nextDirect(), 0)]) && robot1.nextDirect() != Direct::pause)
                { // 只能让 robot1 左绕路
                    Direct next = robot1.nextDirect();
                    robot1.insertDirect(baseDirect(next, 1));
                    robot1.insertDirect(baseDirect(next, 0));
                    // logger.log("只能让 robot1 左绕路");
                }
                else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 0)], robot2.y + dy[baseDirect(robot2.nextDirect(), 0)]) && robot2.nextDirect() != Direct::pause)
                { // 只能让 robot2 左绕路
                    Direct next = robot2.nextDirect();
                    robot2.insertDirect(baseDirect(next, 1));
                    robot2.insertDirect(baseDirect(next, 0));
                    // logger.log("只能让 robot2 左绕路");
                }
                else if (isRobotAccessible(robot2.x + dx[baseDirect(robot2.nextDirect(), 1)], robot2.y + dy[baseDirect(robot2.nextDirect(), 1)]) && robot2.nextDirect() != Direct::pause)
                { // 只能让 robot2 右绕路
                    Direct next = robot2.nextDirect();
                    robot2.insertDirect(baseDirect(next, 0));
                    robot2.insertDirect(baseDirect(next, 1));
                    // logger.log("只能让 robot2 右绕路");
                }
            }
        } else if (calcManhattanDist(robot1.x, robot1.y, robot2.x, robot2.y) == 2) {
            if (robot1.x + dx[robot1.nextDirect()] == robot2.x + dx[robot2.nextDirect()] && robot1.y + dy[robot1.nextDirect()] == robot2.y + dy[robot2.nextDirect()]) {
                robot1.insertDirect(Direct::pause);
            }
        } else {
            
        }
        if (temp == collision || while_limit <= 0) {
            break;
        }
        collision = detectCollision();
    }
}