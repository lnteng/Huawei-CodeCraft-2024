#pragma once
#include <bits/stdc++.h>
#include "logger.hpp"

using namespace std;
#define Point pair<int, int>

const int zhen_total = 15000; // 总帧数
const int n = 200;            // 实际地图宽度
const int robot_num = 10;     // 初始机器人数量
const int berth_num = 10;     // 泊位数量
const int boat_num = 5;       // 船只数量
const int N = 210;            // 初始化地图宽度
const int thousand = 1000;
const int Fault_tolerance = 500;       // 时间容错，单位：帧，用于船舶最后返航时间的容错
const double Goods_tolerance = 1.1;       // 取货物容错系数,乘以实际最短距离，应该大于1
const double boat_return_weight = 0.8; // 接近船舶满载权重，用于泊位剩余货物充足的判定
const int select_berth_num = 5;        // 选择的固定泊位数量
const int MAX_LIMIT = 999;             // 将此距离视为不可达
const Point boat_virtual_point = make_pair(200, 200); // 船舶虚拟点/不可达点

// #Debug Info
int robot_recover_count = 0;      // 机器人碰撞恢复总帧数统计

enum Direct // 机器人移动方向
{
    right,
    left,
    upper,
    down,
    pause
};

struct Robot // 机器人
{
    int x, y, goods; // x,y:坐标 goods:货物数量
    int status;      // 机器人状态 0:恢复中 1:正常运行
    // int mbx, mby;
    vector<Direct> path; // 记录 A* 计算的路径
    int pid;             // 走到第几步
    int goodValue;       // 携带 good 的 value
    Robot() {}
    Robot(int startX, int startY)
    {
        x = startX;
        y = startY;
        pid = 0;
        goodValue = 0;
    }
    void incrementPid() //更新pid和下一步方向dir
    {
        ++pid;
    }
    bool hasPath() // 路径是否走完
    {
        return pid < path.size();
    }
    void newPath(vector<Direct> &paths) // 更新路径信息
    {
        path.clear();
        pid = 0;
        this->path = paths;
    }
    void rollBack() // 目前没有使用，使用会出现更多问题
    {
        if (pid > 0) {
            --pid;
        }
    }
} robots[robot_num + 10];

struct Berth // 泊位
{
    int x;
    int y;
    int transport_time;   // 运输时间，打印传入时信息在1000帧左右
    int loading_speed;    // 装载速度（个每帧）
    int remain_goods_num; // 剩余货物数量
    queue<int> remain_goods_value; // 剩余货物 value
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
    void popRemainGoods(int num)
    {
        while (num > 0) 
        {
            this->remain_goods_value.pop();
            --num;
        }
    }
} berths[berth_num + 10];
int selected_berth[5]; // 选择的固定泊位数组，固定泊位seleted_berth_id到泊位id

struct Boat // 船舶
{
    int num, pos; // num:用于记录船舶装载数目
    int status; // 船舶状态 0:运输中 1:泊位上 2:泊位外等待
    Boat() : num(0) {}
} boats[10];

// 自定义哈希函数
struct hash_pair {
    size_t operator()(const pair<int, int>& p) const {
        auto hash1 = hash<int>{}(p.first);
        auto hash2 = hash<int>{}(p.second);
        return hash1 ^ hash2;
    }
};

struct GoodsProperty // 货物属性
{
    int value;    // 货物金额，上限200
    int end_time; // 货物消失时间 //TODO：每一帧增加消失货物删除
    double priority; // 机器人拾取优先级
    bool marked; // good 是否被 robot 标记，选择货物时使用
    GoodsProperty() : value(0), end_time(0), priority(0.0) {}
    GoodsProperty(int value, int start_time)
    {
        this->value = value;
        this->end_time = start_time + thousand; // 出现1000帧后消失
        this->marked = false;
        this->priority = 0.0;                     // 0为最低优先级
    }
    void updatePriority(int dist)
    {
        // this->priority = value / dist; // 方案一：货物优先级=货物价值/距离
        this->priority = value * value / dist; // 方案二：货物优先级=货物价值平方/距离
    }
};

unordered_map<Point, GoodsProperty, hash_pair> gds; // 货物(坐标->货物属性)
int money, boat_capacity, id; // boat_capacity:所有船舶容量相同；id:帧ID
char ch[N][N];                // 地图
int dists[berth_num][N][N];   // 泊位到各个点的距离
int berth_field[N][N];        // 属于固定泊位的区域id, 和固定泊位一致，-1表示不可访问区域

Logger logger("./results/debug.log");

// 定义方向：右，左，上，下
const int dx[5] = {0, 0, -1, 1, 0};
const int dy[5] = {1, -1, 0, 0, 0};

// 判断是否越界
bool isVaild(int x, int y, Direct dir)
{
    if (x + dx[dir] < 0 || x + dx[dir] >= n ||
        y + dy[dir] < 0 || y + dy[dir] >= n)
    {
        return false;
    }
    return true;
}

inline bool isRobotAccessible(int x, int y) // 判断机器人是否可以访问
{
    return isVaild(x,y,pause) &&(ch[x][y] == '.' || ch[x][y] == 'A' || ch[x][y] == 'B');
}

inline int getDistByPoint(int bIdx, Point p) {
    return dists[bIdx][p.first][p.second];
}

inline int getDistByRobot(int bIdx, Robot& robot) {
    return dists[bIdx][robot.x][robot.y];
}

inline int getDistByBerth(int bIdx, Berth& berth) {
    return dists[bIdx][berth.x][berth.y];
}

inline void summary(int zhen,int zhenId) { // 总结最后结算信息
    // 记录港口剩余货物数目
    for (int i = 0; i < berth_num; i++)
    {
        string remain_goods_values;
        for (int j = 0; j < 100 && j < berths[i].remain_goods_value.size(); j++)
        {
            remain_goods_values += to_string(berths[i].remain_goods_value.front()) + ", ";
            berths[i].remain_goods_value.pop();
        }
        logger.log(INFO, formatString("berth {} :remain_goods_num: {} ", i, berths[i].remain_goods_num) + remain_goods_values);
    }
    logger.log(INFO, formatString("跳帧:{},机器人恢复状态总帧数:{}", (zhenId-zhen),robot_recover_count));
}