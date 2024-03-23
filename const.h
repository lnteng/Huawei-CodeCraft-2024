#pragma once
#include <bits/stdc++.h>
#include "logger.hpp"

Logger logger("./results/debug.log");

using namespace std;
#define Point pair<int, int>

const int zhen_total = 15000; // 总帧数
const int n = 200;            // 实际地图宽度
const int robot_num = 10;     // 初始机器人数量
const int berth_num = 10;     // 泊位数量
const int boat_num = 5;       // 船只数量
const int N = 210;            // 初始化地图宽度
const int thousand = 1000;
const int Fault_tolerance = 5;       // 时间容错，单位：帧，用于船舶最后返航时间的容错
const double Goods_tolerance = 1.1;       // 取货物容错系数,乘以实际最短距离，应该大于1
const double boat_return_weight = 0.8; // 接近船舶满载权重，用于泊位剩余货物充足的判定
const int select_berth_num = 5;        // 选择的固定泊位数量
const int MAX_LIMIT = 999;             // 将此距离视为不可达
const Point boat_virtual_point = make_pair(200, 200); // 船舶虚拟点/不可达点
const int High_congestion = 2; // 拥堵度阈值(含等于)
const int high_congestion_cost = 1; // 高拥堵度代价
pair<int, int> congestion[N][N]; // 拥堵度，记录每个点的不可达方向数目(-1表示未初始化)和连续高拥堵度点数目(默认为0)
const double goods_withinfield_ratio = 0; // 固定泊位区域外可选货物距离比例（避免机器人过于集中.1为机器人最大范围，0为不选择区域外获取） //TODO也可以考虑直接用一个固定值替代
const int rounding_num = 5; // 小数近似控制，rounding_num舍rounding_num+1入 // TODO:test
const int berth_field_radius = 40; // 变更固定泊位时，比较货物区域半径 // TODO 改为该区域最远点距离的一个比例
const int berth_field_over = 50; // 检测泊位曼哈顿距离小于等于60的点，用于判断是否属于固定泊位选择
const int high_single_channel_counts = 650;

vector<vector<int>> berth_groups_vec; // 泊位组，注意泊位组初始化以后暂时不会变动
int berthBelongGroup[berth_num]; // 泊位所属泊位组，用于变更泊位时使用
// #Debug Info
int robot_recover_count = 0;      // 机器人碰撞恢复总帧数统计
int endBoatGroup[boat_num] = {1}; // 记录仍可使用的泊位组id，用于结束时判断
int single_channel_counts = 0; //单通道数目统计

enum Direct // 机器人移动方向
{
    right,
    left,
    upper,
    down,
    pause
};

int extra_steps2avoid_collision = 0; // 碰撞避免额外走的路程
int expected_scores = 0;    // 预期得分，与实际得分进行对比
int boat_berthing_time_count[berth_num] = {0}; // 统计:船舶靠泊时间
int without_goods_time_count[berth_num] = {0}; // 统计:固定泊位区域无货物的帧数*机器人数

struct Robot // 机器人
{
    int robotId;
    int x, y, goods; // x,y:坐标 goods:货物数量
    int status;      // 机器人状态 0:恢复中 1:正常运行
    int mbx, mby; // 不绑定货物，考虑到取得货物以前可能出现新的更优货物
    vector<Direct> path; // 记录 A* 计算的路径
    int pid;             // 走到第几步
    int goodValue;       // 携带 good 的 value
    int selected_berthIdx;     // 机器人选择固定泊位
    int rollback;
    Robot() {}
    Robot(int startX, int startY)
    {
        x = startX;
        y = startY;
        pid = 0;
        goodValue = 0;
        selected_berthIdx = -1; // 未选择固定泊位
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
    Direct nextDirect() // 目前没有使用，使用会出现更多问题
    {
        if (this->hasPath()) {
            // logger.log("nextDirect " + to_string(this->pid));
            return this->path[this->pid];
        } else {
            return Direct::pause;
        }
    }
    void insertDirect(Direct dir) // 目前没有使用，使用会出现更多问题
    {
        // logger.log("before insertDirect " + to_string(this->pid) + " " + to_string(this->path[this->pid]));
        // logger.log("insertDirect" + to_string(dir));
        this->path.insert(this->path.begin() + this->pid, dir);
        // logger.log("after insertDirect " + to_string(this->pid) + " " + to_string(this->path[this->pid]));
        ++extra_steps2avoid_collision;
    }
    void insertDirectAfter(Direct dir) // 目前没有使用，使用会出现更多问题 //TODO 注意现在下标
    {
        this->path.insert(this->path.begin() + this->pid+1, dir);
        ++extra_steps2avoid_collision;
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
            expected_scores += this->remain_goods_value.front();
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
    { // 二次方左右差别不大
        // this->priority = value / dist; // 方案一：货物优先级=货物价值/距离
        this->priority = value * value / dist; // 方案二：货物优先级=货物价值平方/距离
        // this->priority = pow(value, 2.5) /dist;// 方案三：货物优先级=货物价值1.5次方/距离
    }
    double getPriorityOutsideFeild(int dist)
    {
        return value * value / dist; // 与updatePriority保持一致
    }
};

unordered_map<Point, Point, hash_pair> entry2exit;
unordered_map<Point, int, hash_pair> oneway_lock;

unordered_map<Point, GoodsProperty, hash_pair> gds; // 货物(坐标->货物属性)
int money, boat_capacity, id; // boat_capacity:所有船舶容量相同；id:帧ID
char ch[N][N];                // 地图
int dists[berth_num][N][N];   // 泊位到各个点的距离
int berth_field[N][N];        // 属于固定泊位的区域id, 和固定泊位一致，-1表示不可访问区域
int berth_field_count[berth_num]; //统计固定泊位辐射可达点数目，用于初始化机器人，有冗余的位置避免溢出
int reachable_point_count=0; // 统计可达点数目


// 定义方向：右，左，上，下
const int dx[5] = {0, 0, -1, 1, 0};
const int dy[5] = {1, -1, 0, 0, 0};
const int leftside[5] = {2, 3, 1, 0, 5};
const int rightside[5] = {3, 2, 0, 1, 5};

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
        int num = berths[i].remain_goods_value.size();
        for (int j = 0; j < 100 && j < num; j++)
        {
            remain_goods_values += to_string(berths[i].remain_goods_value.front()) + ", ";
            berths[i].remain_goods_value.pop();
        }
        logger.log(INFO, formatString("berth {} :remain_goods_num: {}, include: ", i, berths[i].remain_goods_num) + remain_goods_values);
    }
    logger.log(INFO, formatString("跳帧:{},机器人恢复状态总帧数:{}", (zhenId-zhen),robot_recover_count));
    logger.log(INFO, formatString("碰撞避免额外走的路程:{}", extra_steps2avoid_collision));
    logger.log(INFO, formatString("预期得分:{}", expected_scores));
    logger.log(INFO,"船舶靠泊时间统计:");
    for (int i=0;i<berth_num;i++) { // TODO：复赛时需要评估船舶瓶颈时细化到每次往返在泊位停靠时间
        logger.log(INFO, formatString("  berth {} :boat berthing_time: {}", i, boat_berthing_time_count[i]));
    }
    logger.log(INFO,"泊位区域无货物的帧数（*机器人数）统计:");
    for (int i=0;i<berth_num;i++) {
        if (without_goods_time_count[i] == 0) continue;
        logger.log(INFO, formatString("  Berth {} :without_goods_time: {}", i, without_goods_time_count[i]));
    }
    logger.log(INFO, formatString("固定泊位辐射面积统计:{}", reachable_point_count)); 
    for (int i=0;i<boat_num;i++) {
        logger.log(INFO, formatString("  selectedBerth {} :berth_field_count: {}; berth", i, berth_field_count[i]));
    }
    for (auto& [entry, exit]: entry2exit) {
        logger.log(formatString("oneway entry: ({}, {}), exit: ({}, {})", entry.first, entry.second, exit.first, exit.second));
    }
    logger.log(INFO, "summary end");
}