#pragma once
#include <bits/stdc++.h>
#include "logger.hpp"

using namespace std;
#define Point pair<int, int>

const int zhen_total = 15000; // 总帧数
const int n = 200; //实际地图宽度
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5; // 船只数量
const int N = 210; 
const int thousand = 1000; 
const int Fault_tolerance = 5; // 时间容错，单位：帧，用于船舶最后返航时间的容错
const int boat_return_weight = 0.8 ; // 港口剩余货物充足的判定，相较于船舶容量
const int select_berth_num = 5; // 选择的固定港口数量



enum Direct {
    right,
    left,
    upper,
    down,
};

struct Robot {
    int x, y, goods;
    int status;
    // int mbx, mby;
    vector<Direct> path; // 记录 A* 计算的路径
    int pid; // 走到第几步
    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
        pid = 0;
    }
    void incrementPid() {
        ++pid;
    }
    bool hasPath() {
        return pid < path.size();
    }
    void newPath(vector<Direct> &paths) {
        path.clear();
        pid = 0;
        this->path = paths;
    }
    void newPath() {
        path.clear();
        pid = 0;
        vector<Direct> paths;
        this->path = paths;
    }
} robots[robot_num + 10];

struct Berth {
    int x;
    int y;
    int transport_time;
    int loading_speed;
    int remain_goods_num;
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
        this->remain_goods_num=0;
    }
} berths[berth_num + 10];
int selected_berth[5]; // 选择的固定港口ID

struct Boat {
    int num, pos, status; //num:use for logging the number of goods on boat
    int zId;
    Boat(): num(0) {}
} boats[10]; 

// value, start_time, priority, robot_id
struct GoodsProperty {
    int value;
    int end_time;
    int priority;
    GoodsProperty() : value(0), end_time(0), priority(0) {}
    GoodsProperty(int value, int start_time) {
        this->value = value;
        this->end_time = start_time + thousand; //zhenId
        this->priority = 0; // 0为最低优先级
    }
    void setPriority(int priority) { //TODO: 计算优先级
        this->priority = priority;
    }
};

int money, boat_capacity, id; //boat_capacity相同
char ch[N][N];
// int gds[N][N];
int dists[berth_num][N][N];
int berth_field[N][N]; //属于固定港口的区域id

// 自定义哈希函数
struct hash_pair {
    size_t operator()(const pair<int, int>& p) const {
        auto hash1 = hash<int>{}(p.first);
        auto hash2 = hash<int>{}(p.second);
        return hash1 ^ hash2;
    }
};

unordered_map<Point, GoodsProperty, hash_pair> gds;
unordered_map<Point, bool, hash_pair> gds_flag; // good 是否被 robot 标记

Logger logger("./replay/debug.log");

// 定义方向
const int dx[4] = {0, 0, -1, 1};
const int dy[4] = {1, -1, 0, 0};