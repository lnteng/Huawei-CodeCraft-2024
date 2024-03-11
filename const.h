#pragma once
#include <bits/stdc++.h>
#include "logger.hpp"

using namespace std;
#define Point pair<int, int>
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

enum Direct {
    right,
    left,
    upper,
    down,
};

struct Robot {
    int x, y, goods;
    int status;
    int mbx, mby;
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
} robots[robot_num + 10];

struct Berth {
    int x;
    int y;
    int transport_time;
    int loading_speed;
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
} berths[berth_num + 10];

struct Boat {
    int num, pos, status;
    int zId;
    Boat(): num(0) {}
} boats[10];

int money, boat_capacity, id;
char ch[N][N];
// int gds[N][N];
int dists[berth_num][N][N];

map<Point, int> gds;
set<Point> paths[robot_num]; // 机器人走过的路径

Logger logger("./replay/debug.log");

// 定义方向
const int dx[4] = {0, 0, -1, 1};
const int dy[4] = {1, -1, 0, 0};