#pragma once
#include <bits/stdc++.h>

#include "logger.hpp"

using namespace std;
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

struct Robot {
    int x, y, goods;
    int status;
    int mbx, mby;
    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
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

#define Point pair<int, int>
map<Point, int> gds;
set<Point> paths[robot_num]; // 机器人走过的路径

Logger logger("./replay/debug.log");
