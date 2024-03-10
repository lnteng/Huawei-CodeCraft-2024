#pragma once
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
} boats[10];

int money, boat_capacity, id;
char ch[N][N];
int gds[N][N];
int dists[berth_num][N][N];

struct Point {
    int x;
    int y;
};