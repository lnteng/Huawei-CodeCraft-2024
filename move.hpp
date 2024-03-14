#pragma once
#include <bits/stdc++.h>
#include "const.h"
/*
    格式化相关操作
    idx: 机器人 ID
    ida: 泊位 ID
    idb: 船舶 ID
*/

inline void robotRight(int idx) {
    printf("move %d %d\n", idx, 0);
}

inline void robotLeft(int idx) {
    printf("move %d %d\n", idx, 1);
}

inline void robotUpper(int idx) {
    printf("move %d %d\n", idx, 2);
}

inline void robotDown(int idx) {
    printf("move %d %d\n", idx, 3);
}

inline void robotGet(int idx) {
    printf("get %d\n", idx);
}

inline void robotPull(int idx) {
    printf("pull %d\n", idx);
}

inline void boatShip(int idb, int ida) {
    printf("ship %d %d\n", idb, ida);
}

inline void boatGo(int idb) {
    printf("go %d\n", idb);
}

inline void Ok() {
    printf("OK\n");
}