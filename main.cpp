#include <bits/stdc++.h>

#include "move.hpp"
#include "graph.hpp"
#include "logger.hpp"

Logger logger("./replay/debug.log");

void Init() {
    for (int i = 0; i < n; i++) { // 从 ch[0][0] -> ch[n-1][n-1]
        scanf("%s", ch[i] + 1);
    }
    for (int i = 0; i < berth_num; i++) {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berths[id].x, &berths[id].y, &berths[id].transport_time, &berths[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    get_dist_by_bfs();
    printf("OK\n");
    fflush(stdout);
}

int Input() {
    scanf("%d%d", &id, &money);
    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
    }
    for (int i = 0; i < robot_num; i++) {
        int sts;
        scanf("%d%d%d%d", &robots[i].goods, &robots[i].x, &robots[i].y, &sts);
    }
    for (int i = 0; i < 5; i++) {
        scanf("%d%d\n", &boats[i].status, &boats[i].pos);
    }
    char okk[100];
    scanf("%s", okk);
    return id;
}

int main() {
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++) {
        int id = Input();
        for(int i = 0; i < robot_num; i ++)
            printf("move %d %d\n", i, rand() % 4);
        puts("OK");
        fflush(stdout);
    }

    for (int i = 0; i < berth_num; i++) { 
        for (int x = 0; x < N; x++) {
            std::ostringstream oss;
            oss << "[";
            for (int y = 0; y < N; y++) {
                oss << dists[i][x][y] << " ";
            }
            oss << "]";
            logger.log(INFO, oss.str());
        }
        printf("\n\n");
    }
    return 0;
}
