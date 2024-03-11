#include "move.hpp"
#include "graph.hpp"
#include "AstarByBert.cpp"

void Init() {
    for (int i = 0; i < n; i++) { // 从 ch[0][0] -> ch[n-1][n-1]
        scanf("%s", ch[i]);
    }
    for (int i = 0; i < berth_num; i++) {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berths[id].x, &berths[id].y, &berths[id].transport_time, &berths[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    getDistByBfs();
    Ok();
    fflush(stdout);
}

int Input() {
    scanf("%d%d", &id, &money); // 帧序号 当前金钱数
    int num;
    scanf("%d", &num); // 新增货物数
    for (int i = 1; i <= num; i++) { // 新增货物
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        Point point = make_pair(x, y);
        gds.insert({point, val});
    }
    for (int i = 0; i < robot_num; i++) { // 机器人状态
        scanf("%d%d%d%d", &robots[i].goods, &robots[i].x, &robots[i].y, &robots[i].status);
    }
    for (int i = 0; i < 5; i++) { // 船状态
        scanf("%d%d\n", &boats[i].status, &boats[i].pos);
    }
    char okk[100];
    scanf("%s", okk);
    return id;
}

void Output(int zhenId) {
    // int i = 0;
    for(int i = 0; i < 1; i++){
            Robot robot = robots[i]; // demo: 仅操作一个机器人
        Berth berth = berths[0]; // 前往第一个 berth
        Point good = gds.begin()->first; // 取出第一个 good
        int value = gds.begin()->second; // 取出第一个 good 的 value

        // gds.erase(good); // 删除第一个 good
        // 现在我希望机器人i去取goods第%robot_num%个货物

        if (robot.goods == 0) { // 未携带货物
            vector<int> path = AStar(robot.x, robot.y, good.first, good.second);
            //如果path没有元素，则
            if (path.size() == 0) {
                robotGet(i);
                logger.log(INFO, "get");
            }else{
                switch (path[0]) {
                    case 0:
                        robotRight(i);
                        logger.log(INFO, "robotRight");
                        break;
                    case 1:
                        robotLeft(i);
                        logger.log(INFO, "robotLeft");
                        break;
                    case 2:
                        robotUpper(i);
                        logger.log(INFO, "robotUpper");
                        break;
                    case 3:
                        robotDown(i);
                        logger.log(INFO, "robotDown");
                        break;
                    default:
                        break;
                }
            }
        } else { // 携带有货物
            if (robot.y + 1 < n && dists[0][robot.x][robot.y + 1] < dists[0][robot.x][robot.y]) { // right
                robotRight(i);
                logger.log(INFO, "robotRight " + to_string(dists[0][robot.x][robot.y]));
                if (dists[0][robot.x][robot.y] == 1) {
                    robotPull(i);
                    boats[0].num += value;
                    boats[0].zId = zhenId + 500;
                    logger.log(INFO, "pull");
                    // logger.log(INFO, "boatGo " + to_string(boats[0].num));
                    // boatGo(0);
                }
            }
            if (robot.y > 0 && dists[0][robot.x][robot.y - 1] < dists[0][robot.x][robot.y]) { // left
                robotLeft(i);
                logger.log(INFO, "robotLeft " + to_string(dists[0][robot.x][robot.y]));
                if (dists[0][robot.x][robot.y] == 1) {
                    robotPull(i);
                    robotPull(i);
                    boats[0].num += value;
                    boats[0].zId = zhenId + 500;
                    logger.log(INFO, "pull");
                    // logger.log(INFO, "boatGo " + to_string(boats[0].num));
                    // boatGo(0);
                }
            }
            if (robot.x > 0 && dists[0][robot.x - 1][robot.y] < dists[0][robot.x][robot.y]) { // upper
                robotUpper(i);
                logger.log(INFO, "robotUpper " + to_string(dists[0][robot.x][robot.y]));
                if (dists[0][robot.x][robot.y] == 1) {
                    robotPull(i);
                    robotPull(i);
                    boats[0].num += value;
                    boats[0].zId = zhenId + 500;
                    logger.log(INFO, "pull");
                    // logger.log(INFO, "boatGo " + to_string(boats[0].num));
                    // boatGo(0);
                }
            }
            if (robot.x + 1 < n && dists[0][robot.x + 1][robot.y] < dists[0][robot.x][robot.y]) { // down
                robotDown(i);
                logger.log(INFO, "robotDown " + to_string(dists[0][robot.x][robot.y]));
                if (dists[0][robot.x][robot.y] == 1) {
                    robotPull(i);
                    robotPull(i);
                    boats[0].num += value;
                    boats[0].zId = zhenId + 500;
                    logger.log(INFO, "pull");
                    // logger.log(INFO, "boatGo " + to_string(boats[0].num));
                    // boatGo(0);
                }
            }
        }

    }

    // 处理船舶
    for (int i = 0; i < 5; i++) {
        if (boats[i].pos == -1 and boats[i].status == 1) {
            logger.log(INFO, "frist boatShip"); 
            boatShip(i, i);
        }
        // if (zhenId == 1 or zhenId == 2 or zhenId == 3) {
        //     logger.log(INFO, to_string(zhenId)+" boat status " + to_string(boats[i].status));
        // }

        if (boats[i].num > 0 && boats[i].zId == zhenId && boats[i].status == 1) { 
            logger.log(INFO, "boatGo " + to_string(i) + to_string(boats[i].num));
            boatGo(i);
        }

        if (boats[i].pos == -1 and boats[i].status == 1) {
            logger.log(INFO, "boatShip");
            boatShip(i, i);
        }
    }
}

int main() {
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++) {
        int zhenId = Input();
        Output(zhenId);
        Ok();
        fflush(stdout);
    }

    // for (int i = 0; i < berth_num; i++) { // debug
    //     for (int x = 0; x < N; x++) {
    //         std::ostringstream oss;
    //         oss << "[";
    //         for (int y = 0; y < N; y++) {
    //             oss << dists[i][x][y] << "\t";
    //         }
    //         oss << "]";
    //         logger.log(INFO, oss.str());
    //     }
    //     printf("\n\n");
    // }

    return 0;
}

