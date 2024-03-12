#include <ctime>
#include <iterator>
#include <cstdlib>
#include "move.hpp"
#include "graph.hpp"

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

bool isVaild(int x, int y, Direct dir) {
    if (x + dx[dir] < 0 || x + dx[dir] >= n || 
        y + dy[dir] < 0 || y + dy[dir] >= n) {
        return false;
    }
    return true;
}

void printMoreDebugINfo() {
    for (int i = 0; i < berth_num; i++) { // debug
        for (int x = 0; x < N; x++) {
            std::ostringstream oss;
            oss << "[";
            for (int y = 0; y < N; y++) {
                oss << dists[i][x][y] << "\t";
            }
            oss << "]";
            logger.log(INFO, oss.str());
        }
    }
}

void Output(int zhenId) {

    int i = 0;
    Robot& robot = robots[i]; // demo: 仅操作一个机器人
    Point pRobut = make_pair(robot.x, robot.y);
    Berth& berth = berths[0]; // 前往第一个 berth
    if (robot.goods == 0) { // 未携带货物
        if (!robot.hasPath()) {
            robotGet(i);
            logger.log(INFO, "get");
            logger.log(INFO, "robot current pos: " + to_string(pRobut.first) + ", " + to_string(pRobut.second));
            std::vector<Direct> paths;
            Point pGood;
            do {
                std::srand(std::time(0));
                if(gds.empty()) {
                    break;
                }
                int randomIndex = std::rand() % gds.size();
                auto it = gds.begin();
                std::advance(it, randomIndex);
                Point pGood = it->first; // 取出第一个 good
                gds.erase(it);
                paths = AStar(pRobut, pGood);
            } while (paths.empty());
            logger.log(INFO, "A star :robot"+to_string(pRobut.first) + ", " + to_string(pRobut.second));
            robot.newPath(paths);
            logger.log(INFO, "calc new path, lenght: " + to_string(robot.path.size()));
        } else {
            logger.log(INFO, "move along path, idx: " +  to_string(robot.pid));
            // 根据计算出来的最短路径移动 robot
            robotMove(i, robot.path[robot.pid]);
            robot.incrementPid();
        }
    } else { // 携带有货物
        robot.newPath();
        for (int dir = 0; dir < 4; dir++) {
            if (isVaild(robot.x, robot.y, (Direct)dir) && dists[0][robot.x + dx[dir]][robot.y + dy[dir]] < dists[0][robot.x][robot.y]) { // TODO: bugfix
                robotMove(i, (Direct)dir);
                logger.log(INFO, to_string((Direct)dir) + " " + to_string(dists[0][robot.x][robot.y]));
                if (dists[0][robot.x][robot.y] == 1) {
                    robotPull(i);
                    berth.remain_goods_num += 1; // TODO:remain pull success
                    logger.log(INFO, "pull");
                    // logger.log(INFO, "boatGo " + to_string(boats[0].num));
                }
            }
        }
    }

    // 处理船舶
    for (int i = 0; i < 5; i++) {
        if (boats[i].status == 0) { // 船舶状态为 0,
            continue;
        }
        if (boats[i].status == 2) { // TODO：船舶状态为 1，是否已有船舶在泊位上
            logger.log(INFO, "boats.status: 1");
        }
        if (boats[i].pos == -1) {
            logger.log(INFO, "boatShip " + to_string(i) + " to " + to_string(i));
            boatShip(i, i);
        }
        bool end_flag = (berth.transport_time+zhenId==14991||berth.transport_time+zhenId==14990);
        // if ((berth.remain_goods_num > boat_capacity || end_flag) 
        //     && boats[i].status == 1 && zhenId%2 && boats[i].num >= boat_capacity*0.9) {  
        //     if (berth.remain_goods_num >= berth.loading_speed && !end_flag) {
        //         berth.remain_goods_num -= berth.loading_speed;
        //         boats[i].num += berth.loading_speed;
        //     } else {
        //         if (berth.remain_goods_num >= 0 && !end_flag) {
        //             boats[i].num += berth.remain_goods_num;
        //             berth.remain_goods_num = 0;
        //             logger.log(INFO, "loading goods end");
        //         }else{
        //             logger.log(INFO, "boatGo " + to_string(i));
        //             boatGo(i);
        //         }
        //     }
        // }
        if (berth.remain_goods_num > boat_capacity * 0.9) {
            berth.remain_goods_num -= berth.loading_speed;
            boats[i].num += berth.loading_speed;          
        } else {
            if (berth.remain_goods_num > 0) {
                boats[i].num += berth.remain_goods_num;
                berth.remain_goods_num = 0;
                logger.log(INFO, "loading goods end");
            } else {
                if (boats->pos != -1){
                    logger.log(INFO, "boatGo " + to_string(i));
                    boatGo(i);
                } 
            }
        }
    }
}

int main() {
    Init();
    printMoreDebugINfo();
    for(int zhen = 1; zhen <= 15000; zhen ++) {
        int zhenId = Input();
        Output(zhenId);
        Ok();
        fflush(stdout);
    }
    return 0;
}
