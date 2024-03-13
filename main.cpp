#include <ctime>
#include <iterator>
#include <cstdlib>
#include "move.hpp"
#include "graph.hpp"
#include "optimizer.hpp"

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
    selectBerth();
    Ok();
    fflush(stdout);
}

inline void robotMove(int idx, Direct dir) {
    robots[idx].x += dx[dir];
    robots[idx].y += dy[dir];
    switch (dir) {
        case Direct::right:
            robotRight(idx);
            break;
        case Direct::left:
            robotLeft(idx);
            break;
        case Direct::down:
            robotDown(idx);
            break;
        case Direct::upper:
            robotUpper(idx);
            break;
        default:
            break;
    }
}

int Input() {
    scanf("%d%d", &id, &money); // 帧序号 当前金钱数
    int num;
    scanf("%d", &num); // 新增货物数
    for (int i = 1; i <= num; i++) { // 新增货物
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        Point point = make_pair(x, y);
        gds[point] = GoodsProperty(val, id); //TODO：计算优先级
        gds_flag[point] = false;
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


Point pickGood(int bIdx, int zhenId) {
    Point p;
    int maxPriority = 0;
    for (const auto& gd : gds) {
        // good 不需要考虑 good 超时删除问题
        if (gd.second.end_time < zhenId) {
            gds_flag[p] = true;
            continue;
        }
        if (gds_flag[gd.first] == true) {
            continue;
        }
        int dist = dists[bIdx][gd.first.first][gd.first.second];
        int newPriority = gd.second.value / dist;
        if (1 * dist + zhenId > gd.second.end_time) { // 当前机器人来不及处理该 good
            continue;
        }
        if (newPriority > maxPriority) {
            p = gd.first;
            maxPriority = newPriority;
        }
    }
    gds_flag[p] = true;
    return p;
}

int nearBerth(Point curPoint) {
    int bIdx = 0;
    int minDist = INT16_MAX;
    for (int i: selected_berth) {
        int newDist = dists[i][curPoint.first][curPoint.second];
        if (newDist < minDist) {
            bIdx = i;
            minDist = newDist;
        }
    }
    return bIdx; //返回对应港口的dist下标
}

void Output(int zhenId) {
    for (int robotIdx = 0; robotIdx < robot_num; robotIdx++) {
        Robot& robot = robots[robotIdx]; 
        Point pRobut = make_pair(robot.x, robot.y);
        if (robot.status == 0) { // 碰撞后的恢复状态
            continue;
        }
        if (robot.goods == 0) { // 未携带货物
            if (!robot.hasPath()) {
                if (gds.count(pRobut) > 0) { // 当前位置有货物
                    robotGet(robotIdx);
                    logger.log(INFO, "get");
                    gds.erase(pRobut);
                } else { // 当前位置没有货物
                    if(gds.empty()) {
                        continue;
                    }
                    int randomIndex = std::rand() % gds.size();
                    auto it = gds.begin();
                    std::advance(it, randomIndex);
                    Point pGood = it->first;
                    vector<Direct> paths = AStar(pRobut, pGood);
                    robot.newPath(paths);
                    continue;
                }
            } else {
                // 根据计算出来的最短路径移动 robot
                robotMove(robotIdx, robot.path[robot.pid]);
                robot.incrementPid();
            }
        } else { // 携带有货物
            int berthIdx = nearBerth(make_pair(robot.x, robot.y));
            Berth& berth = berths[berthIdx];
            vector<int> nums = {0, 1, 2, 3};
            std::random_device rd;
            std::mt19937 g(rd());
            // 打乱数组顺序
            std::shuffle(nums.begin(), nums.end(), g);
            for (int dir: nums) {
                if (isVaild(robot.x, robot.y, (Direct)dir) && dists[berthIdx][robot.x + dx[dir]][robot.y + dy[dir]] < dists[berthIdx][robot.x][robot.y]) { // TODO: bugfix
                    robotMove(robotIdx, (Direct)dir);
                    if (dists[berthIdx][robot.x][robot.y] == 0) {
                        robotPull(robotIdx);
                        berth.remain_goods_num += 1;
                        logger.log(INFO, "pull");
                        Point pGood = pickGood(berthIdx, zhenId);
                        vector<Direct> paths = AStar(make_pair(robot.x, robot.y), pGood);
                        robot.newPath(paths);
                    }
                }
            }
        }
    }

    // 处理船舶
    for (int i = 0; i < 5; i++) {
        if (boats[i].status == 0) { // 船舶状态为 0,
            if (boats[i].num !=0) {
                boats[i].num =0;
            }
            continue;
        }
        if (boats[i].status == 2) { // TODO：船舶状态为 1，是否已有船舶在泊位上
            logger.log(INFO, "boats.status: 1");
            continue;
        }
        if (boats[i].pos == -1) {
            logger.log(INFO, "boatShip " + to_string(i) + " to " + to_string(i));
            boatShip(i, selected_berth[i]); // TODO：选择
            continue;
        }
        Berth& berth = berths[boats[i].pos];
        // bool end_flag = (berth.transport_time+zhenId==14991||berth.transport_time+zhenId==14990); // TODO ：快要结束时，船舶前往虚拟点（注意避免重复指令导致刷新运送时间）
        bool end_flag = (berth.transport_time + zhenId + Fault_tolerance > zhen_total);
        if (berth.remain_goods_num >= berth.loading_speed && !end_flag && boats[i].num + berth.loading_speed <= boat_capacity) {
            // 船舶装载货物空间充足且港口剩余货物充足且未临近结束时间
            berth.remain_goods_num -= berth.loading_speed;
            boats[i].num += berth.loading_speed;  
        } else {
            if ((berth.remain_goods_num > 0 || boats[i].num + berth.loading_speed > boat_capacity)&& !end_flag) {
                if (berth.remain_goods_num > boat_capacity - boats[i].num){
                    berth.remain_goods_num = boat_capacity - (boat_capacity -boats[i].num);
                    boats[i].num = boat_capacity;
                    logger.log(INFO, "boatGo " + to_string(i)); 
                    boatGo(i);
                }else{
                    boats[i].num += berth.remain_goods_num;
                    berth.remain_goods_num = 0;
                }
            } else { //船舶装载完毕或船舶剩余货物未空或临近结束时间
                if (boats->pos != -1 && (end_flag || boats[i].num == boat_capacity)){ //船舶装载完毕或船舶剩余货物未空
                    logger.log(INFO, "boatGo " + to_string(i)); 
                    boatGo(i);
                } 
            }
        }
    }

}

int main() {
    Init();
    // printMoreDebugINfo();
    for(int zhen = 1; zhen <= zhen_total; zhen ++) {
        int zhenId = Input();
        if (zhen == 1 and id > 1) {
            logger.log(ERROR, "初始化超时: " + to_string(id-zhen) + " 帧 ");
        }
        Output(zhenId);
        Ok();
        fflush(stdout);
    }
    return 0;
}
