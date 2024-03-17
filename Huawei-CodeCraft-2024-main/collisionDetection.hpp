#pragma once
#include "const.h"
#include "graph.hpp"
/*
    实现碰撞检测和碰撞避免
    改进1：后续让携带货物的机器人先走，无货物的机器人停止，优先级：携带高价值货物的机器人>携带低价值货物的机器人>不携带货物的机器人
    改进2：考虑机器人性价比，携带货物价值/机器人到目的地距离（无论是到货品的距离还是到船舶的距离），越大，机器人优先级越高
*/
//机器人性价比
double priority_robot(Robot robotx){
    double priority=9.9;
    priority=robotx.goods/(robotx.path.size()-robotx.pid);
    return priority;
}
//通过遍历所有机器人检测与robot1可能相撞的所有情况，为避免碰撞，返回一个新的机器人排序数组sortedRobots[10]
void collisionAvoid(Robot &robot1,int sortedRobots[10]){
    for(int i;i<10;i++){
        sortedRobots[i]=i;
    }
    for(int i=0;i<10;i++){
        Robot robot2=robots[i];
        if(calcManhattanDist(robot1.x,robot1.y,robot2.x,robot2.y)==1){//两机器人紧挨着
            if(robot2.path[robot2.pid]==1&&robot1.y<robot2.y&&robot1.x==robot2.x&&robot1.robotId>robot2.robotId){
                printf("2 7 13 17");//碰撞类型：机器人2先左移一格，机器人1下一帧为上下左右任一动作时，均会相撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿一帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                }
                else{//robot2优先级高，两个机器人重排序
                    int temp_id=robot1.robotId;
                    sortedRobots[robot1.robotId]=robot2.robotId;
                    robot1.robotId=robot2.robotId;
                    sortedRobots[robot2.robotId]=temp_id;
                    robot2.robotId=temp_id;
                    //暂时两机器人紧挨着且相向移动的情况
                }
            }
            if(robot2.path[robot2.pid]==0&&robot1.y>robot2.y&&robot1.x==robot2.x&&robot1.robotId>robot2.robotId){
                printf("2 10 16 18");//碰撞类型：机器人2先右移一格，机器人1下一帧为上下左右任一动作时，均会相撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿一帧
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                }
                else{//robot2优先级高，两个机器人重排序
                    int temp_id=robot1.robotId;
                    sortedRobots[robot1.robotId]=robot2.robotId;
                    robot1.robotId=robot2.robotId;
                    sortedRobots[robot2.robotId]=temp_id;
                    robot2.robotId=temp_id;
                    //暂时两机器人紧挨着且相向移动的情况
                }
            }
            if(robot2.path[robot2.pid]==2&&robot1.y==robot2.y&&robot1.x<robot2.x&&robot1.robotId>robot2.robotId){
                printf("4 6 9 19");//碰撞类型：机器人2先上移一格，机器人1下一帧为上下左右任一动作时，均会相撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿一帧
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                }
                else{//robot2优先级高，两个机器人重排序
                    int temp_id=robot1.robotId;
                    sortedRobots[robot1.robotId]=robot2.robotId;
                    robot1.robotId=robot2.robotId;
                    sortedRobots[robot2.robotId]=temp_id;
                    robot2.robotId=temp_id;
                    //暂时两机器人紧挨着且相向移动的情况
                }
            }
            if(robot2.path[robot2.pid]==3&&robot1.y==robot2.y&&robot1.x>robot2.x&&robot1.robotId>robot2.robotId){
                printf("12 15 20 21");//碰撞类型：机器人2先下移一格，机器人1下一帧为上下左右任一动作时，均会相撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                }
                else{//robot2优先级高，两个机器人重排序
                    int temp_id=robot1.robotId;
                    sortedRobots[robot1.robotId]=robot2.robotId;
                    robot1.robotId=robot2.robotId;
                    sortedRobots[robot2.robotId]=temp_id;
                    robot2.robotId=temp_id;
                    //暂时两机器人紧挨着且相向移动的情况
                }
            }
            
        }
        if(calcManhattanDist(robot1.x,robot1.y,robot2.x,robot2.y)==2){//两机器人相距一格或两机器人呈对角
            if(robot1.path[robot1.pid]==0&&robot2.path[robot2.pid]==1&&robot1.y<robot2.y&&robot1.x==robot2.x){
                printf("1");//碰撞类型:相距一格，横向相对撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧，这么处理对不对？
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
                
            }
            if(robot1.path[robot1.pid]==3&&robot2.path[robot2.pid]==2&&robot1.x<robot2.x&&robot1.y==robot2.y){
                printf("3");//碰撞类型：相距一格，纵向相对撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
            }
            if(robot1.path[robot1.pid]==2&&robot2.path[robot2.pid]==1&&robot1.y<robot2.y&&robot1.x>robot2.x){
                printf("5");//碰撞类型：两机器人对角相撞，左上撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
            }
            if(robot1.path[robot1.pid]==2&&robot2.path[robot2.pid]==0&&robot1.y>robot2.y&&robot1.x>robot2.x){
                printf("8");//碰撞类型：两机器人对角相撞，右上撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
            }
            if(robot1.path[robot1.pid]==3&&robot2.path[robot2.pid]==1&&robot1.y<robot2.y&&robot1.x<robot2.x){
                printf("11");//碰撞类型：两机器人对角相撞，左下撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
            }
            if(robot1.path[robot1.pid]==3&&robot2.path[robot2.pid]==0&&robot1.y>robot2.y&&robot1.x<robot2.x){
                printf("14");//碰撞类型：两机器人对角相撞，右下撞
                if(priority_robot(robot1)>priority_robot(robot2)){//robot1优先级高，robot2停顿两帧
                    // robot2.path.erase(robot2.path.begin()+robot2.pid);
                    robot2.path.insert(robot2.path.begin()+robot2.pid,Direct::pause);
                    robot2.path.insert(robot2.path.begin()+robot2.pid+1,Direct::pause);
                }
                else{//robot2优先级高，robot1停顿两帧
                    // robot1.path.erase(robot1.path.begin()+robot1.pid);
                    robot1.path.insert(robot1.path.begin()+robot1.pid,Direct::pause);
                    robot1.path.insert(robot1.path.begin()+robot1.pid+1,Direct::pause);
                }
            }
        }
    }
}
