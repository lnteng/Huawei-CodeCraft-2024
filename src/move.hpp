/*
    格式化相关操作
    idx: 机器人 ID
    idb: 泊位 ID
*/

inline void right(int idx) {
    printf("move %d %d\n", idx, 0);
}

inline void left(int idx) {
    printf("move %d %d\n", idx, 1);
}

inline void upper(int idx) {
    printf("move %d %d\n", idx, 2);
}

inline void down(int idx) {
    printf("move %d %d\n", idx, 3);
}

inline void get(int idx) {
    printf("get %d\n", idx);
}

inline void pull(int idx) {
    printf("pull %d\n", idx);
}

inline void ship(int idx, int idb) {
    printf("ship %d %d\n", idx, idb);
}

inline void go(int idb) {
    printf("go %d\n", idb);
}

inline void ok(int idb) {
    printf("OK");
}