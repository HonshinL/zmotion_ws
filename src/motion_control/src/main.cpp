#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "zmcaux.h"

void line() {
    printf("----------------------------------------\n");
}

// 测试一条指令
void test_cmd(ZMC_HANDLE handle, const char* cmd) {
    char buf[256] = {0};
    int ret = ZAux_DirectCommand(handle, cmd, buf, sizeof(buf));

    if (ret == ERR_OK) {
        printf("✔ %-12s  返回: %s\n", cmd, buf);
    } else if (ret == 2033) {
        printf("❌ %-12s  不支持 (2033)\n", cmd);
    } else {
        printf("⚠ %-12s  错误码: %d\n", cmd, ret);
    }
}

int main(int argc, char* argv[])
{
    // 1. IP 地址
    char ip[64] = "192.168.0.11";
    if (argc >= 2) {
        strncpy(ip, argv[1], sizeof(ip));
        ip[sizeof(ip)-1] = '\0';
    }

    printf("ZMotion 旧固件指令探测工具（SDK 2.1 兼容）\n");
    line();
    printf("目标 IP: %s\n", ip);

    // 2. 连接控制器
    ZMC_HANDLE handle = NULL;
    int ret = ZAux_OpenEth(ip, &handle);
    if (ret != ERR_OK) {
        printf("连接失败，错误码: %d\n", ret);
        return -1;
    }
    printf("连接成功！开始探测固件支持的指令...\n");
    line();

    // 3. 测试常见旧固件指令
    printf("系统类指令:\n");
    test_cmd(handle, "AXISNUM?");
    test_cmd(handle, "VER");
    test_cmd(handle, "VERSION");
    test_cmd(handle, "SYSINFO");
    test_cmd(handle, "HELP");
    line();

    printf("IO 指令:\n");
    test_cmd(handle, "IN(0)?");
    test_cmd(handle, "IN(1)?");
    test_cmd(handle, "OUT(0)?");
    test_cmd(handle, "OUT(1)?");
    line();

    printf("轴状态指令（测试轴 0）:\n");
    test_cmd(handle, "P(0)?");
    test_cmd(handle, "SPEED(0)?");
    test_cmd(handle, "STATE(0)?");
    test_cmd(handle, "ACCEL(0)?");
    test_cmd(handle, "DECEL(0)?");
    line();

    printf("运动指令（不会真正运动，只测试是否存在）:\n");
    test_cmd(handle, "MOVE(0,0)");
    test_cmd(handle, "JOG(0)");
    test_cmd(handle, "STOP(0)");
    line();

    ZAux_Close(handle);
    printf("探测完成。\n");

    return 0;
}
