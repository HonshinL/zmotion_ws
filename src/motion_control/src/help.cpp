#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "zmcaux.h"

int main(int argc, char* argv[])
{
    // 1. 处理 IP 地址
    char ip[64] = "192.168.0.11";   // 默认 IP，可自行修改
    if (argc >= 2) {
        strncpy(ip, argv[1], sizeof(ip));
        ip[sizeof(ip)-1] = '\0';
    }

    printf("ZMotion HELP 查询工具（SDK 2.1 兼容）\n");
    printf("目标 IP: %s\n", ip);
    printf("----------------------------------------\n");

    // 2. 打开连接
    ZMC_HANDLE handle = NULL;
    int ret = ZAux_OpenEth(ip, &handle);
    if (ret != ERR_OK) {
        printf("连接失败，错误码: %d\n", ret);
        return -1;
    }

    printf("连接成功！正在执行 HELP...\n");
    printf("----------------------------------------\n");

    // 3. 执行 HELP 指令
    char buffer[4096] = {0};   // HELP 输出可能较长
    ret = ZAux_DirectCommand(handle, "HELP", buffer, sizeof(buffer));

    if (ret == ERR_OK) {
        printf("HELP 输出:\n%s\n", buffer);
    } else {
        printf("HELP 查询失败，错误码: %d\n", ret);
    }

    printf("----------------------------------------\n");

    // 4. 关闭连接
    ZAux_Close(handle);
    printf("查询完成。\n");

    return 0;
}
