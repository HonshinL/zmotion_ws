#include <stdio.h>
#include <string.h>
#include "zmcaux.h"   // 正运动 SDK 头文件

int main(int argc, char* argv[])
{
    // 1. 处理 IP 地址
    char ip[64] = "192.168.0.11";   // 默认 IP，可根据需要修改
    if (argc >= 2) {
        strncpy(ip, argv[1], sizeof(ip));
        ip[sizeof(ip)-1] = '\0';
    }

    printf("尝试连接控制器: %s\n", ip);

    // 2. 打开连接
    ZMC_HANDLE handle = NULL;
    int ret = ZAux_OpenEth(ip, &handle);
    if (ret != ERR_OK) {
        printf("连接失败，错误码: %d\n", ret);
        return -1;
    }

    printf("连接成功！\n");

    // 3. 查询控制器信息
    char softType[128] = {0};
    char softVersion[128] = {0};
    char controllerId[128] = {0};

    ret = ZAux_GetControllerInfo(handle, softType, softVersion, controllerId);
    if (ret != ERR_OK) {
        printf("ZAux_GetControllerInfo 查询失败，错误码: %d\n", ret);
        ZAux_Close(handle);
        return -1;
    }

    // 4. 打印结果
    printf("\n=== 控制器信息 ===\n");
    printf("硬件型号 (SoftType): %s\n", softType);
    printf("固件版本 (SoftVersion): %s\n", softVersion);
    printf("控制器序列号 (ID): %s\n", controllerId);
    printf("===================\n\n");

    // 5. 关闭连接
    ZAux_Close(handle);
    return 0;
}
