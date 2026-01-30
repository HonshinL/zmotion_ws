#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "zmcaux.h"

void line() {
    printf("----------------------------------------\n");
}

int main(int argc, char* argv[])
{
    // 1. IP 地址处理
    char ip[64] = "192.168.0.11";
    if (argc >= 2) {
        strncpy(ip, argv[1], sizeof(ip));
        ip[sizeof(ip)-1] = '\0';
    }

    printf("ZMotion 控制器诊断工具（兼容 SDK 2.1）\n");
    line();
    printf("目标 IP: %s\n", ip);

    // 2. 连接控制器
    ZMC_HANDLE handle = NULL;
    int ret = ZAux_OpenEth(ip, &handle);
    if (ret != ERR_OK) {
        printf("连接失败，错误码: %d\n", ret);
        return -1;
    }
    printf("连接成功！\n");
    line();

    // 3. 基本信息（ZAux_GetControllerInfo）
    char softType[128] = {0};
    char softVersion[128] = {0};
    char controllerId[128] = {0};

    ret = ZAux_GetControllerInfo(handle, softType, softVersion, controllerId);
    if (ret != ERR_OK) {
        printf("获取控制器信息失败，错误码: %d\n", ret);
        ZAux_Close(handle);
        return -1;
    }

    printf("硬件型号: %s\n", softType);
    printf("固件版本: %s\n", softVersion);
    printf("序列号:   %s\n", controllerId);
    line();

    // 4. 查询轴数量（旧固件用 AXISNUM?）
    char buf[128] = {0};
    ret = ZAux_DirectCommand(handle, "AXISNUM?", buf, sizeof(buf));
    if (ret == ERR_OK) {
        int axisCount = atoi(buf);
        printf("轴数量: %d\n", axisCount);
    } else {
        printf("轴数量查询失败，错误码: %d\n", ret);
    }
    line();

    // 5. 输入 IO 状态（IN(n)?）
    printf("输入 IO 状态:\n");
    for (int i = 0; i < 16; i++) {
        char cmd[32];
        sprintf(cmd, "IN(%d)?", i);
        memset(buf, 0, sizeof(buf));
        ret = ZAux_DirectCommand(handle, cmd, buf, sizeof(buf));
        if (ret == ERR_OK) {
            printf(" IN%-2d = %d\n", i, atoi(buf));
        }
    }
    line();

    // 6. 输出 IO 状态（OUT(n)?）
    printf("输出 IO 状态:\n");
    for (int i = 0; i < 16; i++) {
        char cmd[32];
        sprintf(cmd, "OUT(%d)?", i);
        memset(buf, 0, sizeof(buf));
        ret = ZAux_DirectCommand(handle, cmd, buf, sizeof(buf));
        if (ret == ERR_OK) {
            printf(" OUT%-2d = %d\n", i, atoi(buf));
        }
    }
    line();

    // 7. 扩展模块信息（旧固件不支持 SYSINFO，用 HELP 代替）
    printf("扩展模块信息（通过 HELP 推断）:\n");
    memset(buf, 0, sizeof(buf));
    ret = ZAux_DirectCommand(handle, "HELP", buf, sizeof(buf));
    if (ret == ERR_OK) {
        printf("%s\n", buf);
    } else {
        printf("无法获取扩展模块信息（固件过旧）\n");
    }
    line();

    // 8. 固件版本分析
    printf("固件版本分析:\n");
    int fw = atoi(softVersion);
    if (fw < 20200101) {
        printf(" - 固件非常老（%s），强烈建议升级。\n", softVersion);
        printf(" - 新固件将支持 VER、SYSINFO、更多运动功能。\n");
    } else if (fw < 20220101) {
        printf(" - 固件偏旧（%s），建议升级以获得更好兼容性。\n", softVersion);
    } else {
        printf(" - 固件较新，无需升级。\n");
    }
    line();

    ZAux_Close(handle);
    printf("诊断完成。\n");

    return 0;
}
