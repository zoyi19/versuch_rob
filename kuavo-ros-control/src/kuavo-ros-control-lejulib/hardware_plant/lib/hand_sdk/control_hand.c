#include <stdio.h>
#include "hand_sdk.h"
#include <stdlib.h>

int8_t open_position[6] = {0, 0, 0, 0, 0, 0};
int8_t close_position[6] = {100, 100, 100, 100, 100, 100};
int8_t left_position[6] = {0, 0, 0, 0, 0, 0};
int8_t right_position[6] = {0, 0, 0, 0, 0, 0};

void controlHand(const char* hand) {
    int jointID;
    int joint_position;
    printf("请输入控制的关节的ID（1-6）：");
    printf("对应顺序为：1-大拇指关节 2-大拇指外展肌 3-食指关节 4-中指关节 5-无名指关节 6-小指关节\n");
    scanf("%d", &jointID);

    if (jointID >= 1 && jointID <= 6) {
        printf("正在控制%s的关节%d\n", hand, jointID);
    } else {
        printf("无效的关节ID，请输入1到6之间的数字。\n");
        return;
    }

    printf("请输入控制的关节的POSITION（0-100）：");
    scanf("%d", &joint_position);

    if (joint_position >= 0 && joint_position <= 100) {
        printf("正在控制%s的关节%d的位置为%d\n", hand, jointID, joint_position);
    } else {
        printf("无效的位置，请输入0到100之间的数字。\n");
        return;
    }

    if (strcmp(hand, "左手") == 0) {
        left_position[jointID - 1] = joint_position;
        printf("left_position: %d %d %d %d %d %d\n", left_position[0], left_position[1], left_position[2], left_position[3], left_position[4], left_position[5]);
        hand_sdk.send_position(hand_sdk.qiangnao_port[1], right_position);
        hand_sdk.send_position(hand_sdk.qiangnao_port[0], left_position);
    } else if (strcmp(hand, "右手") == 0) {
        right_position[jointID - 1] = joint_position;
        printf("right_position: %d %d %d %d %d %d\n", right_position[0], right_position[1], right_position[2], right_position[3], right_position[4], right_position[5]);
        hand_sdk.send_position(hand_sdk.qiangnao_port[1], right_position);
        hand_sdk.send_position(hand_sdk.qiangnao_port[0], left_position);
    }
    sleep(1);
}

int main()
{
    hand_sdk.init();

    int com_num = 2;

    int choice;
    while (1) {
        printf("\n菜单:\n");
        printf("1. 控制左手\n");
        printf("2. 控制右手\n");
        printf("3, 全部掌开\n");
        printf("4. 全部掌关\n");
        printf("5. 退出\n");
        printf("请选择一个选项：");
        scanf("%d", &choice);

        switch (choice) {
            case 1:
                controlHand("左手");
                break;
            case 2:
                controlHand("右手");
                break;
            case 3:
                hand_sdk.send_position(hand_sdk.qiangnao_port[1], open_position);
                hand_sdk.send_position(hand_sdk.qiangnao_port[0], open_position);
                break;
            case 4:
                hand_sdk.send_position(hand_sdk.qiangnao_port[1], close_position);
                hand_sdk.send_position(hand_sdk.qiangnao_port[0], close_position);
                break;
            case 5:
                printf("退出程序。\n");
                exit(0);
            default:
                printf("无效的选项，请选择1到3之间的数字。\n");
        }
    }

    for (uint8_t i = 0; i < com_num; i++)
    {
        hand_sdk.close_com(hand_sdk.qiangnao_port[i]);
        printf("Close COM port:%d\n", hand_sdk.qiangnao_port[i]);
    }
    return 0;
}
