#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
/* 类型定义 ------------------------------------------------------------------*/
/*
 * @brief command id
 */
typedef enum
{
    GAME_STATE_CMD_ID           = 0x0001,       //比赛状态数据 1HZ
    GAME_RESULT_CMD_ID          = 0x0002,       //比赛结果数据，比赛结束后发送
    GAME_ROBOT_SURVIV_CMD_ID    = 0x0003,       //比赛机器人血量数据 1HZ
    DART_STATUS_CMD_ID          = 0x0004,       //飞镖发射状态,飞镖发射后发送
    ICRA_BUFF_ZONE_STATUS_CMD_ID= 0x0005,       //人工智能挑战赛加成与惩罚区状态,1Hz
    EVENT_DATA_CMD_ID           = 0x0101,       //场地事件数据，事件改变后发送
    REFEREE_WARNING_CMD_ID      = 0x0104,       //裁判警告数据，警告后发送
    DART_REMAINING_TIME_CMD_ID  = 0x0105,       //飞镖发射口倒计时,1Hz
    GAME_ROBOT_STATE_CMD_ID     = 0x0201,       //机器人状态数据，10HZ
    POWER_HEAT_DATA_CMD_ID      = 0x0202,       //实时功率热量数据，50HZ
    GAME_ROBOT_POS_CMD_ID       = 0x0203,       //机器人位置数据,10HZ
    BUFF_MUCK_CMD_ID            = 0x0204,       //机器人增益数据
    AERIAL_ENERGY_CMD_ID        = 0x0205,       //空中机器人能量状态数据,10HZ
    ROBOT_HURT_CMD_ID           = 0x0206,       //伤害状态数据
    SHOOT_DATA_CMD_ID           = 0x0207,       //实时射击数据
    BULLET_REMAINING_CMD_ID     = 0x0208,       //弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送,1HZ
    RFID_STATUS_CMD_ID          = 0x0209,       //机器人 RFID 状态,1Hz
    DART_CLIENT_CMD_ID          = 0x020A,       //飞镖机器人客户端指令数据,10Hz
    CLIENT_CUSTOM_DATA_CMD_ID   = 0x0301,       //机器人间交互数据，发送方触发发送,上限10HZ
} cmd_id_e;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

#pragma pack(push,1)
typedef struct     //比赛状态数据:0x0001 发送频率:1Hz
{
    uint8_t game_type : 4;  //0-3 bit：比赛类型
                            //• 1：RoboMaster 机甲大师超级对抗赛；
                            //• 2：RoboMaster 机甲大师高校单项赛；
                            //• 3：ICRA RoboMaster 高校人工智能挑战赛
                            //• 4: RoboMaster 机甲大师高校联盟赛 3V3 对抗
                            //• 5: RoboMaster 机甲大师高校联盟赛步兵对抗
                            

    uint8_t game_progress : 4;  //4-7 bit：当前比赛阶段
                                //• 0：未开始比赛；
                                //• 1：准备阶段；
                                //• 2：自检阶段；
                                //• 3：5s倒计时；
                                //• 4：对战中；
                                //• 5：比赛结算中

    uint16_t stage_remain_time; //当前阶段剩余时间，单位s
    uint64_t SyncTimeStamp;     //机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_state_t;

typedef struct     //比赛结果数据：0x0002。发送频率：比赛结束后发送
{
    uint8_t winner;         //0 平局 1 红方胜利 2 蓝方胜利
} ext_game_result_t;

typedef struct     //机器人血量数据：0x0003。发送频率：1Hz
{
    uint16_t ally_1_robot_HP;   //己方 1 号英雄机器人血量，若该机器人未上场或者被罚下，则血量为 0，下文同理
    uint16_t ally_2_robot_HP;   //己方 2 号工程机器人血量
    uint16_t ally_3_robot_HP;   //己方 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;   //己方 4 号步兵机器人血量
    uint16_t reserved;          //保留位
    uint16_t ally_7_robot_HP;   //己方 7 号哨兵机器人血量
    uint16_t ally_outpost_HP;   //己方前哨站血量
    uint16_t ally_base_HP;      //己方前哨站血量

} ext_game_robot_HP_t;




typedef struct     // 场地事件数据：0x0101。发送频率：事件改变后发送
{
/*
 *  0：未占领/未激活
 *  1：已占领/已激活
 *   bit 0-2：
 *   • bit 0：己方与资源区区不重叠的补给区占领状态，1 为已占领
 *   • bit 1：己方与资源区重叠的补给区占领状态，1 为已占领
 *   • bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
 *   bit 3-6：己方能量机关状态
 *   • bit 3-4：己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
 *   • bit 5-6：己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活 
 *   bit 7-8：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
 *   bit 9-10：己方梯形高地的占领状态，1 为已占领
 *   bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0）
 *   bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标
 * 
 *   bit 23-24：中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）
 *   bit 25-26：己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
 *   bit 27-28：己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
 *   bit 29：己方基地增益点的占领状态，1 为已占领
 *   bit 30-31：保留位
 */
    uint32_t event_type;
} ext_event_data_t;


typedef struct      //裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送  （没修改，以后打对抗赛再修改）
{
    uint8_t level;                  //警告等级：
    uint8_t foul_robot_id;          //犯规机器人 ID：1级以及5级警告时，机器人ID为0二三四级警告时，机器人ID为犯规机器人ID
} ext_referee_warning_t;

typedef struct      //飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz   没修改，以后打对抗赛再修改
{
    uint8_t dart_remaining_time;    //15s 倒计时
} ext_dart_remaining_time_t;

typedef struct     //比赛机器人状态：0x0201。发送频率：10Hz
{
    uint8_t robot_id;       //机器人ID：1：红方英雄机器人；2：红方工程机器人；3/4/5：红方步兵机器人；6：红方空中机器人；7：红方哨兵机器人；11：蓝方英雄机器人；12：蓝方工程机器人；13/14/15：蓝方步兵机器人；16：蓝方空中机器人；17：蓝方哨兵机器人。
    uint8_t robot_level;    //机器人等级：1：一级；2：二级；3：三级
    uint16_t remain_HP;     //机器人剩余血量
    uint16_t max_HP;        //机器人上限血量
    uint16_t shooter_cooling_rate;     //机器人枪口每秒冷却值
    uint16_t shooter_cooling_limit;    //机器人枪口热量上限
    uint16_t chassis_power_limit;               //机器人最大底盘功率， 单位 w

    /* 主控电源输出情况： */
    uint8_t mains_power_gimbal_output : 1;  //0 bit：gimbal口输出： 1为有24V输出，0为无24v输出；
    uint8_t mains_power_chassis_output : 1; //1 bit：chassis口输出：1为有24V输出，0为无24v输出；
    uint8_t mains_power_shooter_output : 1; //2 bit：shooter口输出：1为有24V输出，0为无24v输出；

    // uint16_t shooter_id1_17mm_speed_limit;      //机器人 1 号 17mm 枪口上限速度 单位 m/s
    // uint16_t shooter_id2_17mm_cooling_rate;     //机器人 2 号 17mm 枪口每秒冷却值
    // uint16_t shooter_id2_17mm_cooling_limit;    //机器人 2 号 17mm 枪口热量上限
    // uint16_t shooter_id2_17mm_speed_limit;      //机器人 2 号 17mm 枪口上限速度 单位 m/s
    // uint16_t shooter_id1_42mm_cooling_rate;     //机器人 42mm 枪口每秒冷却值
    // uint16_t shooter_id1_42mm_cooling_limit;    //机器人 42mm 枪口热量上限
    // uint16_t shooter_id1_42mm_speed_limit;      //机器人 42mm 枪口上限速度 单位 m/s
    
    /* 主控电源输出情况： */
    
} ext_game_robot_state_t;

typedef struct     // 实时功率热量数据：0x0202。发送频率：50Hz
{
    uint16_t  reserved1;          //保留位
    uint16_t  reserved2;          //保留位
    float     reserved3;            //保留位
    uint16_t chassis_power_buffer;  //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_id1_17mm_cooling_heat;         //17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;         //42mm 枪口热量
} ext_power_heat_data_t;

typedef struct     //机器人位置：0x0203。发送频率：10Hz
{
    float x;        //位置x坐标，单位m
    float y;        //位置y坐标，单位m
    float z;        //位置z坐标，单位m
    float yaw;      //位置枪口，单位度
} ext_game_robot_pos_t;

typedef struct     //机器人增益：0x0204。发送频率：状态改变后发送
{
    uint8_t recovery_buff; //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint16_t cooling_buff; //机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
    uint8_t defence_buff; //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;  //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint8_t remaining_energy;   // bit 0-6：机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x80。机器人初始能量视为 100%
                                // bit 0：在剩余能量≥125%时为 1，其余情况为 0
                                // bit 1：在剩余能量≥100%时为 1，其余情况为 0
                                // bit 2：在剩余能量≥50%时为 1，其余情况为 0
                                // bit 3：在剩余能量≥30%时为 1，其余情况为 0
                                // bit 4：在剩余能量≥15%时为 1，其余情况为 0
                                // bit 5：在剩余能量≥5%时为 1，其余情况为 0
                                // bit 6：在剩余能量≥1%时为 1，其余情况为 0
}ext_buff_musk_t;

typedef struct     //空中机器人能量状态：0x0205。发送频率：10Hz
{
    uint8_t attack_time;    //可攻击时间 单位 s。30s 递减至0
} aerial_robot_energy_t;

typedef struct     //伤害状态：0x0206。发送频率：伤害发生后发送
{
    uint8_t armor_id : 4;   // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t hurt_type : 4;  // bit 4-7：血量变化类型 0x0 装甲伤害扣血；0x1 模块掉线扣血；0x5 装甲撞击扣血
} ext_robot_hurt_t;

typedef struct     //实时射击信息：0x0207。发送频率：射击后发送
{
    uint8_t bullet_type;    //子弹类型: 1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;     //发射机构 ID：1：1 号 17mm 发射机构;  2：保留位;  3：42mm 发射机构
    uint8_t bullet_freq;    //子弹射频 单位 Hz
    float bullet_speed;     //子弹射速 单位 m/s
} ext_shoot_data_t;

typedef struct     //子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人以及哨兵机器人主控发送
{
    uint16_t bullet_remaining_num_17mm;     //17mm 子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;     //42mm 子弹剩余发射数目
    uint16_t coin_remaining_num;            //剩余金币数量
    uint16_t projectile_allowance_fortress; //堡垒增益点提供的储备 17mm 弹丸允许发弹量；该值与机器人是否实际占领堡垒无关
} ext_bullet_remaining_t;

typedef struct      //机器人 RFID 状态：0x0209。发送频率：1Hz  未修改，打对抗赛的时候再改
{
/*
 * bit 0：基地增益点 RFID 状态；
 * bit 1：高地增益点 RFID 状态；
 * bit 2：能量机关激活点 RFID 状态；
 * bit 3：飞坡增益点 RFID 状态；
 * bit 4：前哨岗增益点 RFID 状态；
 * bit 5：资源岛增益点 RFID 状态；
 * bit 6：补血点增益点 RFID 状态；
 * bit 7：工程机器人补血卡 RFID 状态；
 * bit 8-25：保留 
 * bit 26-31：人工智能挑战赛 F1-F6 RFID 状态；
 * RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不
  *   能获取对应的增益效果。
 */
    uint32_t rfid_status; 
    uint8_t rfid_status_2;
} ext_rfid_status_t;

typedef struct      //飞镖机器人客户端指令数据：0x020A。发送频率：10Hz 
{
    uint8_t dart_launch_opening_status;     //当前飞镖发射口的状态0：关闭；   1：正在开启或者关闭中;    2：已经开启
    uint8_t reserved;                       //保留位
    uint16_t target_change_time;           //切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;       //最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
} ext_dart_client_cmd_t;

///* 人机交互数据 */
//typedef struct
//{
//    uint16_t data_cmd_id;
//    uint16_t send_ID;
//    uint16_t receiver_ID;
//} ext_student_interactive_header_data_t;
//
//typedef struct     // 客户端自定义数据：0x0301。内容ID:0xD180。发送频率：上限10Hz
//{
//    float data1;            // 自定义浮点数据1
//    float data2;            // 自定义浮点数据2
//    float data3;            // 自定义浮点数据3
//    struct
//    {
//        uint8_t comm_sig:1;
//        uint8_t flag_1:1;
//        uint8_t flag_2:1;
//        uint8_t flag_3:1;
//        uint8_t flag_4:1;
//        uint8_t flag_5:1;
//        uint8_t flag_6:1;   //无效
//        uint8_t flag_7:1;   //无效
//    } masks;        // 自定义8位数据4 bit 0-5：分别控制客户端自定义数据显示面板上的六个指示灯，值为1时显示绿色，值为0是显示红色。
//} client_custom_data_t;
//
//typedef struct         //交互数据 机器人间通信：0x0301。内容 ID:0x0200~0x02FF.发送频率：上限 10Hz
//{
//    uint8_t data[100];          //数据段 n 需要小于 113
//} robot_interactive_data_t;
//
//typedef struct         //客户端自定义图形 机器人间通信：0x0301。数据的内容 ID 0x0100.发送频率：上限 10Hz
//{
//    uint8_t operate_tpye;       //图形操作 0:空操作，（什么都不画）1: 增加图形2:修改图形3:删除单个图形5:删除一个图层的图形6:删除所有图形
//    uint8_t graphic_tpye;       //0:空形（什么都不画）1:直线2:矩形3:正圆4:椭圆5:弧形6:文本（ASCII 字码）
//    uint8_t graphic_name[5];    //在删除，修改等操作的索引，对于增加操作，发送多条图形名相同的不同图形命令，绘制第一条命令的图形
//    uint8_t layer;              //图层数: 0~9图层间会相互覆盖，原则是图层数大的图形覆盖图层数小的图形,相同图层内,先画的图形覆盖后画的图形。
//    uint8_t color;              //0: 红蓝主色（对于红方，为红色；对于蓝方，为蓝色）红色RGB为0xFF4545;蓝色RGB为4592FF1;黄色RGB为FFEE452;绿色RGB为A9FD2D3;橙色RGB为FFA3084;紫红色RGB为F029F75:;粉色RGB为FF648E6;青色RGB为45FFF37;黑色RGB为0000008;白色RGB为FFFFFF
//    uint8_t width;              //图形的线宽，单位像素点，比赛显示屏幕分辨率为 1920*1080
//    uint16_t start_x;           //起点 x 坐标，范围为[0,1920)屏幕左下角为(0,0) 屏幕中央为(960,540)对于不同分辨率,客户端会安装比例缩放坐标适配1920*1080.
//    uint16_t start_y;           //起点 y 坐标，范围[0,1080)
//    uint16_t radius;            //对于文本图形为文本的字体大小对于正圆图形为半径对于文本图形，推荐字体大小与线宽比例为 10:1
//    uint16_t end_x;             //终点 x 坐标
//    uint16_t end_y;             //终点 y 坐标
//    int16_t start_angle;        //圆弧的起始角度，圆弧顺时针绘制，单位为度，范围[-180,180]
//    int16_t end_angle;          //圆弧的终止角度，单位为度，范围[-180,180]
//    uint8_t text_lenght;        //文本信息的长度，最大 30
//    uint8_t text[30];           //30 个文件字符长度，ASCII 字符
//} ext_client_graphic_draw_t;


/* 人机交互数据 0x0301，需结合下面其他结构体一起使用*/
typedef struct
{
    uint16_t data_cmd_id;       //数据段的内容 ID
    uint16_t sender_ID;         //发送者的 ID
    uint16_t receiver_ID;       //接收者的 ID
    // uint8_t user_data[111];    // 内容数据段 最大112个字节
} ext_student_interactive_header_data_t;

typedef struct          //交互数据 机器人间通信。内容 ID:0x0200~0x02FF.发送频率：上限 10Hz
{
    uint8_t data[111];          //数据段 n 需要小于 113
} robot_interactive_data_t;

typedef struct          //客户端删除图形。内容 ID:0x0100
{
    uint8_t operate_tpye;       //图形操作。0: 空操作； 1: 删除图层； 2: 删除所有；
    uint8_t layer;              //图层数：0~9
} ext_client_custom_graphic_delete_t;

typedef struct          //图形数据  ID:0x0101
{
    uint8_t graphic_name[3];    //在删除，修改等操作中，作为客户端的索引。
    uint32_t operate_tpye:3;    //图形操作：0：空操作；1：增加；2：修改；3：删除；
    uint32_t graphic_tpye:3;    //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；4：圆弧；6：整型数；7：字符；
    uint32_t layer:4;           //图层数，0~9
    uint32_t color:4;           //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
    uint32_t start_angle:9;     //起始角度，单位：°，范围[0,360]；
    uint32_t end_angle:9;       //终止角度，单位：°，范围[0,360]。
    uint32_t width:10;          //线宽；
    uint32_t start_x:11;        //起点 x 坐标；
    uint32_t start_y:11;        //起点 y 坐标。
    uint32_t radius:10;         //字体大小或者半径；
    uint32_t end_x:11;          //终点 x 坐标；
    uint32_t end_y:11;          //终点 y 坐标
} graphic_data_struct_t;



typedef struct  //字内容 ID : 0X0110
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;


#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define REFEREE_SYSTEM_HEADER_SOF   0xA5
#define REFEREE_SYSTEM_FIFO_SIZE    (512u)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void RefereeSystem_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
uint8_t RefereeSystem_GetRobotID(void);
ext_game_robot_state_t* RefereeSystem_RobotState_Pointer(void);
ext_power_heat_data_t* RefereeSystem_PowerHeatData_Pointer(void);
ext_game_state_t* Game_State_Pointer(void);
#endif  // REFEREE_SYSTEM_H

