# v1.6.1 0x0201 机器人性能体系数据
uint8 robot_id # 机器人ID （1~7->红，101~107->蓝
uint16 current_hp # 血量

# v1.6.1 0x0202 实时功率热量数据
uint16 shooter_heat # 枪口热量

# 经下位机处理后的数据
bool team_color # 队伍颜色 0->红 1->蓝
bool is_attacked # 是否受到攻击 0->未受到攻击 1->受到攻击
