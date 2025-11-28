uint32 group_id                 # 小群编号（0,1,2,...）
uint32[] members               # 小群内无人机的 global ID

geometry_msgs/PoseStamped[] formation_offsets  # 成员对应阵型偏移（必须与 members 顺序一致）

geometry_msgs/PoseStamped[] goals       # 小群目标点，可多个
