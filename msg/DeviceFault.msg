# 设备故障主动上报消息
std_msgs/Header header   # 时间戳
uint32 dev_type          # 设备类型（见 AutoMachineDef.msg 中 DEV_*）
uint8  device_index      # 设备索引（同一类设备的第几台，0/1...；非多机设备恒为 0）
uint32 fault_code        # 设备返回的原始故障码（通讯故障时 = 0）
bool   comm_fault        # true = 通讯故障（读不到设备/掉线）
