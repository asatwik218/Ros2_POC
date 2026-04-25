std_msgs/Header header
bool connected
bool enabled
bool operational
bool estopped

# Mode enum: 0=IDLE 1=CONNECTED 2=ENABLED 3=NRT_MODE 4=RT_MODE 5=FAULT
uint8 mode

# Mode constants
uint8 MODE_IDLE=0
uint8 MODE_CONNECTED=1
uint8 MODE_ENABLED=2
uint8 MODE_NRT=3
uint8 MODE_RT=4
uint8 MODE_FAULT=5
