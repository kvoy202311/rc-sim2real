# Overall robot state published by FSM
std_msgs/Header header

uint8 FSM_WAITING  = 0
uint8 FSM_STANDING = 1
uint8 FSM_RUNNING  = 2
uint8 FSM_STANDUP  = 3
uint8 FSM_LIEDOWN  = 4
uint8 FSM_ESTOP    = 5

uint8 fsm_state
