std_msgs/Header header
int32 class_id
float32 confidence
float32 depth          # LiDAR로 측정된 실제 거리 (미터). 유효하지 않으면 -1.0
float32 angle          # 로봇 전방 기준 객체 각도 (라디안, 좌=양수, 우=음수)
float32 map_x          # 맵 프레임 기준 X 좌표 (미터). map_valid=false면 0.0
float32 map_y          # 맵 프레임 기준 Y 좌표 (미터). map_valid=false면 0.0
bool depth_valid       # LiDAR 깊이 측정 성공 여부
bool map_valid         # SLAM 맵 좌표 변환 성공 여부
