# tunnel_wall_detector_node(publish_mode="relay")가 LOCK 순간 딱 한 번 발행하는 이벤트.
# segment_sequencer_node가 segment_id로 "이전 구간의 지연 도착 메시지"를 걸러낸 뒤에만
# target_pose/plane_pose를 신뢰한다(2026-09-28 리뷰 지적사항: "이전 구간의 지연된 LOCK
# 메시지가 새 구간에 쓰이지 않도록 구간 ID나 동등한 구분 방법도 넣어줘").

int32 segment_id
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped plane_pose
