# segment_sequencer_node -> tunnel_wall_detector_node(publish_mode="relay") 전용.
# 다음 웨이포인트의 예측 위치 근방(ROI)만 보도록 재무장을 요청한다 - "아치 전체 포인트에 평면
# 하나를 피팅하거나 우연히 보이는 가장 큰 평면을 무조건 목표로 선택하지 말라"는 요구사항을
# 만족하려면 검출기가 항상 "다음에 어디를 봐야 하는지" 힌트를 받아야 한다.

int32 segment_id                 # 단조증가 - 검출기는 LOCK 시 이 값을 SegmentLockEvent에 그대로 실어보냄
geometry_msgs/PointStamped roi_center  # base_frame 기준 예측 표면점(명목 기하, 실측 아님)
float64 roi_radius_m
