# 单个检测结果
string class_name       # 类别名: "trash", "obstacle", "person"
float32 confidence      # 置信度 [0, 1]
int32[4] bbox           # [x, y, width, height] 像素坐标
