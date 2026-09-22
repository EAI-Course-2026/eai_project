"""将归一化输入映射到配置范围，无硬件操作。"""
import math


def ratio_to_position(ratio, minimum, maximum, direction=1):
    if not math.isfinite(ratio) or not 0 <= ratio <= 1:
        raise ValueError("输入必须是 0～1 的有限数值")
    if not 0 <= minimum < maximum <= 1023 or direction not in (-1, 1):
        raise ValueError("SCS215 范围应为 0..1023 内的递增区间，方向为 ±1")
    if direction == -1:
        ratio = 1 - ratio
    return round(minimum + ratio * (maximum - minimum))
