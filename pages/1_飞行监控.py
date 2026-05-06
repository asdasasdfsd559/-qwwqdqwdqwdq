import streamlit as st
import pandas as pd
import math
import random
import time
import json
from datetime import datetime, timezone, timedelta
import folium
from streamlit.components.v1 import html

st.set_page_config(page_title="飞行监控", layout="wide")

# ==================== 辅助函数 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

def haversine(lon1, lat1, lon2, lat2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def init_flight_task(waypoints, speed=8.5):
    if not waypoints or len(waypoints) < 2:
        return None
    seg_distances = []
    total = 0.0
    for i in range(len(waypoints)-1):
        d = haversine(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][1])
        seg_distances.append(d)
        total += d
    return {
        "waypoints": waypoints,
        "seg_distances": seg_distances,
        "total_distance": total,
        "speed": speed,
        "start_time": None,
        "active": False,
        "flown_distance": 0.0,
        "current_seg_idx": 0,
        "completion": 0.0
    }

def update_flight_task(task):
    if not task or not task["active"]:
        return
    elapsed = time.time() - task["start_time"]
    flown = task["speed"] * elapsed
    if flown >= task["total_distance"]:
        task["active"] = False
        task["flown_distance"] = task["total_distance"]
        task["completion"] = 1.0
        return
    task["flown_distance"] = flown
    task["completion"] = flown / task["total_distance"]
    cumulative = 0.0
    for i, d in enumerate(task["seg_distances"]):
        if cumulative + d >= flown:
            task["current_seg_idx"] = i
            return
        cumulative += d

def get_current_position(task):
    if not task:
        return None
    if task["completion"] == 0:
        return task["waypoints"][0]
    if task["completion"] >= 1.0:
        return task["waypoints"][-1]
    flown = task["flown_distance"]
    cumulative = 0.0
    for i, d in enumerate(task["seg_distances"]):
        if cumulative + d >= flown:
            ratio = (flown - cumulative) / d if d > 0 else 1.0
            w1 = task["waypoints"][i]
            w2 = task["waypoints"][i+1]
            lng = w1[0] + (w2[0] - w1[0]) * ratio
            lat = w1[1] + (w2[1] - w1[1]) * ratio
            return (lng, lat)
        cumulative += d
    return task["waypoints"][-1]

def build_static_map(waypoints, obstacles):
    """
    构建完整的地图HTML（包含规划航线、障碍物、起点终点）
    返回地图对象的 HTML 字符串和地图中心经纬度
    """
    if not waypoints or len(waypoints) < 2:
        # 默认位置
        center = [32.234097, 118.749413]
        zoom = 16
    else:
        center = [waypoints[0][1], waypoints[0][0]]
        zoom = 18
    m = folium.Map(location=center, zoom_start=zoom, control_scale=True)

    # 添加底图
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 添加障碍物（原始多边形，红色）
    for ob in obstacles:
        points = [[p[1], p[0]] for p in ob['points']]
        folium.Polygon(locations=points, color='red', fill=True, fill_opacity=0.5, popup=ob['name']).add_to(m)

    # 规划航线（蓝色）
    route = [[lat, lng] for lng, lat in waypoints]
    folium.PolyLine(route, color='blue', weight=4, opacity=0.8, popup="规划航线").add_to(m)

    # 起点和终点
    folium.Marker([waypoints[0][1], waypoints[0][0]], icon=folium.Icon(color='green', icon='play'), popup="起飞点").add_to(m)
    folium.Marker([waypoints[-1][1], waypoints[-1][0]], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    folium.LayerControl().add_to(m)
    # 将地图转换为HTML字符串
    map_html = m.get_root().render()
    return map_html, (center[0], center[1])

# ==================== 页面主体 ====================
st.header("🚁 飞行实时画面 - 任务执行监控")

# 初始化 session_state
if "flight_task" not in st.session_state:
    st.session_state.flight_task = None
if "static_map_html" not in st.session_state:
    st.session_state.static_map_html = None
if "map_center" not in st.session_state:
    st.session_state.map_center = None

# 获取航点和障碍物（从航线规划传递过来）
waypoints = st.session_state.get("flight_waypoints", [])
obstacles = st.session_state.get("obstacles", [])   # 从航线规划中获取障碍物列表

# 更新飞行任务
if st.session_state.flight_task is None:
    if waypoints and len(waypoints) >= 2:
        st.session_state.flight_task = init_flight_task(waypoints, speed=8.5)
elif st.session_state.flight_task and st.session_state.flight_task.get("waypoints") != waypoints:
    st.session_state.flight_task = init_flight_task(waypoints, speed=8.5)

# 生成静态地图（只生成一次）
if st.session_state.static_map_html is None and waypoints and len(waypoints) >= 2:
    map_html, center = build_static_map(waypoints, obstacles)
    st.session_state.static_map_html = map_html
    st.session_state.map_center = center

# 控制按钮
col1, col2 = st.columns(2)
with col1:
    if st.button("▶️ 开始任务", use_container_width=True):
        if st.session_state.flight_task and len(st.session_state.flight_task["waypoints"]) >= 2:
            st.session_state.flight_task["active"] = True
            st.session_state.flight_task["start_time"] = time.time()
            st.session_state.flight_task["flown_distance"] = 0.0
            st.session_state.flight_task["current_seg_idx"] = 0
            st.session_state.flight_task["completion"] = 0.0
            st.rerun()
        else:
            st.warning("请先在「航线规划」页面生成飞行航线")
with col2:
    if st.button("⏹️ 停止任务", use_container_width=True):
        if st.session_state.flight_task:
            st.session_state.flight_task["active"] = False
            st.rerun()

# 每秒刷新页面（只用于更新飞行指标和获取飞机位置）
# 但地图 HTML 不会重新生成，所以地图不会闪烁
st_autorefresh = __import__('streamlit_autorefresh').st_autorefresh
# 为了避免导入错误，使用 try-except
try:
    from streamlit_autorefresh import st_autorefresh
    st_autorefresh(interval=1000, key="flight_data")
except:
    time.sleep(1)
    st.rerun()

# 更新飞行任务状态
if st.session_state.flight_task and st.session_state.flight_task.get("active"):
    update_flight_task(st.session_state.flight_task)

# 显示飞行指标
task = st.session_state.flight_task
if task and task.get("waypoints"):
    total_wp = len(task["waypoints"])
    current_seg = min(task["current_seg_idx"] + 1, total_wp)
    progress = task["completion"] * 100
    elapsed = time.time() - task["start_time"] if task.get("active") and task.get("start_time") else 0
    if not task.get("active") and task["completion"] >= 1.0:
        elapsed = task["total_distance"] / task["speed"]
    remaining_dist = max(0, task["total_distance"] - task["flown_distance"])
    remaining_time = remaining_dist / task["speed"] if task["speed"] > 0 else 0
    battery = max(5, 100 - progress * 0.95)

    def fmt_time(sec):
        m = int(sec // 60)
        s = int(sec % 60)
        return f"{m:02d}:{s:02d}"
    elapsed_str = fmt_time(elapsed)
    eta_str = fmt_time(remaining_time)

    st.markdown("---")
    mcol1, mcol2, mcol3 = st.columns(3)
    with mcol1:
        st.metric("📌 当前航点", f"{current_seg} / {total_wp}")
        st.metric("⚡ 飞行速度", f"{task['speed']:.1f} m/s")
    with mcol2:
        st.metric("⏱️ 已用时间", elapsed_str)
        st.metric("📏 剩余距离", f"{remaining_dist:.0f} m")
    with mcol3:
        st.metric("🕒 预计到达", eta_str)
        st.metric("🔋 电量模拟", f"{battery:.1f}%")
    st.progress(progress/100.0, text=f"飞行进度 {progress:.1f}%")
    st.markdown("---")

    # 左右布局：地图（左） + 通信链路（右）
    left_col, right_col = st.columns([2, 1])
    with left_col:
        st.subheader("🗺️ 实时飞行地图（飞机移动，地图不闪烁）")
        # 获取当前飞机位置
        current_pos = get_current_position(task)
        if current_pos and st.session_state.static_map_html:
。
            st.warning("由于技术限制，实时飞机移动会导致地图轻微闪烁。如需要完全不闪烁，请改用静态地图（飞机位置用数字显示）。")
    with right_col:
        st.subheader("📡 通信链路拓扑与数据流")
        st.success("✅ GCS 在线")
        st.success("✅ OBC 在线")
        st.success("✅ FCU 在线")
        st.caption("数据链路正常 • 心跳间隔1s")
else:
    st.info("暂无飞行航线，请前往「航线规划」页面绘制障碍物并生成航线")

# ==================== 心跳模拟器 ====================
# ... (保持不变)
