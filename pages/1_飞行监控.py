import streamlit as st
import pandas as pd
import math
import random
import time
import threading
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium

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
    """更新飞行任务状态（非线程安全，需要在主线程调用）"""
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

def create_static_map(waypoints):
    """生成静态规划航线地图（不刷新）"""
    if not waypoints or len(waypoints) < 2:
        m = folium.Map(location=[32.234097, 118.749413], zoom_start=16, control_scale=True)
        folium.TileLayer(
            tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
            attr='高德-街道', name='街道图'
        ).add_to(m)
        folium.TileLayer(
            tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德-卫星', name='卫星图'
        ).add_to(m)
        folium.LayerControl().add_to(m)
        return m
    # 使用第一个航点作为中心
    center = [waypoints[0][1], waypoints[0][0]]
    m = folium.Map(location=center, zoom_start=18, control_scale=True)
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)
    route = [[lat, lng] for lng, lat in waypoints]
    folium.PolyLine(route, color='blue', weight=4, opacity=0.8, popup="规划航线").add_to(m)
    # 起点终点
    folium.Marker([waypoints[0][1], waypoints[0][0]], icon=folium.Icon(color='green', icon='play'), popup="起飞点").add_to(m)
    folium.Marker([waypoints[-1][1], waypoints[-1][0]], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)
    folium.LayerControl().add_to(m)
    return m

# ==================== 页面主体 ====================
st.header("🚁 飞行实时画面 - 任务执行监控")

# 初始化 session_state
if "flight_task" not in st.session_state:
    st.session_state.flight_task = None
if "static_map" not in st.session_state:
    st.session_state.static_map = None
if "update_placeholder" not in st.session_state:
    st.session_state.update_placeholder = None
if "running_update" not in st.session_state:
    st.session_state.running_update = False

# 获取航点
waypoints = st.session_state.get("flight_waypoints", [])

# 初始化或更新飞行任务
if st.session_state.flight_task is None:
    if waypoints and len(waypoints) >= 2:
        st.session_state.flight_task = init_flight_task(waypoints, speed=8.5)
        # 生成静态地图（仅在任务初始化时生成一次，之后不再刷新）
        st.session_state.static_map = create_static_map(waypoints)
elif st.session_state.flight_task and st.session_state.flight_task.get("waypoints") != waypoints:
    st.session_state.flight_task = init_flight_task(waypoints, speed=8.5)
    st.session_state.static_map = create_static_map(waypoints)

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
            st.session_state.running_update = True
            st.rerun()
        else:
            st.warning("请先在「航线规划」页面生成飞行航线")
with col2:
    if st.button("⏹️ 停止任务", use_container_width=True):
        if st.session_state.flight_task:
            st.session_state.flight_task["active"] = False
            st.session_state.running_update = False
            st.rerun()

# 创建占位符用于动态更新数据（不刷新整个页面）
if st.session_state.update_placeholder is None:
    st.session_state.update_placeholder = st.empty()

# 更新飞行任务状态（仅当任务激活时）
if st.session_state.flight_task and st.session_state.flight_task["active"]:
    update_flight_task(st.session_state.flight_task)
    # 每秒刷新一次数据（通过 st.rerun() 实现，但地图不会重建，因为地图已保存）
    # 注意：我们不会重建地图，所以地图区域不会闪烁
    time.sleep(1)
    st.rerun()

# 显示静态地图（仅生成一次）
if st.session_state.static_map:
    with st.session_state.update_placeholder.container():
        # 显示飞行指标
        task = st.session_state.flight_task
        if task:
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

            col_a, col_b, col_c = st.columns(3)
            with col_a:
                st.metric("📌 当前航点", f"{current_seg} / {total_wp}")
                st.metric("⚡ 飞行速度", f"{task['speed']:.1f} m/s")
            with col_b:
                st.metric("⏱️ 已用时间", elapsed_str)
                st.metric("📏 剩余距离", f"{remaining_dist:.0f} m")
            with col_c:
                st.metric("🕒 预计到达", eta_str)
                st.metric("🔋 电量模拟", f"{battery:.1f}%")
            st.progress(progress/100.0, text=f"飞行进度 {progress:.1f}%")
        else:
            st.info("等待任务开始")
else:
    st.info("暂无飞行航线，请前往「航线规划」页面绘制障碍物并生成航线")

# 显示地图（独立于动态更新区域）
st.subheader("🗺️ 规划航线地图")
if st.session_state.static_map:
    st_folium(st.session_state.static_map, width=1100, height=500, key="static_flight_map")
else:
    st.info("请先在航线规划页面生成航线")

# 通信链路拓扑（右侧，但这里放在地图下方，因为地图占宽）
st.subheader("📡 通信链路拓扑与数据流")
link_cols = st.columns(3)
link_cols[0].success("✅ GCS 在线")
link_cols[1].success("✅ OBC 在线")
link_cols[2].success("✅ FCU 在线")
st.caption("数据链路正常 • 心跳间隔1s")

# 心跳模拟器（略... 可保留或移除）
