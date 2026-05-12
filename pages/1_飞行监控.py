import streamlit as st
import pandas as pd
import math
import random
import time
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from streamlit_autorefresh import st_autorefresh

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

def simplify_waypoints_by_distance(waypoints, max_points=12):
    """
    基于路径长度均匀采样航点，确保航点在地理上均匀分布。
    waypoints: [(lng,lat), ...]
    max_points: 最大显示航点数（包括起点和终点）
    """
    if len(waypoints) <= max_points:
        return waypoints
    # 计算累计距离
    distances = [0.0]
    for i in range(1, len(waypoints)):
        d = haversine(waypoints[i-1][1], waypoints[i-1][0], waypoints[i][1], waypoints[i][0])
        distances.append(distances[-1] + d)
    total = distances[-1]
    # 产生均匀间隔的距离值
    step = total / (max_points - 1)
    target_dists = [i * step for i in range(max_points)]
    # 插值找到对应点
    sampled = []
    j = 0
    for td in target_dists:
        while j < len(distances) - 1 and distances[j+1] < td:
            j += 1
        if j == len(distances) - 1:
            sampled.append(waypoints[-1])
        else:
            d0 = distances[j]
            d1 = distances[j+1]
            if d1 - d0 == 0:
                ratio = 0
            else:
                ratio = (td - d0) / (d1 - d0)
            p0 = waypoints[j]
            p1 = waypoints[j+1]
            lng = p0[0] + (p1[0] - p0[0]) * ratio
            lat = p0[1] + (p1[1] - p0[1]) * ratio
            sampled.append((lng, lat))
    return sampled

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

def create_flight_map(task, obstacles):
    if not task or not task["waypoints"]:
        m = folium.Map(location=[32.234097, 118.749413], zoom_start=16, control_scale=True)
    else:
        current_pos = get_current_position(task)
        if current_pos:
            center = [current_pos[1], current_pos[0]]
        else:
            center = [task["waypoints"][0][1], task["waypoints"][0][0]]
        m = folium.Map(location=center, zoom_start=18, control_scale=True)

    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 障碍物
    for ob in obstacles:
        points = [[p[1], p[0]] for p in ob['points']]
        folium.Polygon(locations=points, color='red', fill=True, fill_opacity=0.5, popup=ob['name']).add_to(m)

    # 完整航线（蓝色）
    waypoints = task["waypoints"]
    route = [[lat, lng] for lng, lat in waypoints]
    folium.PolyLine(route, color='blue', weight=4, opacity=0.8, popup="规划航线").add_to(m)

    # 基于距离均匀采样的航点（紫色圆点）
    display_pts = simplify_waypoints_by_distance(waypoints, max_points=12)
    for i, (lng, lat) in enumerate(display_pts):
        folium.CircleMarker(
            location=[lat, lng],
            radius=3,
            color='purple',
            fill=True,
            fill_color='purple',
            fill_opacity=0.8,
            popup=f"航点 {i+1}"
        ).add_to(m)

    # 已飞路径（绿色）
    flown_dist = task["flown_distance"]
    if flown_dist > 0:
        flown_points = []
        cum = 0.0
        segs = task["seg_distances"]
        for i, d in enumerate(segs):
            if cum + d <= flown_dist:
                flown_points.append(waypoints[i])
                flown_points.append(waypoints[i+1])
            else:
                ratio = (flown_dist - cum) / d if d > 0 else 1.0
                w1, w2 = waypoints[i], waypoints[i+1]
                interp = (w1[0] + (w2[0]-w1[0])*ratio, w1[1] + (w2[1]-w1[1])*ratio)
                flown_points.append(w1)
                flown_points.append(interp)
                break
            cum += d
        unique = []
        for p in flown_points:
            if not unique or (p[0] != unique[-1][0] or p[1] != unique[-1][1]):
                unique.append(p)
        if len(unique) >= 2:
            folium.PolyLine([[lat, lng] for lng, lat in unique], color='green', weight=4, opacity=0.9, popup="已飞路径").add_to(m)

    # 无人机当前位置（红色飞机）
    current_pos = get_current_position(task)
    if current_pos:
        folium.Marker(
            location=[current_pos[1], current_pos[0]],
            icon=folium.Icon(color='red', icon='plane', prefix='fa'),
            popup=f"当前位置 ({current_pos[0]:.6f}, {current_pos[1]:.6f})"
        ).add_to(m)

    # 起点和终点
    folium.Marker([waypoints[0][1], waypoints[0][0]], icon=folium.Icon(color='green', icon='play'), popup="起飞点").add_to(m)
    folium.Marker([waypoints[-1][1], waypoints[-1][0]], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 页面主体 ====================
st.header("🚁 飞行实时画面 - 任务执行监控")

if "flight_task" not in st.session_state:
    st.session_state.flight_task = None
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.running = True

waypoints = st.session_state.get("flight_waypoints", [])
obstacles = st.session_state.get("obstacles", [])

if st.session_state.flight_task is not None:
    if st.session_state.flight_task.get("waypoints") != waypoints:
        st.session_state.flight_task = None

if st.session_state.flight_task is None and waypoints and len(waypoints) >= 2:
    st.session_state.flight_task = init_flight_task(waypoints, speed=8.5)

col_btn, _ = st.columns([1, 5])
with col_btn:
    if st.button("🔄 重新加载航线", use_container_width=True):
        st.session_state.flight_task = None
        st.rerun()

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

st_autorefresh(interval=2000, key="flight_monitor")

if st.session_state.flight_task and st.session_state.flight_task.get("active"):
    update_flight_task(st.session_state.flight_task)

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
    battery = max(5, 100 - (task["flown_distance"] / task["total_distance"]) * 95)

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

    left_col, right_col = st.columns([2, 1])
    with left_col:
        st.subheader("🗺️ 实时飞行地图（每2秒刷新一次）")
        map_obj = create_flight_map(task, obstacles)
        st_folium(map_obj, width=700, height=500, key="moving_map")
    with right_col:
        st.subheader("📡 通信链路拓扑与数据流")
        st.success("✅ GCS 在线")
        st.success("✅ OBC 在线")
        st.success("✅ FCU 在线")
        st.caption("数据链路正常 • 心跳间隔1s（后台）")
        st.markdown("---")
        st.subheader("📍 航点列表（均匀采样）")
        display_pts = simplify_waypoints_by_distance(task["waypoints"], max_points=12)
        for i, (lng, lat) in enumerate(display_pts):
            if i == 0:
                st.caption(f"✈️ 起点 ({lng:.6f}, {lat:.6f})")
            elif i == len(display_pts)-1:
                st.caption(f"🏁 终点 ({lng:.6f}, {lat:.6f})")
            else:
                st.caption(f"🟣 航点 {i+1}: ({lng:.6f}, {lat:.6f})")
        if len(task["waypoints"]) > len(display_pts):
            st.caption(f"... 共 {len(task['waypoints'])} 个航点，仅显示均匀采样点")
else:
    st.info("暂无飞行航线，请前往「航线规划」页面绘制障碍物并生成航线")

# ==================== 心跳模拟器 ====================
st.markdown("---")
st.subheader("📶 无人机心跳模拟器（背景监控）")

col_h1, col_h2 = st.columns(2)
with col_h1:
    if st.button("⏸️ 暂停心跳", use_container_width=True):
        st.session_state.running = False
        st.rerun()
with col_h2:
    if st.button("▶️ 开始心跳", use_container_width=True):
        st.session_state.running = True
        st.rerun()

if st.session_state.running:
    st.session_state.seq += 1
    current_time = get_beijing_time_str()
    battery = round(random.uniform(70, 100), 2)
    signal = round(random.uniform(60, 95), 2)
    temperature = round(random.uniform(20, 45), 1)
    satellites = random.randint(6, 12)
    st.session_state.heartbeat_data.append({
        "序号": st.session_state.seq,
        "时间": current_time,
        "电池(%)": battery,
        "信号(%)": signal,
        "温度(°C)": temperature,
        "卫星数": satellites
    })
    if len(st.session_state.heartbeat_data) > 60:
        st.session_state.heartbeat_data.pop(0)

df = pd.DataFrame(st.session_state.heartbeat_data)
st.metric("累计心跳数", len(df))
if not df.empty:
    st.line_chart(df, x="时间", y="序号", color="#ff4560")
    st.dataframe(df, use_container_width=True, height=200)
else:
    st.info("心跳数据等待中...")
if st.session_state.running:
    st.success("✅ 心跳运行中（每秒更新）")
else:
    st.warning("⏸️ 心跳已暂停")
