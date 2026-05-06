import streamlit as st
import pandas as pd
import time
import math
import random
from datetime import datetime, timezone, timedelta
from streamlit_autorefresh import st_autorefresh

st.set_page_config(page_title="飞行监控", layout="wide")

# 北京时间
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ------------------ 辅助函数：计算两点间距离（米）采用haversine公式 ------------------
def haversine(lon1, lat1, lon2, lat2):
    R = 6371000  # 地球半径（米）
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ------------------ 飞行任务模拟 ------------------
def init_flight(waypoints, speed=8.5):
    """初始化飞行任务，计算各段距离和总距离"""
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
        "active": False
    }

def update_flight_progress(flight_state):
    """根据当前时间更新飞行进度，返回 (当前航点索引, 当前航段进度比例, 已飞距离, 已用时间, 剩余距离, 预计剩余时间)"""
    if not flight_state or not flight_state["active"]:
        return None
    elapsed = time.time() - flight_state["start_time"]
    flown = flight_state["speed"] * elapsed
    total = flight_state["total_distance"]
    if flown >= total:
        # 任务完成
        flight_state["active"] = False
        return (len(flight_state["waypoints"])-1, 1.0, total, elapsed, 0.0, 0.0)
    # 找到当前航段
    cum = 0.0
    seg_idx = 0
    for i, d in enumerate(flight_state["seg_distances"]):
        if cum + d >= flown:
            seg_idx = i
            break
        cum += d
    seg_progress = (flown - cum) / flight_state["seg_distances"][seg_idx] if flight_state["seg_distances"][seg_idx] > 0 else 1.0
    remaining = total - flown
    eta = remaining / flight_state["speed"]
    return (seg_idx, seg_progress, flown, elapsed, remaining, eta)

# 初始化 session_state 中的飞行任务
if "flight_task" not in st.session_state:
    st.session_state.flight_task = None

st.header("🚁 飞行实时画面 - 任务执行监控")

# 获取航线规划中保存的航点
waypoints = st.session_state.get("flight_waypoints", [])
if waypoints and len(waypoints) >= 2 and (st.session_state.flight_task is None or st.session_state.flight_task["waypoints"] != waypoints):
    # 如果航点更新了，重新初始化任务（但不自动激活）
    st.session_state.flight_task = init_flight(waypoints, speed=8.5)

col1, col2 = st.columns([1, 3])
with col1:
    if st.button("▶️ 开始任务", use_container_width=True):
        if st.session_state.flight_task and len(st.session_state.flight_task["waypoints"]) >= 2:
            st.session_state.flight_task["active"] = True
            st.session_state.flight_task["start_time"] = time.time()
            st.rerun()
        else:
            st.warning("请先在「航线规划」页面生成飞行航线")
    if st.button("⏹️ 停止任务", use_container_width=True):
        if st.session_state.flight_task:
            st.session_state.flight_task["active"] = False
            st.rerun()

# 每秒自动刷新页面（用于更新实时数据）
st_autorefresh(interval=1000, key="flight_monitor")

# 显示实时监控数据
if st.session_state.flight_task:
    task = st.session_state.flight_task
    progress_info = update_flight_progress(task)
    if progress_info:
        seg_idx, seg_progress, flown, elapsed, remaining, eta = progress_info
        total = task["total_distance"]
        # 计算当前航点
        current_wp = seg_idx + 1
        total_wp = len(task["waypoints"])
        # 格式化时间
        def format_time(sec):
            m = int(sec // 60)
            s = int(sec % 60)
            return f"{m:02d}:{s:02d}"
        elapsed_str = format_time(elapsed)
        eta_str = format_time(eta) if eta > 0 else "00:00"
        # 电量模拟（根据飞行比例下降，最低 5%）
        battery = max(5, 100 - (flown / total) * 95)
        # 进度百分比
        progress_pct = (flown / total) * 100 if total > 0 else 0

        # 布局显示
        col_a, col_b, col_c = st.columns(3)
        with col_a:
            st.metric("📌 当前航点", f"{current_wp} / {total_wp}")
            st.metric("⚡ 飞行速度", f"{task['speed']:.1f} m/s")
        with col_b:
            st.metric("⏱️ 已用时间", elapsed_str)
            st.metric("📏 剩余距离", f"{remaining:.0f} m")
        with col_c:
            st.metric("🕒 预计到达", eta_str)
            st.metric("🔋 电量模拟", f"{battery:.1f}%", delta=None)

        # 进度条
        st.progress(progress_pct / 100.0, text=f"飞行进度 {progress_pct:.1f}%")

        # 通信链路拓扑
        st.subheader("📡 通信链路拓扑与数据流")
        link_cols = st.columns(3)
        link_cols[0].success("✅ GCS 在线")
        link_cols[1].success("✅ OBC 在线")
        link_cols[2].success("✅ FCU 在线")

        # 显示当前航点坐标（可选）
        with st.expander("📍 航点坐标详情"):
            st.write("起飞点 → 降落点路径点（经纬度）")
            for i, wp in enumerate(task["waypoints"]):
                st.caption(f"航点 {i}: ({wp[0]:.6f}, {wp[1]:.6f})")
    else:
        if not task["active"]:
            st.info("飞行任务未启动，点击「开始任务」开始模拟飞行")
else:
    st.info("暂无飞行航线，请前往「航线规划」页面绘制障碍物并生成航线")

# ------------------ 保留原有的心跳模拟模块（可选） ------------------
st.markdown("---")
st.subheader("📶 无人机心跳模拟器（背景监控）")

if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.running = True

col_h1, col_h2 = st.columns(2)
with col_h1:
    if st.button("⏸️ 暂停心跳", use_container_width=True):
        st.session_state.running = False
        st.rerun()
with col_h2:
    if st.button("▶️ 开始心跳", use_container_width=True):
        st.session_state.running = True
        st.rerun()

# 心跳自动刷新（与飞行监控共用同一个 autorefresh，但数据独立）
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
