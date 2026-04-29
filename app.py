import streamlit as st
import pandas as pd
import threading
import time
import json
import os
import math
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 北京时间 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 辅助函数（您的航线规划逻辑） ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

SAFE_BUFFER = 0.00015

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

def cubic_bezier(p0, p1, p2, p3, t):
    mt = 1 - t
    x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
    y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
    return (x, y)

def smooth_curve(points, num_per_segment=30, offset_factor=0.3):
    if len(points) < 2:
        return points
    smooth = []
    for i in range(len(points) - 1):
        p0 = points[max(0, i-1)]
        p1 = points[i]
        p2 = points[i+1]
        p3 = points[min(len(points)-1, i+2)]
        cp1 = (p0[0] * offset_factor + p1[0] * (1 - offset_factor),
               p0[1] * offset_factor + p1[1] * (1 - offset_factor))
        cp2 = (p2[0] * offset_factor + p3[0] * (1 - offset_factor),
               p2[1] * offset_factor + p3[1] * (1 - offset_factor))
        for t in [j / num_per_segment for j in range(num_per_segment)]:
            pt = cubic_bezier(p1, cp1, cp2, p2, t)
            smooth.append(pt)
    smooth.append(points[-1])
    return smooth

def plan_safe_path(start, end, obstacles, fly_mode, swap_direction):
    if fly_mode == "直飞最短":
        return [start, end]

    if not obstacles:
        if fly_mode == "弧线最短航线":
            cx, cy = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            dx, dy = end[0] - start[0], end[1] - start[1]
            perp_x, perp_y = -dy, dx
            length = math.hypot(perp_x, perp_y)
            if length > 0:
                perp_x /= length
                perp_y /= length
            offset = 0.0002
            cx += perp_x * offset
            cy += perp_y * offset
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/30 for i in range(31)]]
        return [start, end]

    obs = obstacles[0]
    obs_poly = Polygon(obs["points"])
    buffered_poly = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])

    if not direct_line.intersects(buffered_poly):
        if fly_mode == "弧线最短航线":
            cx, cy = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            dx, dy = end[0] - start[0], end[1] - start[1]
            perp_x, perp_y = -dy, dx
            length = math.hypot(perp_x, perp_y)
            if length > 0:
                perp_x /= length
                perp_y /= length
            offset = 0.0002
            cx += perp_x * offset
            cy += perp_y * offset
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/30 for i in range(31)]]
        return [start, end]

    # 需要绕飞
    intersection = direct_line.intersection(buffered_poly.boundary)
    if intersection.geom_type == 'MultiPoint':
        pts = list(intersection.geoms)
    else:
        pts = [intersection]
    if len(pts) < 2:
        return [start, end]

    p1, p2 = pts[0], pts[1]
    d1 = math.hypot(p1.x - start[0], p1.y - start[1])
    d2 = math.hypot(p2.x - start[0], p2.y - start[1])
    if d1 < d2:
        entry, exit_pt = p1, p2
    else:
        entry, exit_pt = p2, p1

    boundary_pts = list(buffered_poly.exterior.coords)

    def find_nearest_idx(pt):
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(boundary_pts):
            d = math.hypot(p[0] - pt.x, p[1] - pt.y)
            if d < min_d:
                min_d = d
                idx = i
        return idx

    e_idx = find_nearest_idx(entry)
    x_idx = find_nearest_idx(exit_pt)
    n = len(boundary_pts)

    # 方向定义（支持交换）
    if swap_direction:
        left_dir = -1   # 逆时针
        right_dir = 1   # 顺时针
    else:
        left_dir = 1    # 顺时针
        right_dir = -1  # 逆时针

    if fly_mode == "左侧绕飞":
        direction = left_dir
    elif fly_mode == "右侧绕飞":
        direction = right_dir
    else:  # 弧线自动选短边
        len_cw = (x_idx - e_idx) % n
        len_ccw = (e_idx - x_idx) % n
        direction = 1 if len_cw <= len_ccw else -1

    bypass_pts = []
    i = e_idx
    if direction == 1:   # 顺时针
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i + 1) % n
        bypass_pts.append(boundary_pts[x_idx])
    else:                # 逆时针
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i - 1) % n
        bypass_pts.append(boundary_pts[x_idx])

    if fly_mode == "弧线最短航线":
        full_path = [start] + bypass_pts + [end]
        smoothed = smooth_curve(full_path, num_per_segment=30, offset_factor=0.2)
        return smoothed
    else:
        return [start] + bypass_pts + [end]

def create_map(center_lng, center_lat, waypoints, home_point, land_point, obstacles, coord_system, temp_points, fly_mode, swap_direction):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, control_scale=True, tiles=None)

    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    if home_point:
        h_lng, h_lat = home_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat, h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    if land_point:
        l_lng, l_lat = land_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker([l_lat, l_lng], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)

    if len(waypoints) >= 2:
        safe_path = plan_safe_path(waypoints[0], waypoints[-1], obstacles, fly_mode, swap_direction)
        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        color = {
            "直飞最短": "blue",
            "左侧绕飞": "#0066cc",
            "右侧绕飞": "#000000",
            "弧线最短航线": "#F79E02"
        }.get(fly_mode, "blue")

        folium.PolyLine(route, color=color, weight=5, opacity=1, popup="无人机航线").add_to(m)

    if len(temp_points) >= 3:
        ps = [[lat, lng] for lng, lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 状态持久化 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "waypoints": st.session_state.waypoints,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points,
        "fly_mode": st.session_state.fly_mode,
        "swap_direction": st.session_state.swap_direction
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None

# ==================== 初始化 session_state ====================
if "home_point" not in st.session_state:
    loaded = load_state()
    OFFICIAL_LNG = 118.749413
    OFFICIAL_LAT = 32.234097
    defaults = {
        "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
        "land_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
        "waypoints": [],
        "coord_system": "gcj02",
        "obstacles": [],
        "draw_points": [],
        "last_click": None,
        "fly_mode": "左侧绕飞",
        "swap_direction": False,
        "page": "飞行监控"   # 当前页面
    }
    for k, v in defaults.items():
        if loaded and k in loaded:
            st.session_state[k] = loaded[k]
        else:
            st.session_state[k] = v

# ==================== 侧边栏（页面切换 + 参数控制） ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")
    
    # 页面切换（不影响地图闪烁）
    st.session_state.page = st.radio("功能", ["📡 飞行监控", "🗺️ 航线规划"])
    
    # 以下控件仅在航线规划页面显示（避免监控页面出现无关控件）
    if st.session_state.page == "🗺️ 航线规划":
        st.session_state.coord_system = st.selectbox(
            "坐标系", ["gcj02", "wgs84"],
            format_func=lambda x: "GCJ02(国内)" if x == "gcj02" else "WGS84(GPS)"
        )
        st.subheader("🏠 起飞点")
        hlng = st.number_input("起飞经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("起飞纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("🚩 降落点")
        llng = st.number_input("降落经度", value=st.session_state.land_point[0], format="%.6f")
        llat = st.number_input("降落纬度", value=st.session_state.land_point[1], format="%.6f")
        if st.button("更新降落点"):
            st.session_state.land_point = (llng, llat)
            save_state()
            st.rerun()

        st.subheader("🛫 飞行策略")
        st.session_state.fly_mode = st.selectbox(
            "绕飞模式", ["直飞最短", "左侧绕飞", "右侧绕飞", "弧线最短航线"]
        )
        st.session_state.swap_direction = st.checkbox("交换左右方向", value=st.session_state.swap_direction)

        st.subheader("✈️ 航线")
        if st.button("生成航线"):
            st.session_state.waypoints = [st.session_state.home_point, st.session_state.land_point]
            save_state()
            st.rerun()
        if st.button("清空航线"):
            st.session_state.waypoints = []
            save_state()
            st.rerun()

        st.subheader("🚧 圈选障碍物")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        height = st.number_input("高度(m)", 1, 500, 25)
        name = st.text_input("名称", "教学楼")
        if st.button("✅ 保存障碍物"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({
                    "name": name, "height": height, "points": st.session_state.draw_points.copy()
                })
                st.session_state.draw_points = []
                save_state()
                st.success("保存成功")
                st.rerun()
            else:
                st.warning("至少3个点")
        if st.button("❌ 清空当前打点"):
            st.session_state.draw_points = []
            save_state()
            st.rerun()

        st.subheader("📋 已保存障碍物")
        obs_names = [f"{i+1}. {o['name']}" for i, o in enumerate(st.session_state.obstacles)]
        if obs_names:
            selected = st.selectbox("选择删除", obs_names)
            if st.button("删除选中"):
                idx = int(selected.split(".")[0]) - 1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()
        if st.button("🗑️ 清空所有障碍物"):
            st.session_state.obstacles = []
            save_state()
            st.rerun()

# ==================== 飞行监控页面（后台线程心跳，无闪烁） ====================
if st.session_state.page == "📡 飞行监控":
    st.header("📡 飞行监控（1秒精准心跳 · 无闪烁）")

    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False
        st.session_state.last_beat_time = time.time()

    # 后台线程函数
    def heartbeat_worker():
        while st.session_state.running:
            now = time.time()
            if now - st.session_state.last_beat_time >= 1.0:
                st.session_state.seq += 1
                t = get_beijing_time_str()
                st.session_state.heartbeat_data.append({
                    "序号": st.session_state.seq,
                    "时间": t,
                    "状态": "在线正常"
                })
                if len(st.session_state.heartbeat_data) > 60:
                    st.session_state.heartbeat_data.pop(0)
                st.session_state.last_beat_time = now
            time.sleep(0.2)

    # 控制线程
    if "heartbeat_thread" not in st.session_state:
        st.session_state.heartbeat_thread = None

    col1, col2 = st.columns(2)
    with col1:
        if st.button("▶️ 开始心跳", use_container_width=True):
            if not st.session_state.running:
                st.session_state.running = True
                st.session_state.last_beat_time = time.time()
                if st.session_state.heartbeat_thread is None or not st.session_state.heartbeat_thread.is_alive():
                    st.session_state.heartbeat_thread = threading.Thread(target=heartbeat_worker, daemon=True)
                    st.session_state.heartbeat_thread.start()
    with col2:
        if st.button("⏸️ 暂停心跳", use_container_width=True):
            st.session_state.running = False

    # 局部刷新区域（使用 st.empty 循环，不会导致整个页面重绘）
    placeholder = st.empty()
    while st.session_state.running:
        df = pd.DataFrame(st.session_state.heartbeat_data)
        with placeholder.container():
            if not df.empty:
                st.line_chart(df, x="时间", y="序号", color="#ff4560")
                st.dataframe(df, use_container_width=True, height=220)
            else:
                st.info("等待心跳数据...")
        time.sleep(0.5)  # 每0.5秒刷新一次显示（不是心跳生成间隔）
    else:
        # 未运行时显示最后一次数据
        df = pd.DataFrame(st.session_state.heartbeat_data)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=220)
        else:
            st.info("点击「开始心跳」查看实时数据")

# ==================== 航线规划页面（地图独立，永不闪烁） ====================
else:
    st.header("🗺️ 航线规划（地图独立，心跳不影响）")
    st.success(f"✅ 当前模式：{st.session_state.fly_mode}  |  方向交换：{'是' if st.session_state.swap_direction else '否'}")

    clng, clat = st.session_state.home_point
    m = create_map(
        clng, clat,
        st.session_state.waypoints,
        st.session_state.home_point,
        st.session_state.land_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points,
        st.session_state.fly_mode,
        st.session_state.swap_direction
    )
    o = st_folium(m, width=1100, height=680, key="main_map")

    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
