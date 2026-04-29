import streamlit as st
import pandas as pd
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

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ========== 安全缓冲区（仅用于路径规划，不绘制） ==========
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

def cubic_bezier(p0, p1, p2, p3, t):
    """三次贝塞尔曲线点"""
    mt = 1 - t
    x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
    y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
    return (x, y)

def smooth_curve(points, num_per_segment=30):
    """
    对一系列点（包含起点、中间关键点、终点）进行贝塞尔平滑
    每相邻两个点之间插入 num_per_segment 个插值点
    """
    if len(points) < 2:
        return points
    smooth = []
    for i in range(len(points) - 1):
        p0 = points[max(0, i-1)]
        p1 = points[i]
        p2 = points[i+1]
        p3 = points[min(len(points)-1, i+2)]
        # 贝塞尔曲线的控制点：取 p1 和 p2 作为端点，控制点使用邻近点使曲线更圆滑
        cp1 = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        cp2 = (p2[0] + p3[0]) / 2, (p2[1] + p3[1]) / 2
        for t in [j / num_per_segment for j in range(num_per_segment)]:
            pt = cubic_bezier(p1, cp1, cp2, p2, t)
            smooth.append(pt)
    smooth.append(points[-1])  # 添加终点
    return smooth

def plan_safe_path(start, end, obstacles, fly_mode):
    """
    避障规划：
    - 直飞最短：无视障碍物，直接直线。
    - 左侧绕飞：顺时针沿缓冲区边界（无平滑）。
    - 右侧绕飞：逆时针沿缓冲区边界（无平滑）。
    - 弧线最短航线：自动选择较短侧绕行，并生成平滑贝塞尔曲线。
    """
    # 直飞最短：完全无视障碍
    if fly_mode == "直飞最短":
        return [start, end]

    if not obstacles:
        if fly_mode == "弧线最短航线":
            # 无障碍时，贝塞尔曲线，控制点为中点
            cx, cy = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/20 for i in range(21)]]
        return [start, end]

    obs = obstacles[0]
    obs_poly = Polygon(obs["points"])
    buffered_poly = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])

    # 如果不与缓冲区相交，则无需绕飞
    if not direct_line.intersects(buffered_poly):
        if fly_mode == "弧线最短航线":
            cx, cy = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/20 for i in range(21)]]
        return [start, end]

    # --- 需要绕飞 ---
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

    # 获取缓冲区边界点（逆时针顺序）
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

    # 根据飞行模式选择绕行方向
    if fly_mode == "左侧绕飞":
        # 左侧绕飞 = 顺时针
        direction = 1
    elif fly_mode == "右侧绕飞":
        # 右侧绕飞 = 逆时针
        direction = -1
    else:  # 弧线最短航线：自动选择较短一侧
        len_cw = (x_idx - e_idx) % n       # 顺时针长度
        len_ccw = (e_idx - x_idx) % n      # 逆时针长度
        direction = 1 if len_cw <= len_ccw else -1

    # 收集绕飞路径点（不含起点终点）
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

    # 弧线模式：对完整路径（起点-绕飞点-终点）进行贝塞尔平滑
    if fly_mode == "弧线最短航线":
        full_path = [start] + bypass_pts + [end]
        # 平滑曲线（每段插值30个点）
        smoothed = smooth_curve(full_path, num_per_segment=30)
        return smoothed
    else:
        # 左右绕飞模式：返回折线（起点->入口->绕飞点->出口->终点）
        return [start] + bypass_pts + [end]

# ==================== 地图绘制（不绘制橙色缓冲区） ====================
def create_map(center_lng, center_lat, waypoints, home_point, land_point, obstacles, coord_system, temp_points):
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

    # 仅绘制原始障碍物（红色），不绘制缓冲区
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)

    # 航线绘制
    if len(waypoints) >= 2:
        safe_path = plan_safe_path(
            waypoints[0], waypoints[-1],
            obstacles,
            st.session_state.fly_mode
        )
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
        }.get(st.session_state.fly_mode, "blue")

        folium.PolyLine(route, color=color, weight=5, opacity=1, popup="无人机航线").add_to(m)

    if len(temp_points) >= 3:
        ps = [[lat, lng] for lng, lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 保存/加载 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "waypoints": st.session_state.waypoints,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points,
        "fly_mode": st.session_state.fly_mode
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None

# ==================== 初始化 ====================
if 'page' not in st.session_state:
    st.session_state.page = "飞行监控"

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
    "fly_mode": "左侧绕飞"
}

for k, v in defaults.items():
    if loaded and k in loaded:
        st.session_state[k] = loaded[k]
    elif k not in st.session_state:
        st.session_state[k] = v

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")
    page = st.radio("功能", ["📡 飞行监控", "🗺️ 航线规划"])
    st.session_state.page = page

    if "🗺️ 航线规划" in page:
        st.session_state.coord_system = st.selectbox(
            "坐标系", ["gcj02", "wgs84"], format_func=lambda x: "GCJ02(国内)" if x == "gcj02" else "WGS84(GPS)"
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

# ==================== 飞行监控页面 ====================
if "飞行监控" in st.session_state.page:
    st.header("📡 飞行监控（1秒精准心跳）")

    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False
        st.session_state.last_beat_time = time.time()

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳", use_container_width=True):
            st.session_state.running = True
            st.session_state.last_beat_time = time.time()
    with c2:
        if st.button("⏸️ 暂停心跳", use_container_width=True):
            st.session_state.running = False

    placeholder = st.empty()

    if st.session_state.running:
        now = time.time()
        if now - st.session_state.last_beat_time >= 1.0:
            st.session_state.seq += 1
            t = get_beijing_time_str()
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq, "时间": t, "状态": "在线正常"
            })
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
            st.session_state.last_beat_time = now

    with placeholder.container():
        df = pd.DataFrame(st.session_state.heartbeat_data)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=220)

    # 仅当心跳运行时才刷新（且仅在监控页面，不会影响地图页面）
    if st.session_state.running:
        time.sleep(0.05)
        st.rerun()

# ==================== 航线规划页面 ====================
else:
    st.header("🗺️ 航线规划（左右绕飞方向已修正 | 弧线平滑圆润 | 右侧绕飞无多余拐点）")
    st.success("✅ 左侧绕飞=顺时针 | 右侧绕飞=逆时针 | 弧线最短航线=贝塞尔平滑曲线")

    clng, clat = st.session_state.home_point
    map_container = st.empty()

    with map_container:
        m = create_map(
            clng, clat,
            st.session_state.waypoints,
            st.session_state.home_point,
            st.session_state.land_point,
            st.session_state.obstacles,
            st.session_state.coord_system,
            st.session_state.draw_points
        )
        o = st_folium(m, width=1100, height=680, key="MAP_NO_BUFFER3")

    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
