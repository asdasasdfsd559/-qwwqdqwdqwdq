import streamlit as st
import pandas as pd
import random
import math
from datetime import datetime, timezone, timedelta
from streamlit_autorefresh import st_autorefresh
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon, LinearRing

# ==================== 坐标转换（复用航线规划的逻辑） ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 安全缓冲区（复用航线规划的逻辑） ====================
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

def get_polyline_around_path(start, end, obstacle, fly_mode):
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    vertices = list(buffered.exterior.coords)
    
    start_pt = Point(start)
    end_pt = Point(end)
    
    def get_nearest_point(pt, points):
        min_dist = float('inf')
        nearest = None
        for p in points:
            dist = pt.distance(Point(p))
            if dist < min_dist:
                min_dist = dist
                nearest = p
        return nearest
    
    start_nearest = get_nearest_point(start_pt, vertices)
    end_nearest = get_nearest_point(end_pt, vertices)
    
    start_idx = vertices.index(start_nearest)
    end_idx = vertices.index(end_nearest)
    
    if fly_mode == "左侧绕飞":
        if start_idx < end_idx:
            polyline_points = vertices[start_idx:end_idx+1]
        else:
            polyline_points = vertices[start_idx:] + vertices[:end_idx+1]
    else:
        if start_idx > end_idx:
            polyline_points = vertices[end_idx:start_idx+1][::-1]
        else:
            polyline_points = (vertices[end_idx:] + vertices[:start_idx+1])[::-1]
    
    unique_points = []
    prev = None
    for p in polyline_points:
        if prev is None or not math.isclose(p[0], prev[0]) or not math.isclose(p[1], prev[1]):
            unique_points.append(p)
            prev = p
    return unique_points

def get_smooth_around_path(start, end, obstacle):
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    ring = LinearRing(buffered.exterior.coords)
    
    start_pt = Point(start)
    end_pt = Point(end)
    start_dist = ring.project(start_pt)
    end_dist = ring.project(end_pt)
    
    ring_len = ring.length
    cw_dist = (end_dist - start_dist) % ring_len
    ccw_dist = (start_dist - end_dist) % ring_len
    
    if cw_dist <= ccw_dist:
        num_points = 50
        distances = [start_dist + i * cw_dist / num_points for i in range(num_points + 1)]
        distances = [d % ring_len for d in distances]
    else:
        num_points = 50
        distances = [start_dist - i * ccw_dist / num_points for i in range(num_points + 1)]
        distances = [d % ring_len for d in distances]
    
    around_points = []
    for d in distances:
        p = ring.interpolate(d)
        around_points.append((p.x, p.y))
    return around_points

def plan_safe_path(start, end, obstacles, fly_mode):
    if not obstacles:
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            perp_x = -dy * 0.0004
            perp_y = dx * 0.0004
            ctrl_x = mid_x + perp_x
            ctrl_y = mid_y + perp_y
            
            bezier_points = []
            for t in [i/100 for i in range(101)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl_x + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl_y + t**2 * end[1]
                bezier_points.append((x, y))
            return bezier_points
        return [start, end]
    
    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])
    
    if not direct_line.intersects(buffered):
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            perp_x = -dy * 0.0004
            perp_y = dx * 0.0004
            ctrl_x = mid_x + perp_x
            ctrl_y = mid_y + perp_y
            
            bezier_points = []
            for t in [i/100 for i in range(101)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl_x + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl_y + t**2 * end[1]
                bezier_points.append((x, y))
            return bezier_points
        return [start, end]
    
    if fly_mode == "左侧绕飞":
        polyline_points = get_polyline_around_path(start, end, obstacle, "左侧绕飞")
        full_path = [start] + polyline_points + [end]
        return full_path
    
    elif fly_mode == "右侧绕飞":
        polyline_points = get_polyline_around_path(start, end, obstacle, "右侧绕飞")
        full_path = [start] + polyline_points + [end]
        return full_path
    
    elif fly_mode == "弧线最短航线":
        around_points = get_smooth_around_path(start, end, obstacle)
        full_path = [start] + around_points + [end]
        return global_bezier_smooth(full_path)
    else:
        return [start, end]

def global_bezier_smooth(points):
    if len(points) < 3:
        return points
    num_ctrl = len(points) - 1
    ctrl_pts = []
    for i in range(num_ctrl):
        t = i / num_ctrl
        ctrl_pts.append((
            (1-t)*points[i][0] + t*points[i+1][0],
            (1-t)*points[i][1] + t*points[i+1][1]
        ))
    bezier = []
    for t in [i/100 for i in range(101)]:
        x = 0
        y = 0
        n = len(ctrl_pts) - 1
        for i in range(len(ctrl_pts)):
            coeff = math.comb(n, i) * (1-t)**(n-i) * t**i
            x += coeff * ctrl_pts[i][0]
            y += coeff * ctrl_pts[i][1]
        bezier.append((x, y))
    return bezier

# ==================== 地图绘制 ====================
def create_map(center_lng, center_lat, waypoints, home_point, land_point, obstacles, coord_system, temp_points, fly_mode, uav_pos):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=st.session_state.get("zoom", 19),
                   control_scale=True, tiles=None)

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

    # 绘制障碍物
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)

    # 绘制航线
    if len(waypoints) >= 2:
        safe_path = plan_safe_path(waypoints[0], waypoints[-1], obstacles, fly_mode)
        st.session_state.safe_path = safe_path  # 保存路径用于无人机移动
        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        color = {
            "直飞最短": "blue",
            "左侧绕飞": "#0066cc",
            "右侧绕飞": "#000000",
            "弧线最短航线": "#28a745"
        }.get(fly_mode, "#28a745")
        
        folium.PolyLine(route, color=color, weight=5, opacity=1, popup="无人机航线").add_to(m)

    # 绘制无人机当前位置
    if uav_pos:
        u_lng, u_lat = uav_pos if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*uav_pos)
        folium.Marker(
            [u_lat, u_lng], 
            icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
            popup="无人机当前位置"
        ).add_to(m)

    if len(temp_points) >= 3:
        ps = [[lat, lng] for lng, lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 时间工具 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 页面配置 ====================
st.set_page_config(page_title="飞行监控", layout="wide")

# ==================== 状态初始化 ====================
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.running = True
    # 无人机飞行状态
    OFFICIAL_LNG, OFFICIAL_LAT = 118.749413, 32.234097
    st.session_state.uav_pos = st.session_state.get("home_point", (OFFICIAL_LNG, OFFICIAL_LAT))
    st.session_state.path_idx = 0
    st.session_state.last_battery = 96.0

# 初始化默认的航线规划状态（如果用户还没去过航线规划页面）
if "waypoints" not in st.session_state:
    OFFICIAL_LNG, OFFICIAL_LAT = 118.749413, 32.234097
    st.session_state.waypoints = []
    st.session_state.home_point = (OFFICIAL_LNG, OFFICIAL_LAT)
    st.session_state.land_point = (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005)
    st.session_state.coord_system = "gcj02"
    st.session_state.obstacles = []
    st.session_state.fly_mode = "弧线最短航线"
    st.session_state.draw_points = []

# ==================== 页面标题 ====================
st.header("📡 飞行监控（自动每秒心跳，可暂停）")

# ==================== 控制按钮 ====================
col1, col2 = st.columns(2)
with col1:
    if st.button("⏸️ 暂停心跳", use_container_width=True):
        st.session_state.running = False
        st.rerun()
with col2:
    if st.button("▶️ 开始心跳", use_container_width=True):
        st.session_state.running = True
        st.rerun()

# ==================== 自动刷新 ====================
st_autorefresh(interval=1000, key="heartbeat_auto")

# ==================== 数据更新（运行时） ====================
current_battery = st.session_state.last_battery
current_signal = 90
current_temp = 28
current_sat = 10
safe_path = st.session_state.get("safe_path", [])
path_idx = st.session_state.path_idx

if st.session_state.running:
    st.session_state.seq += 1
    current_time = get_beijing_time_str()
    
    # 更新心跳数据
    current_battery = round(max(0, st.session_state.last_battery - random.uniform(0.01, 0.03)), 2)
    current_signal = round(random.uniform(75, 95), 2)
    current_temp = round(random.uniform(22, 38), 1)
    current_sat = random.randint(8, 12)
    st.session_state.last_battery = current_battery

    # 更新无人机位置（沿航线移动）
    if len(st.session_state.waypoints) >= 2 and len(safe_path) > 0:
        if path_idx < len(safe_path):
            st.session_state.uav_pos = safe_path[path_idx]
            st.session_state.path_idx += 1
            path_idx = st.session_state.path_idx

    # 保存心跳数据
    st.session_state.heartbeat_data.append({
        "序号": st.session_state.seq,
        "时间": current_time,
        "电池(%)": current_battery,
        "信号(%)": current_signal,
        "温度(°C)": current_temp,
        "卫星数": current_sat
    })

    # 只保留最近60条
    if len(st.session_state.heartbeat_data) > 60:
        st.session_state.heartbeat_data.pop(0)

# ==================== 顶部任务监控指标 ====================
st.divider()
st.subheader("✈️ 飞行实时画面 - 任务执行监控")
cols = st.columns(5)
total_waypoints = len(st.session_state.waypoints)
current_waypoint = min(path_idx + 1, total_waypoints) if total_waypoints > 0 else 0
elapsed_time = path_idx
remaining_distance = max(0, (len(safe_path) - path_idx) * 8) if safe_path else 0

with cols[0]:
    st.metric("当前航点", f"{current_waypoint}/{total_waypoints}" if total_waypoints>0 else "--/--")
with cols[1]:
    st.metric("飞行速度", "8.5 m/s")
with cols[2]:
    st.metric("已用时间", f"{elapsed_time//60:02d}:{elapsed_time%60:02d}")
with cols[3]:
    st.metric("剩余距离", f"{remaining_distance:.1f} m" if remaining_distance>0 else "0 m")
with cols[4]:
    st.metric("剩余电量", f"{current_battery:.1f}%")

# ==================== 地图与通信状态 ====================
col_map, col_comm = st.columns([3, 1])

with col_map:
    # 检查是否有航线
    if len(st.session_state.waypoints) == 0:
        st.info("💡 请先前往【航线规划】页面完成航线规划后，即可查看实时飞行轨迹")
    
    # 绘制地图
    center = st.session_state.uav_pos
    m = create_map(
        center[0], center[1],
        st.session_state.waypoints,
        st.session_state.home_point,
        st.session_state.land_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points,
        st.session_state.fly_mode,
        st.session_state.uav_pos
    )
    st_folium(m, width=1100, height=500, key="monitor_map")

with col_comm:
    st.subheader("通信链路拓扑与数据流")
    # 4G链路
    with st.container(border=True):
        st.markdown("#### 📶 4G链路")
        st.success("✅ 正常")
        st.metric("信号强度", f"{current_signal}%")
    
    # 数传链路
    with st.container(border=True):
        st.markdown("#### 📡 数传链路")
        st.success("✅ 正常")
        st.metric("传输速率", "2.4 Mbps")
    
    # 图传链路
    with st.container(border=True):
        st.markdown("#### 🎥 图传链路")
        st.success("✅ 正常")
        st.metric("视频分辨率", "1080P")
    
    # GPS定位
    with st.container(border=True):
        st.markdown("#### 🛰️ GPS定位")
        st.success("✅ 正常")
        st.metric("卫星数量", f"{current_sat} 颗")

# ==================== 心跳数据监控 ====================
st.divider()
st.subheader("💓 设备心跳数据")
df = pd.DataFrame(st.session_state.heartbeat_data)
st.metric("累计心跳数", len(df))
if not df.empty:
    st.line_chart(df, x="时间", y=["电池(%)", "信号(%)"], color=["#ff4560", "#0066cc"])
    st.dataframe(df, use_container_width=True, height=300)
else:
    st.info("点击「开始心跳」开始生成数据")

# 运行状态
if st.session_state.running:
    st.success("✅ 心跳运行中（每秒更新）")
else:
    st.warning("⏸️ 心跳已暂停")
