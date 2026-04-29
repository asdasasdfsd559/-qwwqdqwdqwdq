import streamlit as st
import pandas as pd
import json
import os
import math
from datetime import datetime
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="无人机绕飞航线生成器", layout="wide")

# ==================== 辅助函数 ====================
class CoordTransform:
    """简单坐标系转换（模拟高德GCJ02）"""
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ------------------ 几何处理 ------------------
def point_in_polygon(point, polygon):
    """判断点是否在多边形内部"""
    return polygon.contains(Point(point))

def line_intersects_polygon(line, polygon):
    """线段是否与多边形相交（包含边界接触）"""
    return line.intersects(polygon)

def buffer_polygon(polygon, distance):
    """生成安全缓冲区（向外扩展）"""
    return polygon.buffer(distance)

def get_intersection_points(line, polygon_boundary):
    """返回直线与多边形边界的所有交点"""
    inter = line.intersection(polygon_boundary)
    if inter.is_empty:
        return []
    if inter.geom_type == 'Point':
        return [inter]
    if inter.geom_type == 'MultiPoint':
        return list(inter.geoms)
    # 如果相交结果是一条线段，取两端点（极少出现）
    if inter.geom_type == 'LineString':
        return [Point(inter.coords[0]), Point(inter.coords[-1])]
    return []

def nearest_point_on_boundary(pt, boundary_coords):
    """找到边界上离给定点最近的点索引"""
    min_dist = float('inf')
    idx = 0
    for i, (x, y) in enumerate(boundary_coords):
        d = math.hypot(x - pt.x, y - pt.y)
        if d < min_dist:
            min_dist = d
            idx = i
    return idx

def generate_arc_path(points, num_segments=50):
    """
    对关键点序列进行三次贝塞尔曲线平滑，并采样为多段线。
    points: 起点 -> 一系列边界点 -> 终点
    """
    if len(points) < 2:
        return points
    # 使用 Catmull-Rom 样条插值（简单易用）
    smooth = []
    for i in range(len(points) - 1):
        p0 = points[max(0, i-1)]
        p1 = points[i]
        p2 = points[i+1]
        p3 = points[min(len(points)-1, i+2)]
        # 每段插值若干个点
        for t in [j / num_segments for j in range(1, num_segments+1)]:
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            smooth.append((x, y))
    smooth.append(points[-1])
    # 去重（浮点误差）
    unique = []
    for pt in smooth:
        if not unique or math.hypot(pt[0]-unique[-1][0], pt[1]-unique[-1][1]) > 1e-9:
            unique.append(pt)
    return unique

# ------------------ 核心绕飞规划 ------------------
def plan_detour_path(start, end, obstacle_poly, buffer_dist=0.00015, mode="左侧绕飞"):
    """
    规划绕飞路径
    参数：
        start, end: 起点终点坐标 (lng, lat)
        obstacle_poly: Shapely Polygon 对象（原始障碍物）
        buffer_dist: 安全缓冲区距离（经纬度度，约15米）
        mode: "左侧绕飞", "右侧绕飞", "弧线最短"
    返回：
        路径点列表 [(lng,lat), ...] 或 None（无解时返回直线）
    """
    # 生成缓冲区多边形
    buffered = buffer_polygon(obstacle_poly, buffer_dist)
    direct_line = LineString([start, end])

    # 检测原始直线是否安全
    if not line_intersects_polygon(direct_line, buffered):
        # 直线无冲突，直接返回直线（两点即可）
        return [start, end]

    # 直线与缓冲区相交 -> 需要绕飞
    # 计算直线与缓冲区边界的交点
    boundary = buffered.boundary
    pts = get_intersection_points(direct_line, boundary)
    if len(pts) < 2:
        # 退化情况，返回直线
        return [start, end]

    # 确定入口（离起点近）和出口（离终点近）
    d1 = math.hypot(pts[0].x - start[0], pts[0].y - start[1])
    d2 = math.hypot(pts[1].x - start[0], pts[1].y - start[1])
    if d1 < d2:
        entry, exit_pt = pts[0], pts[1]
    else:
        entry, exit_pt = pts[1], pts[0]

    # 获取缓冲区边界坐标（逆时针顺序）
    boundary_coords = list(buffered.exterior.coords)
    n = len(boundary_coords)

    # 找到入口和出口在边界上的最近点索引
    e_idx = nearest_point_on_boundary(entry, boundary_coords)
    x_idx = nearest_point_on_boundary(exit_pt, boundary_coords)

    # 计算顺时针和逆时针方向的长度
    clockwise_len = (x_idx - e_idx) % n
    counter_len = (e_idx - x_idx) % n

    # 根据模式选择方向
    if mode == "左侧绕飞":
        # 定义左侧绕飞 = 顺时针方向（索引递减）
        direction = 'clockwise'
    elif mode == "右侧绕飞":
        # 右侧绕飞 = 逆时针方向（索引递增）
        direction = 'counterclockwise'
    else:  # 弧线最短
        direction = 'clockwise' if clockwise_len <= counter_len else 'counterclockwise'

    # 收集绕飞边界点
    bypass = []
    i = e_idx
    if direction == 'clockwise':   # 顺时针（索引递减）
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i - 1) % n
        bypass.append(boundary_coords[x_idx])
    else:                           # 逆时针（索引递增）
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i + 1) % n
        bypass.append(boundary_coords[x_idx])

    # 构建核心折线路径：起点 -> 入口 -> 绕行点 -> 出口 -> 终点
    core_path = [start] + bypass + [end]

    # 如果选择弧线模式，对核心路径进行平滑（仍输出多段线）
    if mode == "弧线最短":
        final_path = generate_arc_path(core_path, num_segments=40)
        return final_path
    else:
        return core_path

# ==================== 地图绘制 ====================
def create_map(center_lng, center_lat, home_point, land_point, obstacles, temp_points, fly_mode, buffer_dist=0.00015):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, control_scale=True)

    # 底图
    folium.TileLayer(tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
                     attr='高德-街道', name='街道图').add_to(m)
    folium.TileLayer(tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
                     attr='高德-卫星', name='卫星图').add_to(m)

    # 起点/终点
    if home_point:
        folium.Marker([home_point[1], home_point[0]], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    if land_point:
        folium.Marker([land_point[1], land_point[0]], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    # 障碍物和安全缓冲区
    for ob in obstacles:
        poly = Polygon(ob['points'])
        buff = buffer_polygon(poly, buffer_dist)
        # 原始障碍物（红色）
        pts_orig = [[p[1], p[0]] for p in ob['points']]
        folium.Polygon(locations=pts_orig, color='red', fill=True, fill_opacity=0.5, popup=ob['name']).add_to(m)
        # 缓冲区（橙色）
        buff_coords = [[p[1], p[0]] for p in buff.exterior.coords]
        folium.Polygon(locations=buff_coords, color='orange', fill=True, fill_opacity=0.2, weight=1).add_to(m)

    # 航线生成
    if home_point and land_point and obstacles:
        # 使用第一个障碍物（简化示例）
        obs_poly = Polygon(obstacles[0]['points'])
        path = plan_detour_path(home_point, land_point, obs_poly, buffer_dist, fly_mode)
        # 转换坐标（如果需要，这里假设高德图）
        route = [[p[1], p[0]] for p in path]
        color = {"左侧绕飞": "blue", "右侧绕飞": "black", "弧线最短": "orange"}.get(fly_mode, "blue")
        folium.PolyLine(route, color=color, weight=5, opacity=1, popup=f"航线模式: {fly_mode}").add_to(m)

    # 临时打点（用于圈选障碍物）
    if len(temp_points) >= 3:
        pts = [[p[1], p[0]] for p in temp_points]
        folium.Polygon(locations=pts, color='purple', weight=2, fill_opacity=0.3).add_to(m)
    for p in temp_points:
        folium.CircleMarker([p[1], p[0]], radius=4, color='purple', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 状态管理 ====================
STATE_FILE = "flight_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "obstacles": st.session_state.obstacles,
        "temp_points": st.session_state.temp_points,
        "fly_mode": st.session_state.fly_mode
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            return json.load(f)
    return None

# ==================== 界面 ====================
if "home_point" not in st.session_state:
    loaded = load_state()
    OFFICIAL_LNG, OFFICIAL_LAT = 118.749413, 32.234097
    st.session_state.home_point = (OFFICIAL_LNG, OFFICIAL_LAT)
    st.session_state.land_point = (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005)
    st.session_state.obstacles = []
    st.session_state.temp_points = []
    st.session_state.last_click = None
    st.session_state.fly_mode = "左侧绕飞"
    if loaded:
        for k, v in loaded.items():
            st.session_state[k] = v

st.title("无人机绕飞航线生成器")
st.caption("点击地图打点 → 保存障碍物 → 选择绕飞模式 → 生成航线")

with st.sidebar:
    st.header("设置")
    st.session_state.fly_mode = st.selectbox("绕飞模式", ["左侧绕飞", "右侧绕飞", "弧线最短"])
    st.divider()
    st.subheader("起飞点 / 降落点")
    home_lng = st.number_input("起飞经度", value=st.session_state.home_point[0], format="%.6f")
    home_lat = st.number_input("起飞纬度", value=st.session_state.home_point[1], format="".6f)
    if st.button("更新起飞点"):
        st.session_state.home_point = (home_lng, home_lat)
        save_state()
        st.rerun()

    land_lng = st.number_input("降落经度", value=st.session_state.land_point[0], format="%.6f")
    land_lat = st.number_input("降落纬度", value=st.session_state.land_point[1], format="%.6f")
    if st.button("更新降落点"):
        st.session_state.land_point = (land_lng, land_lat)
        save_state()
        st.rerun()

    st.divider()
    st.subheader("障碍物绘制")
    st.write(f"当前打点: {len(st.session_state.temp_points)}")
    name = st.text_input("障碍物名称", "大楼")
    if st.button("✅ 保存当前打点为障碍物"):
        if len(st.session_state.temp_points) >= 3:
            st.session_state.obstacles.append({
                "name": name,
                "points": st.session_state.temp_points.copy()
            })
            st.session_state.temp_points = []
            save_state()
            st.success(f"障碍物 {name} 已保存")
            st.rerun()
        else:
            st.warning("至少需要 3 个点")
    if st.button("❌ 清空当前打点"):
        st.session_state.temp_points = []
        save_state()
        st.rerun()

    st.subheader("已保存障碍物")
    for i, ob in enumerate(st.session_state.obstacles):
        col1, col2 = st.columns([3, 1])
        col1.write(f"{i+1}. {ob['name']} ({len(ob['points'])}点)")
        if col2.button("删除", key=f"del_{i}"):
            st.session_state.obstacles.pop(i)
            save_state()
            st.rerun()
    if st.button("清空所有障碍物"):
        st.session_state.obstacles = []
        save_state()
        st.rerun()

# 主地图区域
st.header("规划地图")
map_center = st.session_state.home_point
m = create_map(
    center_lng=map_center[0], center_lat=map_center[1],
    home_point=st.session_state.home_point,
    land_point=st.session_state.land_point,
    obstacles=st.session_state.obstacles,
    temp_points=st.session_state.temp_points,
    fly_mode=st.session_state.fly_mode,
    buffer_dist=0.00015
)

output = st_folium(m, width=1100, height=680, key="plan_map")

# 处理地图点击
if output and output.get("last_clicked"):
    lat = output["last_clicked"]["lat"]
    lng = output["last_clicked"]["lng"]
    pt = (round(lng, 6), round(lat, 6))
    if st.session_state.last_click != pt:
        st.session_state.last_click = pt
        st.session_state.temp_points.append(pt)
        save_state()
        st.rerun()
