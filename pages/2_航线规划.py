import streamlit as st
import json
import os
import math
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="无人机绕飞航线生成器", layout="wide")

# ==================== 坐标转换（用于高德地图） ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 几何工具函数 ====================
def point_in_polygon(point, polygon):
    """判断点是否在多边形内部（不包括边界）"""
    return polygon.contains(Point(point))

def line_intersects_polygon(line, polygon):
    """线段是否与多边形相交（包含边界接触）"""
    return line.intersects(polygon)

def get_intersection_points(line, polygon_boundary):
    """返回直线与多边形边界的所有交点"""
    inter = line.intersection(polygon_boundary)
    if inter.is_empty:
        return []
    if inter.geom_type == 'Point':
        return [inter]
    if inter.geom_type == 'MultiPoint':
        return list(inter.geoms)
    # 如果相交部分是一条线段，取两端点
    if inter.geom_type == 'LineString':
        return [Point(inter.coords[0]), Point(inter.coords[-1])]
    return []

def nearest_point_on_boundary(pt, boundary_coords):
    """找到多边形边界上离给定点最近的点索引"""
    min_dist = float('inf')
    idx = 0
    for i, (x, y) in enumerate(boundary_coords):
        d = math.hypot(x - pt.x, y - pt.y)
        if d < min_dist:
            min_dist = d
            idx = i
    return idx

def arc_smooth(points, num_per_segment=40):
    """
    Catmull-Rom 样条插值，输出密集的多段线（模拟平滑圆弧）
    """
    if len(points) < 2:
        return points
    smooth = []
    for i in range(len(points) - 1):
        p0 = points[max(0, i-1)]
        p1 = points[i]
        p2 = points[i+1]
        p3 = points[min(len(points)-1, i+2)]
        for t in [j / num_per_segment for j in range(1, num_per_segment+1)]:
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
    # 去除重复点
    uniq = []
    for pt in smooth:
        if not uniq or math.hypot(pt[0]-uniq[-1][0], pt[1]-uniq[-1][1]) > 1e-9:
            uniq.append(pt)
    return uniq

# ==================== 核心绕飞规划 ====================
def plan_detour_path(start, end, obstacle_poly, mode="左侧绕飞"):
    """
    规划绕飞路径，严格禁止进入障碍物内部
    start, end : (lng, lat)
    obstacle_poly : Shapely Polygon（原始障碍物）
    mode : "左侧绕飞", "右侧绕飞", "弧线最短"
    返回: 路径点列表 [(lng, lat), ...]
    """
    direct_line = LineString([start, end])

    # 1. 检测原始直线是否与障碍物相交（包括边界接触）
    if not line_intersects_polygon(direct_line, obstacle_poly):
        # 直线安全，直接返回两点线段
        return [start, end]

    # 2. 需要绕飞：找出直线与障碍物边界的两个交点
    pts = get_intersection_points(direct_line, obstacle_poly.boundary)
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

    # 获取障碍物边界坐标（逆时针顺序）
    boundary_coords = list(obstacle_poly.exterior.coords)
    n = len(boundary_coords)

    # 找到入口和出口在边界上的最近索引
    e_idx = nearest_point_on_boundary(entry, boundary_coords)
    x_idx = nearest_point_on_boundary(exit_pt, boundary_coords)

    # 计算顺时针和逆时针方向的步数
    cw_dist = (x_idx - e_idx) % n      # 顺时针前进步数
    ccw_dist = (e_idx - x_idx) % n     # 逆时针前进步数

    # 根据模式选择方向
    if mode == "左侧绕飞":
        # 定义左侧绕飞 = 顺时针方向（索引递减）
        direction = "clockwise"
    elif mode == "右侧绕飞":
        # 右侧绕飞 = 逆时针方向（索引递增）
        direction = "counterclockwise"
    else:  # 弧线最短
        direction = "clockwise" if cw_dist <= ccw_dist else "counterclockwise"

    # 收集边界点序列（不包括起点终点）
    bypass = []
    i = e_idx
    if direction == "clockwise":
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i - 1) % n
        bypass.append(boundary_coords[x_idx])
    else:
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i + 1) % n
        bypass.append(boundary_coords[x_idx])

    # 核心折线路径
    core_path = [start] + bypass + [end]

    # 如果是弧线模式，进行样条平滑（最终仍为多段线）
    if mode == "弧线最短":
        return arc_smooth(core_path, num_per_segment=40)
    else:
        return core_path

# ==================== 地图绘制 ====================
def create_map(center_lng, center_lat, home, land, obstacles, temp_points, fly_mode):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=st.session_state.get("map_zoom", 19),
                   control_scale=True)

    # 底图（高德）
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 起飞点
    if home:
        folium.Marker([home[1], home[0]], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    # 降落点
    if land:
        folium.Marker([land[1], land[0]], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    # 障碍物（红色填充）
    for ob in obstacles:
        poly = Polygon(ob['points'])
        pts = [[p[1], p[0]] for p in ob['points']]
        folium.Polygon(locations=pts, color='red', fill=True, fill_opacity=0.5, popup=ob['name']).add_to(m)

    # 航线生成（使用第一个障碍物）
    if home and land and obstacles:
        obs_poly = Polygon(obstacles[0]['points'])
        path = plan_detour_path(home, land, obs_poly, mode=fly_mode)
        route = [[p[1], p[0]] for p in path]
        color_map = {"左侧绕飞": "blue", "右侧绕飞": "black", "弧线最短": "green"}
        color = color_map.get(fly_mode, "blue")
        folium.PolyLine(route, color=color, weight=5, opacity=1, popup=f"模式: {fly_mode}").add_to(m)

    # 临时打点（圈选障碍物）
    if len(temp_points) >= 3:
        temp_poly = [[p[1], p[0]] for p in temp_points]
        folium.Polygon(locations=temp_poly, color='purple', weight=2, fill_opacity=0.3).add_to(m)
    for pt in temp_points:
        folium.CircleMarker([pt[1], pt[0]], radius=4, color='purple', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 状态持久化 ====================
STATE_FILE = "flight_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "obstacles": st.session_state.obstacles,
        "temp_points": st.session_state.temp_points,
        "fly_mode": st.session_state.fly_mode,
        "map_zoom": st.session_state.map_zoom,
        "map_center": st.session_state.map_center
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            return json.load(f)
    return None

# ==================== 初始化 ====================
if "home_point" not in st.session_state:
    loaded = load_state()
    OFFICIAL_LNG, OFFICIAL_LAT = 118.749413, 32.234097
    defaults = {
        "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
        "land_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
        "obstacles": [],
        "temp_points": [],
        "last_click": None,
        "fly_mode": "左侧绕飞",
        "map_zoom": 19,
        "map_center": (OFFICIAL_LNG, OFFICIAL_LAT)
    }
    if loaded:
        for k, v in loaded.items():
            defaults[k] = v
    for k, v in defaults.items():
        st.session_state[k] = v

# ==================== 侧边栏界面 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")
    st.divider()

    st.subheader("🛫 飞行模式")
    st.session_state.fly_mode = st.selectbox(
        "绕飞模式", ["左侧绕飞", "右侧绕飞", "弧线最短"]
    )

    st.divider()
    st.subheader("✈️ 起飞点 / 降落点")
    col1, col2 = st.columns(2)
    with col1:
        home_lng = st.number_input("起飞经度", value=st.session_state.home_point[0], format="%.6f")
        home_lat = st.number_input("起飞纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point = (home_lng, home_lat)
            st.session_state.map_center = (home_lng, home_lat)
            save_state()
            st.rerun()
    with col2:
        land_lng = st.number_input("降落经度", value=st.session_state.land_point[0], format="%.6f")
        land_lat = st.number_input("降落纬度", value=st.session_state.land_point[1], format="%.6f")
        if st.button("更新降落点"):
            st.session_state.land_point = (land_lng, land_lat)
            save_state()
            st.rerun()

    st.divider()
    st.subheader("🚧 障碍物绘制")
    st.write(f"当前打点数量: {len(st.session_state.temp_points)}")
    obs_name = st.text_input("障碍物名称", "建筑物")
    if st.button("✅ 保存当前打点为障碍物"):
        if len(st.session_state.temp_points) >= 3:
            st.session_state.obstacles.append({
                "name": obs_name,
                "points": st.session_state.temp_points.copy()
            })
            st.session_state.temp_points = []
            save_state()
            st.success(f"障碍物 '{obs_name}' 已保存")
            st.rerun()
        else:
            st.warning("至少需要 3 个点才能构成封闭区域")
    if st.button("🗑️ 清空当前打点"):
        st.session_state.temp_points = []
        save_state()
        st.rerun()

    st.subheader("📦 已保存障碍物")
    for i, ob in enumerate(st.session_state.obstacles):
        col1, col2 = st.columns([3, 1])
        col1.write(f"{i+1}. {ob['name']} ({len(ob['points'])}点)")
        if col2.button("删除", key=f"del_ob_{i}"):
            st.session_state.obstacles.pop(i)
            save_state()
            st.rerun()
    if st.button("清空所有障碍物"):
        st.session_state.obstacles = []
        save_state()
        st.rerun()

# ==================== 地图区域 ====================
st.header("🗺️ 无人机绕飞航线规划（纯几何避障）")
st.markdown("**操作指南**：在地图上点击打点（至少3个）→ 侧边栏保存为障碍物 → 设置起降点 → 选择绕飞模式 → 自动生成紧贴障碍物边界的多段航线")

center = st.session_state.map_center
zoom = st.session_state.map_zoom

m = create_map(
    center_lng=center[0],
    center_lat=center[1],
    home=st.session_state.home_point,
    land=st.session_state.land_point,
    obstacles=st.session_state.obstacles,
    temp_points=st.session_state.temp_points,
    fly_mode=st.session_state.fly_mode
)
output = st_folium(m, width=1100, height=680, key="flight_map")

# 保存地图视图
if output and output.get("center") and output.get("zoom"):
    st.session_state.map_center = (output["center"]["lng"], output["center"]["lat"])
    st.session_state.map_zoom = output["zoom"]

# 处理打点
if output and output.get("last_clicked"):
    lat = output["last_clicked"]["lat"]
    lng = output["last_clicked"]["lng"]
    pt = (round(lng, 6), round(lat, 6))
    if st.session_state.last_click != pt:
        st.session_state.last_click = pt
        st.session_state.temp_points.append(pt)
        save_state()
        st.rerun()
