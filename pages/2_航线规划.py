import streamlit as st
import json
import os
import math
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="航线规划", layout="wide")

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 安全缓冲区（仅用于路径计算，不绘制） ====================
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

# ==================== 几何辅助函数 ====================
def line_intersects_polygon(line, poly):
    """线段是否与多边形相交（包含边界接触）"""
    return line.intersects(poly)

def get_intersection_points(line, boundary):
    """获取线段与多边形边界的所有交点"""
    inter = line.intersection(boundary)
    if inter.is_empty:
        return []
    if inter.geom_type == 'Point':
        return [inter]
    if inter.geom_type == 'MultiPoint':
        return list(inter.geoms)
    if inter.geom_type == 'LineString':
        return [Point(inter.coords[0]), Point(inter.coords[-1])]
    return []

def nearest_boundary_index(point, coords):
    """找到多边形边界上离给定点最近的点索引"""
    min_dist = float('inf')
    idx = 0
    for i, (x, y) in enumerate(coords):
        d = math.hypot(x - point.x, y - point.y)
        if d < min_dist:
            min_dist = d
            idx = i
    return idx

def arc_smooth(points, num_per_segment=40):
    """Catmull-Rom 样条插值，生成密集的多段线（模拟圆弧）"""
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
    # 去重
    uniq = []
    for pt in smooth:
        if not uniq or math.hypot(pt[0]-uniq[-1][0], pt[1]-uniq[-1][1]) > 1e-9:
            uniq.append(pt)
    return uniq

# ==================== 核心绕飞规划（带安全缓冲区） ====================
def plan_safe_path(start, end, obstacles, fly_mode):
    """
    无人机绕飞航线生成（使用安全缓冲区，航线距离障碍物15米）
    方向：左侧绕飞 = 逆时针，右侧绕飞 = 顺时针
    """
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

    # 使用第一个障碍物的缓冲区
    obs_poly = Polygon(obstacles[0]["points"])
    buffered_poly = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])

    # 检测直线是否与缓冲区相交
    if not line_intersects_polygon(direct_line, buffered_poly):
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

    # 需要绕飞：获取直线与缓冲区边界的两个交点
    pts = get_intersection_points(direct_line, buffered_poly.boundary)
    if len(pts) < 2:
        return [start, end]

    # 确定入口（靠近起点）和出口（靠近终点）
    d1 = math.hypot(pts[0].x - start[0], pts[0].y - start[1])
    d2 = math.hypot(pts[1].x - start[0], pts[1].y - start[1])
    if d1 < d2:
        entry, exit_pt = pts[0], pts[1]
    else:
        entry, exit_pt = pts[1], pts[0]

    # 获取缓冲区边界点（逆时针顺序）
    boundary_coords = list(buffered_poly.exterior.coords)
    n = len(boundary_coords)

    e_idx = nearest_boundary_index(entry, boundary_coords)
    x_idx = nearest_boundary_index(exit_pt, boundary_coords)

    # 计算顺时针（索引递增）和逆时针（索引递减）的步数
    cw_steps = (x_idx - e_idx) % n
    ccw_steps = (e_idx - x_idx) % n

    # 根据模式选择绕行方向（已交换：左侧=逆时针，右侧=顺时针）
    if fly_mode == "左侧绕飞":
        direction = "counterclockwise"   # 逆时针
    elif fly_mode == "右侧绕飞":
        direction = "clockwise"          # 顺时针
    else:  # 弧线最短航线
        direction = "clockwise" if cw_steps <= ccw_steps else "counterclockwise"

    # 收集绕飞边界点
    bypass = []
    i = e_idx
    if direction == "clockwise":
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i + 1) % n
        bypass.append(boundary_coords[x_idx])
    else:
        while i != x_idx:
            bypass.append(boundary_coords[i])
            i = (i - 1) % n
        bypass.append(boundary_coords[x_idx])

    # 构建核心路径
    core_path = [start] + bypass + [end]

    if fly_mode == "弧线最短航线":
        return arc_smooth(core_path, num_per_segment=40)
    else:
        return core_path

# ==================== 地图绘制（只绘制障碍物，不绘制缓冲区） ====================
def create_map(center_lng, center_lat, waypoints, home_point, land_point, obstacles, coord_system, temp_points, fly_mode):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=st.session_state.get("map_zoom", 19),
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

    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)

    if len(waypoints) >= 2:
        safe_path = plan_safe_path(waypoints[0], waypoints[-1], obstacles, fly_mode)
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
        "map_zoom": st.session_state.get("map_zoom", 19),
        "map_center": st.session_state.get("map_center", st.session_state.home_point)
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None

# ==================== 初始化 ====================
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
        "map_zoom": 19,
        "map_center": (OFFICIAL_LNG, OFFICIAL_LAT)
    }
    for k, v in defaults.items():
        if loaded and k in loaded:
            st.session_state[k] = loaded[k]
        else:
            st.session_state[k] = v

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")

    st.session_state.coord_system = st.selectbox(
        "坐标系", ["gcj02", "wgs84"],
        format_func=lambda x: "GCJ02(国内)" if x == "gcj02" else "WGS84(GPS)"
    )
    st.subheader("🏠 起飞点")
    hlng = st.number_input("起飞经度", value=st.session_state.home_point[0], format="%.6f")
    hlat = st.number_input("起飞纬度", value=st.session_state.home_point[1], format="%.6f")
    if st.button("更新起飞点"):
        st.session_state.home_point = (hlng, hlat)
        st.session_state.map_center = (hlng, hlat)
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

# ==================== 地图显示 ====================
st.header("🗺️ 航线规划（左右绕飞方向已交换，航线与障碍物保持15米安全距离）")
st.success(f"✅ 当前模式：{st.session_state.fly_mode}")

center = st.session_state.get("map_center", st.session_state.home_point)
zoom = st.session_state.get("map_zoom", 19)

m = create_map(
    center[0], center[1],
    st.session_state.waypoints,
    st.session_state.home_point,
    st.session_state.land_point,
    st.session_state.obstacles,
    st.session_state.coord_system,
    st.session_state.draw_points,
    st.session_state.fly_mode
)
output = st_folium(m, width=1100, height=680, key="main_map")

if output and output.get("center") and output.get("zoom"):
    st.session_state.map_center = (output["center"]["lng"], output["center"]["lat"])
    st.session_state.map_zoom = output["zoom"]

if output and output.get("last_clicked"):
    lat = output["last_clicked"]["lat"]
    lng = output["last_clicked"]["lng"]
    pt = (round(lng, 6), round(lat, 6))
    if st.session_state.last_click != pt:
        st.session_state.last_click = pt
        st.session_state.draw_points.append(pt)
        save_state()
        st.rerun()
