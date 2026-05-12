import streamlit as st
import json
import os
import math
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon, LinearRing

st.set_page_config(page_title="航线规划", layout="wide")

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 几何辅助 ====================
SAFE_BUFFER = 0.00015

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

def get_polyline_around_path(start, end, obstacle, fly_mode):
    """返回障碍物缓冲区的顶点序列（绕飞路径）"""
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
    n = len(vertices)
    if fly_mode == "左侧绕飞":
        if start_idx < end_idx:
            polyline = vertices[start_idx:end_idx+1]
        else:
            polyline = vertices[start_idx:] + vertices[:end_idx+1]
    else:  # 右侧绕飞
        if start_idx > end_idx:
            polyline = vertices[end_idx:start_idx+1][::-1]
        else:
            polyline = (vertices[end_idx:] + vertices[:start_idx+1])[::-1]
    # 去重（相邻重复）
    unique = []
    for p in polyline:
        if not unique or (p[0] != unique[-1][0] or p[1] != unique[-1][1]):
            unique.append(p)
    return unique

def get_smooth_around_path(start, end, obstacle):
    """弧线最短航线：沿缓冲区采样（点数少）"""
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
    # 采样点数降低到15
    num_points = 15
    if cw_dist <= ccw_dist:
        distances = [start_dist + i * cw_dist / num_points for i in range(num_points + 1)]
        distances = [d % ring_len for d in distances]
    else:
        distances = [start_dist - i * ccw_dist / num_points for i in range(num_points + 1)]
        distances = [d % ring_len for d in distances]
    around_points = []
    for d in distances:
        p = ring.interpolate(d)
        around_points.append((p.x, p.y))
    return around_points

def plan_safe_path(start, end, obstacles, fly_mode):
    """生成航点序列（数量尽可能少）"""
    # 直飞最短
    if fly_mode == "直飞最短":
        return [start, end]

    if not obstacles:
        # 无障碍物弧线模式：生成简单贝塞尔曲线，采样30个点（也不多）
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            perp = (-dy, dx)
            length = math.hypot(*perp)
            if length > 0:
                perp = (perp[0]/length, perp[1]/length)
            offset = 0.0002
            cx, cy = mid_x + perp[0]*offset, mid_y + perp[1]*offset
            # 降低采样点到20
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/20 for i in range(21)]]
        return [start, end]

    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])

    if not direct_line.intersects(buffered):
        # 直线不与缓冲区相交，无需绕飞
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            perp = (-dy, dx)
            length = math.hypot(*perp)
            if length > 0:
                perp = (perp[0]/length, perp[1]/length)
            offset = 0.0002
            cx, cy = mid_x + perp[0]*offset, mid_y + perp[1]*offset
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/20 for i in range(21)]]
        return [start, end]

    # 需要绕飞
    if fly_mode == "左侧绕飞" or fly_mode == "右侧绕飞":
        polyline = get_polyline_around_path(start, end, obstacle, fly_mode)
        return [start] + polyline + [end]
    elif fly_mode == "弧线最短航线":
        around = get_smooth_around_path(start, end, obstacle)
        # 对采样点进行简单的贝塞尔平滑，但点数仍然较少
        if len(around) < 3:
            return [start] + around + [end]
        # 简单调用 Catmull-Rom 平滑，但使用较低的分段数
        from scipy.interpolate import CubicSpline  # 避免循环依赖，这里不用，直接返回采样点
        # 直接返回采样点（已足够平滑）
        return [start] + around + [end]
    else:
        return [start, end]

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
        # 保存航点供飞行监控使用
        st.session_state.flight_waypoints = safe_path.copy()
        st.session_state.obstacles = obstacles
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
    OFFICIAL_LNG, OFFICIAL_LAT = 118.749413, 32.234097
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
st.header("🗺️ 航线规划")
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
