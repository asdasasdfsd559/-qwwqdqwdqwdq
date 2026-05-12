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

# ==================== 安全缓冲区 ====================
SAFE_BUFFER = 0.00015

def get_obstacle_with_buffer(obs_poly):
    return obs_poly.buffer(SAFE_BUFFER)

def simplify_vertices(vertices, max_vertices=10):
    """
    对顶点列表进行简化，保留最多 max_vertices 个点（始终保留首尾）
    """
    if len(vertices) <= max_vertices:
        return vertices
    step = (len(vertices) - 1) / (max_vertices - 1)
    indices = [int(round(i * step)) for i in range(max_vertices - 1)] + [len(vertices) - 1]
    return [vertices[i] for i in indices]

def plan_safe_path(start, end, obstacles, fly_mode):
    """
    生成航点序列，点数大幅减少：
     - 直飞最短：始终返回 [start, end] (2个点)
     - 左右绕飞：返回 [start] + (缓冲区关键顶点) + [end] (顶点数 ≤ 12)
     - 弧线最短：沿缓冲区边界均匀采样最多12个点 (包括起终点共≤14)
    """
    # 直飞最短：无条件返回起点终点
    if fly_mode == "直飞最短":
        return [start, end]

    # 无任何障碍物
    if not obstacles:
        if fly_mode == "弧线最短航线":
            # 贝塞尔曲线，采样11个点（共11个）
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            perp = (-dy, dx)
            length = math.hypot(*perp)
            if length > 0:
                perp = (perp[0]/length, perp[1]/length)
            offset = 0.0002
            cx += perp[0]*offset
            cy += perp[1]*offset
            points = []
            for t in [i/10 for i in range(11)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * cx + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * cy + t**2 * end[1]
                points.append((x, y))
            return points
        return [start, end]

    # 有障碍物
    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])

    # 直线不与缓冲区相交，无需绕飞
    if not direct_line.intersects(buffered):
        if fly_mode == "弧线最短航线":
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            perp = (-dy, dx)
            length = math.hypot(*perp)
            if length > 0:
                perp = (perp[0]/length, perp[1]/length)
            offset = 0.0002
            cx += perp[0]*offset
            cy += perp[1]*offset
            points = []
            for t in [i/10 for i in range(11)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * cx + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * cy + t**2 * end[1]
                points.append((x, y))
            return points
        return [start, end]

    # ---------- 需要绕飞 ----------
    # 获取直线与缓冲区的两个交点
    inter = direct_line.intersection(buffered.boundary)
    if inter.geom_type == 'MultiPoint':
        pts = list(inter.geoms)
    else:
        pts = [inter] if not inter.is_empty else []
    if len(pts) < 2:
        return [start, end]

    # 入口（离起点近）和出口
    d1 = math.hypot(pts[0].x - start[0], pts[0].y - start[1])
    d2 = math.hypot(pts[1].x - start[0], pts[1].y - start[1])
    entry, exit_pt = (pts[0], pts[1]) if d1 < d2 else (pts[1], pts[0])

    # 获取缓冲区边界顶点
    boundary_coords = list(buffered.exterior.coords)
    # 去重
    unique_vertices = []
    for p in boundary_coords:
        if not unique_vertices or (p[0] != unique_vertices[-1][0] or p[1] != unique_vertices[-1][1]):
            unique_vertices.append(p)
    n = len(unique_vertices)

    # 找到入口和出口最近的顶点索引
    def nearest_idx(pt):
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(unique_vertices):
            d = math.hypot(p[0] - pt.x, p[1] - pt.y)
            if d < min_d:
                min_d = d
                idx = i
        return idx
    e_idx = nearest_idx(entry)
    x_idx = nearest_idx(exit_pt)

    # 顺时针（索引递增）和逆时针（索引递减）的顶点序列
    def get_vertices_seq(start_idx, end_idx, clockwise=True):
        seq = []
        i = start_idx
        if clockwise:
            while i != end_idx:
                seq.append(unique_vertices[i])
                i = (i + 1) % n
            seq.append(unique_vertices[end_idx])
        else:
            while i != end_idx:
                seq.append(unique_vertices[i])
                i = (i - 1) % n
            seq.append(unique_vertices[end_idx])
        return seq

    cw_seq = get_vertices_seq(e_idx, x_idx, clockwise=True)
    ccw_seq = get_vertices_seq(e_idx, x_idx, clockwise=False)

    # 根据模式选择方向
    if fly_mode == "左侧绕飞":
        chosen_seq = ccw_seq   # 左侧绕飞 = 逆时针
    elif fly_mode == "右侧绕飞":
        chosen_seq = cw_seq    # 右侧绕飞 = 顺时针
    else:  # 弧线最短航线
        chosen_seq = cw_seq if len(cw_seq) <= len(ccw_seq) else ccw_seq

    # 对选择的序列进行精简（最多保留10个点，保证起点和终点在序列中已被包含）
    # 注意 chosen_seq 已经包含了入口和出口附近的顶点，但数量可能很多（例如多边形顶点多时）
    # 我们再次精简
    if len(chosen_seq) > 12:
        step = (len(chosen_seq) - 1) / 10
        indices = [int(round(i * step)) for i in range(11)]
        chosen_seq = [chosen_seq[i] for i in indices]

    # 构建最终路径
    final_path = [start] + chosen_seq + [end]

    # 如果弧线模式，可以对路径再进行一次均匀采样（可选，但会稍微增加点数，目前不做了）
    return final_path

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
        # 保存精简后的航点供飞行监控使用
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
st.header("🗺️ 航线规划（航点数已精简至最低）")
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
