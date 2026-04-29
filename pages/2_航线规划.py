import streamlit as st
import json
import os
import math
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon, LinearRing
from shapely.ops import unary_union, simplify

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
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    """生成障碍物的安全缓冲区（保留直角，不生成圆角）"""
    # 使用join_style=2保留直角，mitre_limit限制尖角，避免缓冲区圆角
    return obs_poly.buffer(SAFE_BUFFER, join_style=2, mitre_limit=10.0)

def get_pure_polyline_around_path(start, end, obstacle, fly_mode):
    """
    生成真正的多段折线绕飞路径（仅保留核心拐点，无平滑）
    fly_mode: "左侧绕飞" / "右侧绕飞"
    """
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    
    # 简化折线：只保留核心拐点（去除冗余点）
    simplified = simplify(buffered.exterior, tolerance=0.00005, preserve_topology=True)
    vertices = list(simplified.coords)
    
    # 计算起点和终点到障碍物的最近顶点
    start_pt = Point(start)
    end_pt = Point(end)
    
    # 找到起点/终点最近的顶点
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
    
    # 根据绕飞方向选择折线顶点顺序（仅保留核心拐点）
    start_idx = vertices.index(start_nearest)
    end_idx = vertices.index(end_nearest)
    
    if fly_mode == "左侧绕飞":
        # 左侧绕飞：顺时针遍历核心顶点（真正折线）
        if start_idx < end_idx:
            polyline_points = vertices[start_idx:end_idx+1]
        else:
            polyline_points = vertices[start_idx:] + vertices[:end_idx+1]
    else:
        # 右侧绕飞：逆时针遍历核心顶点（真正折线）
        if start_idx > end_idx:
            polyline_points = vertices[end_idx:start_idx+1][::-1]
        else:
            polyline_points = (vertices[end_idx:] + vertices[:start_idx+1])[::-1]
    
    # 再次去重+简化：确保每一段都是明显的直线段
    unique_points = []
    prev = None
    for p in polyline_points:
        if prev is None:
            unique_points.append(p)
            prev = p
        else:
            # 只保留距离大于阈值的点（避免密集点）
            dist = math.hypot(p[0]-prev[0], p[1]-prev[1])
            if dist > 0.00008:  # 约8米距离才保留拐点
                unique_points.append(p)
                prev = p
    
    return unique_points

def plan_safe_path(start, end, obstacles, fly_mode):
    """
    左右绕飞：纯多段折线（无任何弧线/平滑）
    弧线最短：暂时保留（如需也可改为折线）
    """
    # 无障碍物：直飞（单段折线）
    if not obstacles:
        return [start, end]
    
    # 有障碍物：取第一个障碍物
    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])
    
    # 检测是否需要绕飞
    if not direct_line.intersects(buffered):
        # 直线不穿过障碍物，直飞（单段折线）
        return [start, end]
    
    # 需要绕飞
    if fly_mode == "左侧绕飞" or fly_mode == "右侧绕飞":
        # 左右绕飞：真正的多段折线（核心修改）
        polyline_points = get_pure_polyline_around_path(start, end, obstacle, fly_mode)
        # 拼接完整折线路径：起点 -> 核心拐点 -> 终点
        full_path = [start] + polyline_points + [end]
        return full_path
    
    # 弧线最短航线：暂时保留原逻辑（如需可改为最短折线）
    else:
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
        
        full_path = [start] + around_points + [end]
        return full_path

# ==================== 地图绘制 ====================
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

    # 仅绘制障碍物本体
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
        
        # 绘制纯折线路径（无平滑）
        folium.PolyLine(route, color=color, weight=5, opacity=1, popup="无人机航线").add_to(m)
        
        # 标记所有折线拐点（清晰可见）
        if fly_mode in ["左侧绕飞", "右侧绕飞"] and len(route) > 2:
            for i, (lat, lng) in enumerate(route):
                if i > 0 and i < len(route)-1:  # 仅标记中间拐点
                    folium.CircleMarker([lat, lng], radius=5, color='yellow', fill=True, 
                                       popup=f"拐点 {i}", weight=2).add_to(m)

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
