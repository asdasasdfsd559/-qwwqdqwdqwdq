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

# ==================== 安全缓冲区 ====================
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    """生成障碍物的安全缓冲区（向外扩展）"""
    return obs_poly.buffer(SAFE_BUFFER)

def get_around_path(start, end, obstacle, fly_mode):
    """
    核心：生成绕飞路径（已交换左右绕飞方向）
    - 左侧绕飞：顺时针（原右侧）
    - 右侧绕飞：逆时针（原左侧）
    """
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    ring = LinearRing(buffered.exterior.coords)
    
    # 计算起点/终点到障碍物的最近点
    start_pt = Point(start)
    end_pt = Point(end)
    start_dist = ring.project(start_pt)
    end_dist = ring.project(end_pt)
    
    # 采样点数（越多越平滑）
    num_points = 30  
    path_points = []
    
    # 交换左右绕飞方向
    if fly_mode == "左侧绕飞":
        # 左侧绕飞 → 顺时针（原右侧逻辑）
        if start_dist > end_dist:
            distances = [start_dist + i*((ring.length - start_dist) + end_dist)/num_points for i in range(num_points+1)]
            distances = [d % ring.length for d in distances]
        else:
            distances = [start_dist + i*(end_dist - start_dist)/num_points for i in range(num_points+1)]
    elif fly_mode == "右侧绕飞":
        # 右侧绕飞 → 逆时针（原左侧逻辑）
        if start_dist < end_dist:
            distances = [start_dist - i*(start_dist + (ring.length - end_dist))/num_points for i in range(num_points+1)]
            distances = [d % ring.length for d in distances]
        else:
            distances = [start_dist - i*(start_dist - end_dist)/num_points for i in range(num_points+1)]
    else:
        return []
    
    # 生成绕飞点
    for d in distances:
        p = ring.interpolate(d)
        path_points.append((p.x, p.y))
    
    return path_points

def plan_safe_path(start, end, obstacles, fly_mode):
    """
    修复后：交换左右绕飞 + 优化弧线弧度/最短路径
    """
    # 无障碍物：直飞/增强弧度的弧线
    if not obstacles:
        if fly_mode == "弧线最短航线":
            # 增大弧线偏移量（弧度更明显）
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            # 偏移量从0.0003增大到0.0008，弧度更明显
            perp = (-dy*0.0008, dx*0.0008)  
            cx += perp[0]
            cy += perp[1]
            # 增加采样点（30→50），弧线更顺滑
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/50 for i in range(51)]]
        return [start, end]
    
    # 有障碍物：取第一个障碍物
    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    direct_line = LineString([start, end])
    
    # 检测是否需要绕飞
    if not direct_line.intersects(get_obstacle_with_buffer(obs_poly)):
        if fly_mode == "弧线最短航线":
            # 无障碍时增大弧线弧度
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx, dy = end[0]-start[0], end[1]-start[1]
            perp = (-dy*0.0008, dx*0.0008)
            cx += perp[0]
            cy += perp[1]
            return [((1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0],
                     (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1]) for t in [i/50 for i in range(51)]]
        return [start, end]
    
    # 需要绕飞
    if fly_mode in ["左侧绕飞", "右侧绕飞"]:
        around_points = get_around_path(start, end, obstacle, fly_mode)
        full_path = [start] + around_points + [end]
        return full_path
    elif fly_mode == "弧线最短航线":
        # 优化最短路径计算：精确计算绕飞总长度（含起点+终点）
        def calc_total_path_length(base_start, base_end, around_points):
            """计算完整路径长度（起点→绕飞点→终点）"""
            length = math.hypot(around_points[0][0]-base_start[0], around_points[0][1]-base_start[1])
            for i in range(1, len(around_points)):
                length += math.hypot(around_points[i][0]-around_points[i-1][0], around_points[i][1]-around_points[i-1][1])
            length += math.hypot(base_end[0]-around_points[-1][0], base_end[1]-around_points[-1][1])
            return length
        
        # 生成左右绕飞路径
        left_around = get_around_path(start, end, obstacle, "左侧绕飞")
        right_around = get_around_path(start, end, obstacle, "右侧绕飞")
        
        # 计算真实总长度
        left_total_len = calc_total_path_length(start, end, left_around)
        right_total_len = calc_total_path_length(start, end, right_around)
        
        # 选更短的路径
        around_points = left_around if left_total_len < right_total_len else right_around
        full_path = [start] + around_points + [end]
        
        # 增强弧线平滑度 + 增大弧度
        def enhanced_catmull_rom(points, num_segments=30):
            """增强版样条插值，弧度更明显"""
            if len(points) < 3:
                return points
            smooth = []
            # 手动增加中间点的偏移，强化弧度
            mid_idx = len(points) // 2
            mid_pt = points[mid_idx]
            dx = mid_pt[0] - (points[0][0]+points[-1][0])/2
            dy = mid_pt[1] - (points[0][1]+points[-1][1])/2
            # 偏移放大系数
            amplify = 1.5  
            adjusted_points = []
            for i, p in enumerate(points):
                if i == mid_idx:
                    adjusted_points.append((p[0]+dx*amplify, p[1]+dy*amplify))
                else:
                    adjusted_points.append(p)
            
            # 样条插值
            for i in range(len(adjusted_points)-1):
                p0 = adjusted_points[max(0, i-1)]
                p1 = adjusted_points[i]
                p2 = adjusted_points[i+1]
                p3 = adjusted_points[min(len(adjusted_points)-1, i+2)]
                for t in [j/num_segments for j in range(num_segments+1)]:
                    t2 = t*t
                    t3 = t2*t
                    x = 0.5 * (2*p1[0] + (-p0[0]+p2[0])*t + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
                    y = 0.5 * (2*p1[1] + (-p0[1]+p2[1])*t + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
                    smooth.append((x, y))
            return smooth
        
        return enhanced_catmull_rom(full_path)
    else:
        return [start, end]

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

    # 绘制障碍物+安全缓冲区
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        # 障碍物本体
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)
        # 安全缓冲区
        obs_poly = Polygon(ob['points'])
        buffered = get_obstacle_with_buffer(obs_poly)
        buffer_ps = []
        for lng, lat in buffered.exterior.coords:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            buffer_ps.append([lat, lng])
        folium.Polygon(locations=buffer_ps, color='orange', fill=True, fill_opacity=0.1, weight=2, popup="安全缓冲区").add_to(m)

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
st.header("🗺️ 航线规划（最终版：交换左右+增强弧线）")
st.success(f"✅ 当前模式：{st.session_state.fly_mode} | 安全距离约15米 | 弧线弧度已增强")

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
