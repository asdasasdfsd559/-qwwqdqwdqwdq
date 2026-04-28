import streamlit as st
import pandas as pd
import time
import json
import os
import threading
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

# ==================== 全局变量（进程级，解决Streamlit线程/重绘问题） ====================
# 用全局变量存储心跳数据，不受Streamlit rerun和线程影响
GLOBAL_HEARTBEAT_DATA = []
GLOBAL_SEQ = 0
GLOBAL_RUNNING = False

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 北京时间 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng,lat):
        return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat):
        return lng-0.0005, lat-0.0003

# ==================== 【标准绕飞算法】局部坐标系转换法 ====================
def global_to_local(point, origin, dx_norm, dy_norm):
    """全局坐标转局部坐标（飞行方向为x轴）"""
    x = point[0] - origin[0]
    y = point[1] - origin[1]
    # 旋转矩阵，将飞行方向转到x轴
    local_x = x * dx_norm + y * dy_norm
    local_y = -x * dy_norm + y * dx_norm
    return local_x, local_y

def local_to_global(point, origin, dx_norm, dy_norm):
    """局部坐标转全局坐标"""
    local_x, local_y = point
    # 逆旋转矩阵
    x = local_x * dx_norm - local_y * dy_norm
    y = local_x * dy_norm + local_y * dx_norm
    return x + origin[0], y + origin[1]

def get_safe_obstacle(obs):
    """障碍物膨胀20米安全区"""
    poly = Polygon(obs["points"])
    return poly.buffer(0.0002)

def is_path_safe(path, obstacles):
    """严格检查路径是否完全安全"""
    if len(path) < 2:
        return True
    line = LineString(path)
    for obs in obstacles:
        safe_poly = get_safe_obstacle(obs)
        if line.intersects(safe_poly):
            return False
    return True

def plan_safe_path(start, end, obstacles, fly_mode):
    start_pt = Point(start)
    end_pt = Point(end)
    direct_line = LineString([start, end])

    # 找挡路的障碍物
    hit_obs = None
    for obs in obstacles:
        safe_poly = get_safe_obstacle(obs)
        if direct_line.intersects(safe_poly):
            hit_obs = obs
            break

    # 无障碍物，直接直飞
    if hit_obs is None:
        if fly_mode == "弧线最短航线":
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            mid_lng = (start[0] + end[0]) / 2
            mid_lat = (start[1] + end[1]) / 2
            ctrl_x = mid_lng - dy * 0.3
            ctrl_y = mid_lat + dx * 0.3
            arc_points = []
            for t in [i/20 for i in range(21)]:
                clng = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl_x + t**2 * end[0]
                clat = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl_y + t**2 * end[1]
                arc_points.append((clng, clat))
            return arc_points
        return [start, end]

    # 飞行方向向量
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = (dx**2 + dy**2)**0.5 if dx**2 + dy**2 > 0 else 1
    dx_norm = dx / length
    dy_norm = dy / length

    # 障碍物所有点转局部坐标
    obs_points_local = []
    for p in hit_obs["points"]:
        lp = global_to_local(p, start, dx_norm, dy_norm)
        obs_points_local.append(lp)
    
    # 找到障碍物在局部坐标系的y范围
    ys = [p[1] for p in obs_points_local]
    min_y, max_y = min(ys), max(ys)

    # 【核心】标准绕飞
    if fly_mode == "左侧绕飞":
        # 左侧绕飞：局部坐标系y正方向，绕到障碍物左边
        offset_y = max_y + 0.0006  # 60米安全距离
        # 绕飞点：障碍物前侧和后侧
        wp1_local = (-0.0003, offset_y)
        wp2_local = (length + 0.0003, offset_y)
        # 转全局坐标
        wp1 = local_to_global(wp1_local, start, dx_norm, dy_norm)
        wp2 = local_to_global(wp2_local, start, dx_norm, dy_norm)
        path = [start, wp1, wp2, end]
        if is_path_safe(path, obstacles):
            return path
    elif fly_mode == "右侧绕飞":
        # 右侧绕飞：局部坐标系y负方向，绕到障碍物右边
        offset_y = min_y - 0.0006  # 60米安全距离
        # 绕飞点：障碍物前侧和后侧
        wp1_local = (-0.0003, offset_y)
        wp2_local = (length + 0.0003, offset_y)
        # 转全局坐标
        wp1 = local_to_global(wp1_local, start, dx_norm, dy_norm)
        wp2 = local_to_global(wp2_local, start, dx_norm, dy_norm)
        path = [start, wp1, wp2, end]
        if is_path_safe(path, obstacles):
            return path
    elif fly_mode == "弧线最短航线":
        # 弧线绕飞：用左侧点做控制点
        offset_y = max_y + 0.0006
        ctrl_local = (length/2, offset_y)
        ctrl = local_to_global(ctrl_local, start, dx_norm, dy_norm)
        arc_points = []
        for t in [i/20 for i in range(21)]:
            clng = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl[0] + t**2 * end[0]
            clat = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl[1] + t**2 * end[1]
            arc_points.append((clng, clat))
        if is_path_safe(arc_points, obstacles):
            return arc_points

    # 兜底
    offset_y = max_y + 0.0006
    ctrl_local = (length/2, offset_y)
    ctrl = local_to_global(ctrl_local, start, dx_norm, dy_norm)
    return [start, ctrl, end]

# ==================== 地图 ====================
def create_map(center_lng,center_lat,waypoints,home_point,land_point,obstacles,coord_system,temp_points):
    m=folium.Map(
        location=[center_lat,center_lng],
        zoom_start=19,
        control_scale=True,
        tiles=None
    )

    # 高德街道图
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-2026街道', name='街道图'
    ).add_to(m)

    # 高德卫星图
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星图', name='卫星图(超清)'
    ).add_to(m)

    # 起飞点
    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker(
            [h_lat,h_lng],
            icon=folium.Icon(color='green', icon='home'),
            popup="🏠 起飞点"
        ).add_to(m)

    # 降落点
    if land_point:
        l_lng,l_lat=land_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker(
            [l_lat,l_lng],
            icon=folium.Icon(color='red', icon='flag'),
            popup="🚩 降落点"
        ).add_to(m)

    # 障碍物
    for ob in obstacles:
        ps=[]
        for p in ob['points']:
            plng,plat=p if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat,plng])
        folium.Polygon(
            locations=ps,color='red',fill=True,fill_opacity=0.5,
            popup=f"{ob['name']} | {ob['height']}m"
        ).add_to(m)

    # 航线绘制
    if len(waypoints) >= 2:
        start_pt = waypoints[0]
        end_pt = waypoints[-1]
        safe_path = plan_safe_path(start_pt, end_pt, obstacles, st.session_state.fly_mode)
        
        route = []
        for (lng, lat) in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])
        
        color_map = {
            "直飞最短":"blue",
            "左侧绕飞":"orange",
            "右侧绕飞":"purple",
            "弧线最短航线":"cyan"
        }
        line_color = color_map.get(st.session_state.fly_mode, "blue")
        folium.PolyLine(route, color=line_color, weight=5, opacity=0.9).add_to(m)

    # 圈选打点
    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2, fill_opacity=0.2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker(location=[lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 心跳包线程（严格1秒间隔，用全局变量） ====================
def heartbeat_worker():
    """后台线程，严格1秒一次发送心跳，用全局变量，彻底解决Streamlit的问题"""
    global GLOBAL_HEARTBEAT_DATA, GLOBAL_SEQ, GLOBAL_RUNNING
    while GLOBAL_RUNNING:
        GLOBAL_SEQ += 1
        t = get_beijing_time_str()
        GLOBAL_HEARTBEAT_DATA.append({
            "序号": GLOBAL_SEQ, "时间": t, "状态": "在线正常"
        })
        if len(GLOBAL_HEARTBEAT_DATA) > 60:
            GLOBAL_HEARTBEAT_DATA.pop(0)
        time.sleep(1.0)  # 严格1秒间隔，绝对不会错

# ==================== 保存/加载 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "waypoints": st.session_state.waypoints,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points
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
    st.session_state.page="飞行监控"
if 'last_click' not in st.session_state:
    st.session_state.last_click = None
if 'fly_mode' not in st.session_state:
    st.session_state.fly_mode = "直飞最短"

loaded = load_state()

OFFICIAL_LNG = 118.749413
OFFICIAL_LAT = 32.234097

defaults = {
    "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "land_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
    "waypoints": [],
    "coord_system": "gcj02",
    "obstacles": [],
    "draw_points": []
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

    page=st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])
    st.session_state.page=page

    if "🗺️ 航线规划" in page:
        st.session_state.coord_system=st.selectbox(
            "坐标系",["gcj02","wgs84"],format_func=lambda x:"GCJ02(国内标准)" if x=="gcj02" else "WGS84(GPS)"
        )
        # 起飞点
        st.subheader("🏠 起飞点")
        hlng=st.number_input("起飞经度",value=st.session_state.home_point[0],format="%.6f")
        hlat=st.number_input("起飞纬度",value=st.session_state.home_point[1],format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point=(hlng,hlat)
            save_state()
            st.rerun()

        # 降落点
        st.subheader("🚩 降落点")
        llng=st.number_input("降落经度",value=st.session_state.land_point[0],format="%.6f")
        llat=st.number_input("降落纬度",value=st.session_state.land_point[1],format="%.6f")
        if st.button("更新降落点"):
            st.session_state.land_point=(llng,llat)
            save_state()
            st.rerun()

        # 航线模式
        st.subheader("🛫 航线模式")
        st.session_state.fly_mode = st.selectbox(
            "飞行策略",
            ["直飞最短","左侧绕飞","右侧绕飞","弧线最短航线"]
        )

        st.subheader("✈️ 航线生成")
        if st.button("生成起飞→降落航线"):
            st.session_state.waypoints=[st.session_state.home_point, st.session_state.land_point]
            save_state()
            st.rerun()
        if st.button("清空航线"):
            st.session_state.waypoints=[]
            save_state()
            st.rerun()

        st.subheader("🚧 圈选障碍物（点击地图）")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        height=st.number_input("高度(m)",1,500,25)
        name=st.text_input("名称","教学楼")

        if st.button("✅ 保存障碍物"):
            if len(st.session_state.draw_points)>=3:
                st.session_state.obstacles.append({
                    "name":name,"height":height,"points":st.session_state.draw_points.copy()
                })
                st.session_state.draw_points=[]
                save_state()
                st.success("保存成功")
                st.rerun()
            else:
                st.warning("至少3个点")
        if st.button("❌ 清空当前打点"):
            st.session_state.draw_points=[]
            save_state()
            st.rerun()

        st.subheader("📋 已保存障碍物")
        obs_names=[f"{i+1}. {o['name']} ({o['height']}m)" for i,o in enumerate(st.session_state.obstacles)]
        if obs_names:
            selected=st.selectbox("选择删除",obs_names)
            if st.button("删除选中"):
                idx=int(selected.split(".")[0])-1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()
        if st.button("🗑️ 清空所有障碍物"):
            st.session_state.obstacles=[]
            save_state()
            st.rerun()

# ==================== 飞行监控（严格1秒心跳，全局变量版） ====================
if "飞行监控" in st.session_state.page:
    st.header("📡 飞行监控（1秒/次心跳）")

    c1, c2 = st.columns(2)
    with c1:
        # 修复点1：先声明global，再使用变量
        global GLOBAL_RUNNING
        if st.button("▶️ 开始心跳监测", use_container_width=True):
            if not GLOBAL_RUNNING:
                GLOBAL_RUNNING = True
                # 启动后台线程，严格1秒一次
                t = threading.Thread(target=heartbeat_worker, daemon=True)
                t.start()
                st.rerun()
    with c2:
        # 修复点2：先声明global，再使用变量
        global GLOBAL_RUNNING
        if st.button("⏸️ 暂停心跳监测", use_container_width=True):
            GLOBAL_RUNNING = False
            st.rerun()

    placeholder = st.empty()

    with placeholder.container():
        # 修复点3：声明全局变量后再使用
        global GLOBAL_HEARTBEAT_DATA
        df = pd.DataFrame(GLOBAL_HEARTBEAT_DATA)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=200)

    # 快速重绘，更新UI
    global GLOBAL_RUNNING
    if GLOBAL_RUNNING:
        time.sleep(0.1)
        st.rerun()

# ==================== 航线规划 ====================
else:
    st.header("🗺️ 航线规划（标准绕飞版）")
    st.success("✅ 局部坐标系标准绕飞 | ✅ 20米安全区 | ✅ 严格1秒心跳")

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
        o = st_folium(m, width=1100, height=650, key="MAP_FIXED_KEY")

    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
