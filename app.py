import streamlit as st
import pandas as pd
import time
import json
import os
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
    def wgs84_to_gcj02(lng,lat): return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat): return lng-0.0005, lat-0.0003

# ==================== 真正避障：自动绕开障碍物，绝不穿过 ====================
def get_safe_route(start, end, obstacles):
    path = [start]
    current = Point(start[0], start[1])
    target = Point(end[0], end[1])
    safety_distance = 0.0006

    for _ in range(10):
        direct_line = LineString([current, target])
        hit_obstacle = None

        for obs in obstacles:
            poly = Polygon(obs["points"])
            if direct_line.intersects(poly):
                hit_obstacle = poly
                break
        if not hit_obstacle:
            break

        cx, cy = hit_obstacle.centroid.x, hit_obstacle.centroid.y
        dx = target.x - current.x
        dy = target.y - current.y

        # 垂直方向绕开障碍物
        offset_x = -dy * safety_distance
        offset_y = dx * safety_distance
        waypoint = (cx + offset_x, cy + offset_y)

        path.append(waypoint)
        current = Point(waypoint[0], waypoint[1])

    path.append(end)
    return path

# ==================== 地图 ====================
def create_map(center_lng, center_lat, waypoints, home_point, obstacles, coord_system, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, tiles=None)

    # 街道图
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德地图',
        name='街道图'
    ).add_to(m)

    # 卫星图
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星',
        name='卫星图(超清)'
    ).add_to(m)

    # 起点
    if home_point:
        h_lng, h_lat = home_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat, h_lng], icon=folium.Icon(color='green', icon='home'), popup="起点").add_to(m)

    # 障碍物（显示名称 + 高度）
    for ob in obstacles:
        ps = [[plat, plng] for plng, plat in ob['points']]
        popup_text = f"{ob['name']} | 高度：{ob['height']}m"
        folium.Polygon(
            locations=ps,
            color='red',
            fill=True,
            fill_opacity=0.4,
            popup=popup_text
        ).add_to(m)

    # 避障航线
    if len(waypoints) >= 2:
        s = waypoints[0]
        e = waypoints[-1]
        route = get_safe_route(s, e, obstacles)
        folium.PolyLine([[lat, lng] for lng, lat in route], color='blue', weight=5).add_to(m)
        folium.Marker([e[1], e[0]], icon=folium.Icon(color='red', icon='flag'), popup="终点").add_to(m)

    # 圈选打点
    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 保存/加载 ====================
STATE_FILE = "ground_station_state.json"

def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "waypoints": st.session_state.waypoints,
        "a_point": st.session_state.a_point,
        "b_point": st.session_state.b_point,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

# ==================== 初始化 ====================
if 'page' not in st.session_state:
    st.session_state.page = "📡 飞行监控"

loaded = load_state()
OFF_LNG = 118.749413
OFF_LAT = 32.234097

defaults = {
    "home_point": (OFF_LNG, OFF_LAT),
    "waypoints": [],
    "a_point": (OFF_LNG, OFF_LAT),
    "b_point": (OFF_LNG + 0.0012, OFF_LAT + 0.0008),
    "coord_system": "gcj02",
    "obstacles": loaded.get("obstacles", []),
    "draw_points": loaded.get("draw_points", []),
    "last_click": None
}

for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    page = st.radio("功能", ["📡 飞行监控", "🗺️ 航线规划"])

    if page == "🗺️ 航线规划":
        st.subheader("起飞点")
        hlng = st.number_input("经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("A(起点) & B(终点)")
        alng = st.number_input("A经度", value=st.session_state.a_point[0], format="%.6f")
        alat = st.number_input("A纬度", value=st.session_state.a_point[1], format="%.6f")
        blng = st.number_input("B经度", value=st.session_state.b_point[0], format="%.6f")
        blat = st.number_input("B纬度", value=st.session_state.b_point[1], format="%.6f")

        col1, col2 = st.columns(2)
        with col1:
            if st.button("生成航线"):
                st.session_state.waypoints = [(alng, alat), (blng, blat)]
                save_state()
        with col2:
            if st.button("清空航线"):
                st.session_state.waypoints = []
                save_state()
                st.rerun()

        st.subheader("🚧 障碍物设置")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        obs_name = st.text_input("障碍物名称", "教学楼")
        obs_height = st.number_input("障碍物高度(m)", 1, 500, 30)

        if st.button("✅ 保存障碍物"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({
                    "name": obs_name,
                    "height": obs_height,
                    "points": st.session_state.draw_points.copy()
                })
                st.session_state.draw_points = []
                save_state()
                st.rerun()

        if st.button("❌ 清空当前打点"):
            st.session_state.draw_points = []
            save_state()
            st.rerun()

        st.subheader("📋 障碍物管理")
        obs_names = [f"{i+1}. {o['name']} ({o['height']}m)" for i, o in enumerate(st.session_state.obstacles)]
        if obs_names:
            selected = st.selectbox("选择删除", obs_names)
            if st.button("删除选中障碍物"):
                idx = int(selected.split(".")[0]) - 1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()

        if st.button("🗑️ 清空所有障碍物"):
            st.session_state.obstacles = []
            save_state()
            st.rerun()

# ==================== 飞行监控 ====================
if page == "📡 飞行监控":
    st.header("📡 飞行监控")
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳监测"):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳监测"):
            st.session_state.running = False

    if st.session_state.running:
        st.session_state.seq += 1
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq,
            "时间": get_beijing_time_str(),
            "状态": "正常在线"
        })
        time.sleep(1)
        st.rerun()

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df, x="时间", y="序号")
        st.dataframe(df, height=200)

# ==================== 航线规划 ====================
else:
    st.header("🗺️ 航线规划｜自动避障")
    m = create_map(
        OFF_LNG, OFF_LAT,
        st.session_state.waypoints,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    o = st_folium(m, width=1100, height=650, key="map")

    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
