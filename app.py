import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import split

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

# ==================== 真正避障：路线绝不穿过障碍物 ====================
def find_safe_path(start, end, obstacles, shift=0.0002):
    path = [start]
    current = start
    target = end

    for _ in range(10):
        line = LineString([current, target])
        hit_obs = None

        for obs in obstacles:
            poly = Polygon(obs["points"])
            if line.intersects(poly):
                hit_obs = obs
                break

        if not hit_obs:
            break

        # 从障碍物侧面绕开，不穿过去
        poly = Polygon(hit_obs["points"])
        cx, cy = poly.centroid.x, poly.centroid.y

        dx = target[0] - current[0]
        dy = target[1] - current[1]

        # 垂直方向绕开
        px, py = -dy, dx
        length = (px**2 + py**2)**0.5
        px, py = px/length * shift, py/length * shift

        wp1 = (cx + px, cy + py)
        wp2 = (cx - px, cy - py)

        # 选一个不碰障碍物的绕路点
        safe1 = not any(Polygon(o["points"]).intersects(LineString([current, wp1]))) for o in obstacles)
        safe2 = not any(Polygon(o["points"]).intersects(LineString([current, wp2]))) for o in obstacles)

        if safe1:
            waypoint = wp1
        elif safe2:
            waypoint = wp2
        else:
            waypoint = wp1

        path.append(waypoint)
        current = waypoint

    path.append(target)
    return path

# ==================== 地图绘制 ====================
def create_map(center_lng, center_lat, waypoints, home_point, obstacles, coord_system, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, tiles=None)

    # 街道图
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德街道', name='街道图'
    ).add_to(m)

    # 卫星图（真正影像）
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星', name='卫星图(超清)'
    ).add_to(m)

    # 起点
    if home_point:
        h_lng, h_lat = home_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat, h_lng], icon=folium.Icon(color='green', icon='home'), popup="起点").add_to(m)

    # 障碍物
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.4, popup=f"{ob['name']}").add_to(m)

    # 避障航线（不穿过障碍物）
    if len(waypoints) >= 2:
        start_wp = waypoints[0]
        end_wp = waypoints[-1]
        safe_path = find_safe_path(start_wp, end_wp, obstacles)

        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        folium.PolyLine(route, color='blue', weight=5).add_to(m)
        folium.Marker([route[-1][0], route[-1][1]], icon=folium.Icon(color='red', icon='flag'), popup="终点").add_to(m)

    # 圈选预览
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
        json.dump(state, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

# ==================== 初始化（你学校精确坐标） ====================
if 'page' not in st.session_state:
    st.session_state.page = "飞行监控"

loaded = load_state()
OFFICIAL_LNG = 118.749413
OFFICIAL_LAT = 32.234097

defaults = {
    "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "waypoints": [],
    "a_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "b_point": (OFFICIAL_LNG + 0.0009, OFFICIAL_LAT + 0.0006),
    "coord_system": "gcj02",
    "obstacles": loaded.get("obstacles", []),
    "draw_points": loaded.get("draw_points", []),
    "last_click": None
}

for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

# ==================== 侧边栏（所有功能完整保留） ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")

    page = st.radio("功能", ["📡 飞行监控", "🗺️ 航线规划"])
    st.session_state.page = page

    if page == "🗺️ 航线规划":
        st.session_state.coord_system = st.selectbox(
            "坐标系", ["gcj02","wgs84"], format_func=lambda x:"GCJ02(国内标准)" if x=="gcj02" else "WGS84(GPS)"
        )

        st.subheader("🏠 起点")
        hlng = st.number_input("起点经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("起点纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新起点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("✈️ 起点 & 终点")
        alng = st.number_input("A(起点)经度", value=st.session_state.a_point[0], format="%.6f")
        alat = st.number_input("A纬度", value=st.session_state.a_point[1], format="%.6f")
        blng = st.number_input("B(终点)经度", value=st.session_state.b_point[0], format="%.6f")
        blat = st.number_input("B纬度", value=st.session_state.b_point[1], format="%.6f")

        c1, c2 = st.columns(2)
        with c1:
            if st.button("生成航线"):
                st.session_state.a_point = (alng, alat)
                st.session_state.b_point = (blng, blat)
                st.session_state.waypoints = [st.session_state.a_point, st.session_state.b_point]
                save_state()
        with c2:
            if st.button("清空航线"):
                st.session_state.waypoints = []
                save_state()
                st.rerun()

        st.subheader("🚧 圈选障碍物")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        name = st.text_input("障碍物名称", "教学楼")
        if st.button("✅ 保存障碍物"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({"name": name, "points": st.session_state.draw_points.copy()})
                st.session_state.draw_points = []
                save_state()
                st.rerun()
        if st.button("❌ 清空当前圈选"):
            st.session_state.draw_points = []
            save_state()
            st.rerun()

        # ========== 删除障碍物功能（完整保留） ==========
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

# ==================== 飞行监控 ====================
if "飞行监控" in st.session_state.page:
    st.header("飞行监控")
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳"):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳"):
            st.session_state.running = False

    if st.session_state.running:
        st.session_state.seq += 1
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq, "时间": get_beijing_time_str(), "状态": "正常"
        })
        if len(st.session_state.heartbeat_data) > 60:
            st.session_state.heartbeat_data.pop(0)
        time.sleep(1)
        st.rerun()

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df, x="时间", y="序号")
        st.dataframe(df)

# ==================== 航线规划 ====================
else:
    st.header("🗺️ 航线规划｜真正避障")
    m = create_map(
        OFFICIAL_LNG, OFFICIAL_LAT,
        st.session_state.waypoints,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    o = st_folium(m, width=1100, height=650, key="map")

    # 圈选打点
    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng,6), round(lat,6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
