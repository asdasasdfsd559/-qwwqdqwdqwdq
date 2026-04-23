import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="无人机地面站", layout="wide")

# ==================== 时间 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003
    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 避障：绝不穿过障碍物 ====================
def compute_safe_path(start, end, obstacles):
    path = [start]
    current = Point(start)
    target = Point(end)
    safety = 0.0004

    for _ in range(6):
        line = LineString([current, target])
        hit = None
        for obs in obstacles:
            poly = Polygon(obs["points"])
            if line.intersects(poly):
                hit = poly
                break
        if not hit:
            break

        cx, cy = hit.centroid.x, hit.centroid.y
        dx = target.x - current.x
        dy = target.y - current.y

        perp_dx = -dy
        perp_dy = dx
        wp = (cx + perp_dx * safety, cy + perp_dy * safety)
        path.append(wp)
        = Point(wp)

    path.append((target.x, target.y))
    return path

# ==================== 地图（修复所有错误） ====================
def create_map(center_lng, center_lat, waypoints, home_point, obstacles, coord_system, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19)

    # 起点
    if home_point:
        h_lng, h_lat = home_point if coord_system == "gcj02" else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat, h_lng], icon=folium.Icon(color="green"), popup="起点").add_to(m)

    # 障碍物
    for ob in obstacles:
        ps = []
        for p in ob["points"]:
            plng, plat = p if coord_system == "gcj02" else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(locations=ps, color="red", fill=True, fill_opacity=0.4).add_to(m)

    # 避障航线
    if len(waypoints) >= 2:
        s = waypoints[0]
        e = waypoints[-1]
        route = compute_safe_path(s, e, obstacles)
        coords = []
        for lng, lat in route:
            if coord_system != "gcj02":
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            coords.append([lat, lng])
        folium.PolyLine(coords, color="blue", weight=5).add_to(m)
        folium.Marker([e[1], e[0]], icon=folium.Icon(color="red"), popup="终点").add_to(m)

    # 圈选打点
    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=4, color="red").add_to(m)

    return m

# ==================== 保存加载 ====================
STATE_FILE = "state.json"
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
        json.dump(state, f)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

# ==================== 初始化 ====================
loaded = load_state()
defaults = {
    "page": "飞行监控",
    "home_point": (118.749413, 32.234097),
    "waypoints": [],
    "a_point": (118.749413, 32.234097),
    "b_point": (118.7503, 32.2347),
    "coord_system": "gcj02",
    "obstacles": [],
    "draw_points": [],
    "last_click": None,
    "heartbeat_data": [],
    "seq": 0,
    "running": False
}

for k, v in defaults.items():
    if k in loaded:
        st.session_state[k] = loaded[k]
    elif k not in st.session_state:
        st.session_state[k] = v

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("无人机地面站")
    page = st.radio("功能", ["飞行监控", "航线规划"])
    st.session_state.page = page

    if page == "航线规划":
        st.subheader("起飞点")
        hlng = st.number_input("经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("航线")
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

        st.subheader("障碍物")
        st.write(f"打点：{len(st.session_state.draw_points)}")
        if st.button("保存障碍物"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({"points": st.session_state.draw_points.copy()})
                st.session_state.draw_points = []
                save_state()
                st.rerun()
        if st.button("取消当前圈选"):
            st.session_state.draw_points = []
            save_state()
            st.rerun()

        st.subheader("删除障碍物")
        names = [f"障碍物 {i+1}" for i in range(len(st.session_state.obstacles))]
        if names:
            sel = st.selectbox("选择", names)
            if st.button("删除选中"):
                idx = int(sel.split()[1]) - 1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()
        if st.button("清空所有障碍物"):
            st.session_state.obstacles = []
            save_state()
            st.rerun()

# ==================== 飞行监控 ====================
if page == "飞行监控":
    st.header("飞行监控")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("开始心跳"):
            st.session_state.running = True
    with c2:
        if st.button("暂停心跳"):
            st.session_state.running = False

    if st.session_state.running:
        st.session_state.seq += 1
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq,
            "时间": get_beijing_time_str(),
            "状态": "正常"
        })
        time.sleep(1)
        st.rerun()

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df, x="时间", y="序号")
        st.dataframe(df)

# ==================== 航线规划 ====================
else:
    st.header("航线规划（避障版）")
    m = create_map(
        118.749413, 32.234097,
        st.session_state.waypoints,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    o = st_folium(m, key="map", width=1100, height=650)

    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
