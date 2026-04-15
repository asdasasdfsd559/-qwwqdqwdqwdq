import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 【统一】北京时间（UTC+8） ====================
BEIJING_TZ = timezone(timedelta(hours=8))

def get_beijing_time():
    return datetime.now(BEIJING_TZ)

def get_beijing_time_str():
    return get_beijing_time().strftime("%H:%M:%S")

def get_beijing_time_ms():
    return get_beijing_time().strftime("%H:%M:%S.%f")[:-3]

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng, lat):
        return lng + 0.0005, lat + 0.0003

    @staticmethod
    def gcj02_to_wgs84(lng, lat):
        return lng - 0.0005, lat - 0.0003

# ==================== 地图 ====================
def create_map(center_lng, center_lat, waypoints, home_point, obstacles, coord_system, temp_points):
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=19,
        control_scale=True,
        tiles=None
    )

    # 2026可用高德街道图
    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-2026最新街道', name='街道图(2026)'
    ).add_to(m)

    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri-2026高清卫星', name='卫星图(超清)'
    ).add_to(m)

    if home_point:
        h_lng, h_lat = home_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker(
            [h_lat, h_lng],
            icon=folium.Icon(color='green', icon='home'),
            popup="南京科技职业学院\n📍 江北新区葛关路625号/欣乐路188号"
        ).add_to(m)
        folium.Circle(radius=120, location=[h_lat, h_lng], color='green', fill=True, fill_opacity=0.3).add_to(m)

    if waypoints:
        pts = []
        for wp in waypoints:
            w_lng, w_lat = wp if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*wp)
            pts.append([w_lat, w_lng])
        folium.PolyLine(pts, color='blue', weight=4).add_to(m)

    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(
            locations=ps, color='red', fill=True, fill_opacity=0.5,
            popup=f"{ob['name']} | {ob['height']}m"
        ).add_to(m)

    if len(temp_points) >= 3:
        ps = [[lat, lng] for lng, lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=3, fill_opacity=0.3).add_to(m)
    for lng, lat in temp_points:
        folium.CircleMarker(location=[lat, lng], radius=5, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 永久保存 ====================
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
    return None

# ==================== 初始化 ====================
if 'page' not in st.session_state:
    st.session_state.page = "飞行监控"

loaded = load_state()

OFFICIAL_LNG = 118.749413
OFFICIAL_LAT = 32.234097

defaults = {
    "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "waypoints": [],
    "a_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "b_point": (OFFICIAL_LNG + 0.0010, OFFICIAL_LAT + 0.0010),
    "coord_system": "gcj02",
    "obstacles": [],
    "draw_points": [],
    "last_click": None
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
    st.caption("📍 葛关路625号 | 欣乐路188号")

    page = st.radio("功能", ["📡 飞行监控", "🗺️ 航线规划"])
    st.session_state.page = page

    if "🗺️ 航线规划" in page:
        st.session_state.coord_system = st.selectbox(
            "坐标系", ["gcj02", "wgs84"],
            format_func=lambda x: "GCJ02(国内标准)" if x == "gcj02" else "WGS84(GPS)"
        )
        st.subheader("🏫 学校中心点（精确）")
        hlng = st.number_input("经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新中心点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("✈️ 航线点")
        alng = st.number_input("A经度", value=st.session_state.a_point[0], format="%.6f")
        alat = st.number_input("A纬度", value=st.session_state.a_point[1], format="%.6f")
        blng = st.number_input("B经度", value=st.session_state.b_point[0], format="%.6f")
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

        st.subheader("🚧 圈选障碍物（点击地图）")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        height = st.number_input("高度(m)", 1, 500, 25)
        name = st.text_input("名称", "教学楼/操场")

        if st.button("✅ 保存障碍物（永久记忆）"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({
                    "name": name, "height": height, "points": st.session_state.draw_points.copy()
                })
                st.session_state.draw_points = []
                save_state()
                st.success("✅ 保存成功！关闭再打开仍存在")
                st.rerun()
            else:
                st.warning("至少3个点才能保存区域")
        if st.button("❌ 清空当前打点"):
            st.session_state.draw_points = []
            save_state()
            st.rerun()

        st.subheader("📋 已保存障碍物")
        obs_names = [f"{i+1}. {o['name']} ({o['height']}m)" for i, o in enumerate(st.session_state.obstacles)]
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
    st.header("📡 飞行监控（南京科院）")

    # ==================== 心跳监控 核心逻辑（全中国时间） ====================
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False

    # 按钮
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳监测", use_container_width=True):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳监测", use_container_width=True):
            st.session_state.running = False

    # 自动刷新 + 实时显示
    placeholder = st.empty()

    # 核心运行逻辑（使用北京时间）
    if st.session_state.running:
        st.session_state.seq += 1
        t = get_beijing_time_str()  # 中国时间
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq,
            "时间": t,
            "状态": "在线正常"
        })

    # 显示图表 + 表格
    with placeholder.container():
        df = pd.DataFrame(st.session_state.heartbeat_data)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=200)

    # 自发自收循环
    if st.session_state.running:
        time.sleep(1)
        st.rerun()

# ==================== 航线规划 —— 【修复地图闪烁】 ====================
else:
    st.header("🗺️ 航线规划（南京科院精确地图）")
    st.success("✅ 街道图(2026最新) | ✅ 卫星图(2026超清) | ✅ 无闪烁圈选")

    if st.session_state.waypoints:
        allp = [st.session_state.home_point] + st.session_state.waypoints
        clng = sum(p[0] for p in allp) / len(allp)
        clat = sum(p[1] for p in allp) / len(allp)
    else:
        clng, clat = st.session_state.home_point

    # 固定地图容器
    map_container = st.empty()
    with map_container:
        m = create_map(
            clng, clat,
            st.session_state.waypoints,
            st.session_state.home_point,
            st.session_state.obstacles,
            st.session_state.coord_system,
            st.session_state.draw_points
        )
        o = st_folium(m, width=1100, height=650, key="MAP_FIXED_KEY")

    # 打点
    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if pt != st.session_state.last_click:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
