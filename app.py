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

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 北京时间 ====================
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
    def wgs84_to_gcj02(lng,lat):
        return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat):
        return lng-0.0005, lat-0.0003

# ==================== 【真正】自动避障算法 ====================
def plan_safe_path(start, end, obstacles):
    safe_points = [start]
    current = start
    remaining_obstacles = obstacles.copy()
    
    for _ in range(5):
        line = LineString([current, end])
        hit = None
        for obs in remaining_obstacles:
            poly = Polygon(obs["points"])
            if line.intersects(poly):
                hit = obs
                break
        if not hit:
            break
            
        poly = Polygon(hit["points"])
        cx = poly.centroid.x
        cy = poly.centroid.y
        dx = end[0] - current[0]
        dy = end[1] - current[1]
        perp_dx = -dy
        perp_dy = dx
        
        step = 0.00018
        p1 = (cx + perp_dx * step, cy + perp_dy * step)
        p2 = (cx - perp_dx * step, cy - perp_dy * step)
        
        line1 = LineString([current, p1])
        line2 = LineString([current, p2])
        hit1 = any(LineString([current, p1]).intersects(Polygon(o["points"])) for o in remaining_obstacles)
        hit2 = any(LineString([current, p2]).intersects(Polygon(o["points"])) for o in remaining_obstacles)
        
        if not hit1:
            waypoint = p1
        elif not hit2:
            waypoint = p2
        else:
            waypoint = p1
            
        safe_points.append(waypoint)
        current = waypoint
        remaining_obstacles = [o for o in remaining_obstacles if o != hit]
    
    safe_points.append(end)
    return safe_points

# ==================== 地图（移除所有圆点圈） ====================
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

    # 高德卫星图（真·卫星图）
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星图', name='卫星图(超清)'
    ).add_to(m)

    # 起飞点（仅图标，无圆圈）
    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker(
            [h_lat,h_lng],
            icon=folium.Icon(color='green', icon='home'),
            popup="🏠 起飞点"
        ).add_to(m)

    # 降落点（仅图标，无圆圈）
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

    # 自动避障航线
    safe_path = []
    if len(waypoints) >= 2:
        start_pt = waypoints[0]
        end_pt = waypoints[-1]
        safe_path = plan_safe_path(start_pt, end_pt, obstacles)
        
        route = []
        for (lng, lat) in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])
        
        folium.PolyLine(route, color='blue', weight=5, opacity=0.9).add_to(m)

    # 圈选打点
    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2, fill_opacity=0.2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker(location=[lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 保存/加载 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
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
    st.session_state.page="飞行监控"

loaded = load_state()

OFFICIAL_LNG = 118.749413
OFFICIAL_LAT = 32.234097

defaults = {
    "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "land_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
    "waypoints": [],
    "a_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "b_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
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

# ==================== 飞行监控（完全未改动） ====================
if "飞行监控" in st.session_state.page:
    st.header("📡 飞行监控")

    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳监测", use_container_width=True):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳监测", use_container_width=True):
            st.session_state.running = False

    placeholder = st.empty()

    if st.session_state.running:
        st.session_state.seq += 1
        t = get_beijing_time_str()
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq, "时间": t, "状态": "在线正常"
        })
        if len(st.session_state.heartbeat_data) > 60:
            st.session_state.heartbeat_data.pop(0)

    with placeholder.container():
        df = pd.DataFrame(st.session_state.heartbeat_data)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=200)

    if st.session_state.running:
        time.sleep(1)
        st.rerun()

# ==================== 航线规划 ====================
else:
    st.header("🗺️ 航线规划（自动避障）")
    st.success("✅ 卫星图｜✅ 起飞/降落点位｜✅ 圈选｜✅ 自动避障")

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
