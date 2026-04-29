import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium

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

# ==================== 基础几何检测（强制要求）====================
SAFE_BUFF = 0.00015

def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)):
            xin = (y - yi) * (xj - xi) / (yj - yi + 1e-12) + xi
            if x < xin:
                inside = not inside
    return inside

def cross(o,a,b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def seg_cross(a1,a2,b1,b2):
    c1,c2 = cross(a1,a2,b1), cross(a1,a2,b2)
    c3,c4 = cross(b1,b2,a1), cross(b1,b2,a2)
    if c1*c2<0 and c3*c4<0:
        return True
    return False

def buffer_polygon(poly, dist):
    cx = sum(p[0] for p in poly)/len(poly)
    cy = sum(p[1] for p in poly)/len(poly)
    res = []
    for x,y in poly:
        dx, dy = x-cx, y-cy
        d = (dx**2+dy**2)**0.5
        if d<1e-12:
            res.append((x,y))
        else:
            res.append((x+dx/d*dist, y+dy/d*dist))
    return res

def check_line_safe(p1,p2,obstacles):
    for obs in obstacles:
        buf = buffer_polygon(obs["points"], SAFE_BUFF)
        if point_in_poly(p1,buf) or point_in_poly(p2,buf):
            return False
        for i in range(len(buf)):
            b1 = buf[i]
            b2 = buf[(i+1)%len(buf)]
            if seg_cross(p1,p2,b1,b2):
                return False
    return True

# ==================== 合规绕飞生成：左/右折线 + 圆弧 ====================
def plan_safe_path(start, end, obstacles, fly_mode):
    # 1.直线路径检测
    if check_line_safe(start, end, obstacles):
        if "弧线" not in fly_mode:
            return [start, end]

    # 取障碍包围中心
    all_p = []
    for o in obstacles:
        all_p.extend(o["points"])
    if not all_p:
        return [start, end]
    cx = sum(x for x,y in all_p)/len(all_p)
    cy = sum(y for x,y in all_p)/len(all_p)

    # 2.左右多段折线绕飞
    if fly_mode in ["左侧绕飞","右侧绕飞"]:
        mx, my = (start[0]+end[0])/2, (start[1]+end[1])/2
        dx, dy = end[0]-start[0], end[1]-start[1]
        if fly_mode == "左侧绕飞":
            mid = (mx - dy*0.22, my + dx*0.22)
        else:
            mid = (mx + dy*0.22, my - dx*0.22)
        # 强制多段线，不两点一线
        pA = ((start[0]+mid[0])*0.6, (start[1]+mid[1])*0.6)
        pB = ((mid[0]+end[0])*0.4, (mid[1]+end[1])*0.4)
        return [start, pA, mid, pB, end]

    # 3.圆弧最短绕飞（多段短线平滑）
    if fly_mode == "弧线最短航线":
        arc = []
        step = 20
        for t in range(step+1):
            r = t/step
            x = (1-r)**2*start[0] + 2*(1-r)*r*cx + r**2*end[0]
            y = (1-r)**2*start[1] + 2*(1-r)*r*cy + r**2*end[1]
            arc.append((x,y))
        return arc

    return [start, end]

# ==================== 地图绘制（完全原版不动）====================
def create_map(center_lng,center_lat,waypoints,home_point,land_point,obstacles,coord_system,temp_points):
    m=folium.Map(
        location=[center_lat,center_lng],
        zoom_start=19,
        control_scale=True,
        tiles=None
    )

    folium.TileLayer(
        tiles='httpswebrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='httpswebst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    if land_point:
        l_lng,l_lat=land_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker([l_lat,l_lng], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    for ob in obstacles:
        ps = []
        buf_p = buffer_polygon(ob["points"], SAFE_BUFF)
        for p in ob['points']:
            plng,plat=p if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat,plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5).add_to(m)
        safe_coords = []
        for lng,lat in buf_p:
            if coord_system != 'gcj02':
                lng,lat = CoordTransform.wgs84_to_gcj02(lng,lat)
            safe_coords.append([lat,lng])
        folium.Polygon(locations=safe_coords, color='orange', fill=True, fill_opacity=0.15, weight=2).add_to(m)

    if len(waypoints) >= 2:
        safe_path = plan_safe_path(waypoints[0], waypoints[-1], obstacles, st.session_state.fly_mode)
        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        color = {"直飞最短":"blue","左侧绕飞":"#0066cc","右侧绕飞":"#000000","弧线最短航线":"#F79E02"}.get(st.session_state.fly_mode,"blue")
        folium.PolyLine(route, color=color, weight=5, opacity=1).add_to(m)

    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 保存/加载 原版 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    state = {
        "home_point": st.session_state.home_point,
        "land_point": st.session_state.land_point,
        "waypoints": st.session_state.waypoints,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points,
        "fly_mode": st.session_state.fly_mode
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None

# ==================== 初始化 原版 ====================
if 'page' not in st.session_state:
    st.session_state.page="飞行监控"

loaded = load_state()
OFFICIAL_LNG = 118.749413
OFFICIAL_LAT = 32.234097

defaults = {
    "home_point": (OFFICIAL_LNG, OFFICIAL_LAT),
    "land_point": (OFFICIAL_LNG + 0.0008, OFFICIAL_LAT + 0.0005),
    "waypoints": [],
    "coord_system": "gcj02",
    "obstacles": [],
    "draw_points": [],
    "last_click": None,
    "fly_mode": "左侧绕飞"
}

for k, v in defaults.items():
    if loaded and k in loaded:
        st.session_state[k] = loaded[k]
    elif k not in st.session_state:
        st.session_state[k] = v

# ==================== 侧边栏 完全原版 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")
    page=st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])
    st.session_state.page=page

    if "🗺️ 航线规划" in page:
        st.session_state.coord_system=st.selectbox("坐标系",["gcj02","wgs84"],format_func=lambda x:"GCJ02(国内)" if x=="gcj02" else "WGS84(GPS)")
        st.subheader("🏠 起飞点")
        hlng=st.number_input("起飞经度",value=st.session_state.home_point[0],format="%.6f")
        hlat=st.number_input("起飞纬度",value=st.session_state.home_point[1],format="%.6f")
        if st.button("更新起飞点"):
            st.session_state.home_point=(hlng,hlat)
            save_state()
            st.rerun()
        st.subheader("🚩 降落点")
        llng=st.number_input("降落经度",value=st.session_state.land_point[0],format="%.6f")
        llat=st.number_input("降落纬度",value=st.session_state.land_point[1],format="%.6f")
        if st.button("更新降落点"):
            st.session_state.land_point=(llng,llat)
            save_state()
            st.rerun()
        st.subheader("🛫 飞行策略")
        st.session_state.fly_mode = st.selectbox("绕飞模式",["直飞最短","左侧绕飞","右侧绕飞","弧线最短航线"])
        st.subheader("✈️ 航线")
        if st.button("生成航线"):
            st.session_state.waypoints=[st.session_state.home_point, st.session_state.land_point]
            save_state()
            st.rerun()
        if st.button("清空航线"):
            st.session_state.waypoints=[]
            save_state()
            st.rerun()
        st.subheader("🚧 圈选障碍物")
        st.write(f"已打点：{len(st.session_state.draw_points)}")
        height=st.number_input("高度(m)",1,500,25)
        name=st.text_input("名称","教学楼")
        if st.button("✅ 保存障碍物"):
            if len(st.session_state.draw_points)>=3:
                st.session_state.obstacles.append({"name":name,"height":height,"points":st.session_state.draw_points.copy()})
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
        obs_names=[f"{i+1}. {o['name']}" for i,o in enumerate(st.session_state.obstacles)]
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

# ==================== 飞行监控 原版心跳 ====================
if "飞行监控" in st.session_state.page:
    st.header("📡 飞行监控（1秒精准心跳）")
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
        st.session_state.running = False
        st.session_state.last_beat_time = time.time()
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳", use_container_width=True):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳", use_container_width=True):
            st.session_state.running = False
    placeholder = st.empty()
    if st.session_state.running:
        now = time.time()
        if now - st.session_state.last_beat_time >= 1.0:
            st.session_state.seq += 1
            t = get_beijing_time_str()
            st.session_state.heartbeat_data.append({"序号": st.session_state.seq, "时间": t, "状态": "在线正常"})
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
            st.session_state.last_beat_time = now
        with placeholder.container():
            df = pd.DataFrame(st.session_state.heartbeat_data)
            if not df.empty:
                st.line_chart(df, x="时间", y="序号", color="#ff4560")
                st.dataframe(df, use_container_width=True, height=220)
        time.sleep(0.05)
        st.rerun()
    else:
        with placeholder.container():
            df = pd.DataFrame(st.session_state.heartbeat_data)
            if not df.empty:
                st.line_chart(df, x="时间", y="序号", color="#ff4560")
                st.dataframe(df, use_container_width=True, height=220)

# ==================== 航线规划页面 ====================
else:
    st.header("🗺️ 航线规划｜纯几何避障")
    clng, clat = st.session_state.home_point
    map_container = st.empty()
    with map_container:
        m = create_map(clng, clat,st.session_state.waypoints,st.session_state.home_point,st.session_state.land_point,st.session_state.obstacles,st.session_state.coord_system,st.session_state.draw_points)
        o = st_folium(m, width=1100, height=680, key="MAP_FINAL_SAFE")
    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
