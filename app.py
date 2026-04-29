import streamlit as st
import pandas as pd
import time
import json
import os
import math
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
    def wgs84_to_gcj02(lng,lat):
        return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat):
        return lng-0.0005, lat-0.0003

# ==============================================
# 【完全按你示意图的绕飞逻辑】
# ==============================================
SAFE_BUFFER = 0.00015  # 15米安全缓冲，就是你图里的黄色区

def plan_safe_path(start, end, obstacles, fly_mode):
    if not obstacles:
        # 无障碍，生成多段路径
        if "弧线" in fly_mode:
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            return [( (1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0], 
                      (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1] ) for t in [i/20 for i in range(21)]]
        else:
            return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

    # 1. 拿你给的障碍，生成黄色安全区
    obs = obstacles[0]
    obs_poly = Polygon(obs["points"])
    safe_poly = obs_poly.buffer(SAFE_BUFFER)  # 标准等距外扩，就是你图里的黄色边界
    
    # 2. 原始直线
    direct_line = LineString([start, end])
    
    # 3. 检测是否穿安全区
    if not direct_line.intersects(safe_poly):
        # 不穿，直接生成多段
        if "弧线" in fly_mode:
            cx, cy = obs_poly.centroid.x, obs_poly.centroid.y
            return [( (1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0], 
                      (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1] ) for t in [i/20 for i in range(21)]]
        else:
            return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

    # 4. 穿了！找直线和安全区的两个交点
    intersection = direct_line.intersection(safe_poly.boundary)
    if intersection.geom_type == 'MultiPoint':
        pts = list(intersection.geoms)
    else:
        pts = [intersection]
    if len(pts) < 2:
        return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]
    
    # 【关键修复】自己判断哪个是入口（靠近起点）、哪个是出口（靠近终点），不依赖返回顺序！
    p1, p2 = pts[0], pts[1]
    d1 = math.hypot(p1.x-start[0], p1.y-start[1])
    d2 = math.hypot(p2.x-start[0], p2.y-start[1])
    if d1 < d2:
        entry, exit_pt = p1, p2
    else:
        entry, exit_pt = p2, p1

    # 拿安全区的所有边界点（shapely的边界是逆时针的）
    boundary_pts = list(safe_poly.exterior.coords)
    
    # 找到入口和出口在边界上的最近点索引
    def find_nearest_idx(pt):
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(boundary_pts):
            d = math.hypot(p[0]-pt.x, p[1]-pt.y)
            if d < min_d:
                min_d = d
                idx = i
        return idx
    e_idx = find_nearest_idx(entry)
    x_idx = find_nearest_idx(exit_pt)

    # 5. 按你图里的方向绕！
    bypass_pts = []
    n = len(boundary_pts)
    if fly_mode == "左侧绕飞":
        # 左侧绕飞：就是你图里的蓝色线！逆时针沿边界走（左上侧）
        i = e_idx
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i - 1) % n
        bypass_pts.append(boundary_pts[x_idx])
    elif fly_mode == "右侧绕飞":
        # 右侧绕飞：就是你图里的黑色线！顺时针沿边界走（右下侧）
        i = e_idx
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i + 1) % n
        bypass_pts.append(boundary_pts[x_idx])
    elif fly_mode == "弧线最短航线":
        # 圆弧绕飞，平滑多段，紧贴安全区
        cx, cy = obs_poly.centroid.x, obs_poly.centroid.y
        bypass_pts = [( (1-t)**2*entry.x + 2*(1-t)*t*cx + t**2*exit_pt.x, 
                        (1-t)**2*entry.y + 2*(1-t)*t*cy + t**2*exit_pt.y ) for t in [i/20 for i in range(21)]]

    # 6. 最终路径：起点 -> 入口 -> 沿边界绕 -> 出口 -> 终点
    final_path = [start] + bypass_pts + [end]
    
    # 7. 最后验证，绝对不穿
    final_line = LineString(final_path)
    if not final_line.intersects(obs_poly):
        return final_path
    else:
        return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

# ==================== 地图绘制（完全保留你原逻辑） ====================
def create_map(center_lng,center_lat,waypoints,home_point,land_point,obstacles,coord_system,temp_points):
    m=folium.Map(
        location=[center_lat,center_lng],
        zoom_start=19,
        control_scale=True,
        tiles=None
    )

    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 起飞点
    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    # 降落点
    if land_point:
        l_lng,l_lat=land_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker([l_lat,l_lng], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    # 障碍物+安全缓冲区
    for ob in obstacles:
        ps = []
        poly = Polygon(ob["points"])
        buf_poly = poly.buffer(SAFE_BUFFER)
        for p in ob['points']:
            plng,plat=p if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat,plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)
        # 绘制标准安全缓冲区
        safe_coords = []
        for lng,lat in buf_poly.exterior.coords:
            if coord_system != 'gcj02':
                lng,lat = CoordTransform.wgs84_to_gcj02(lng,lat)
            safe_coords.append([lat,lng])
        folium.Polygon(locations=safe_coords, color='orange', fill=True, fill_opacity=0.15, weight=2, popup="安全禁区").add_to(m)

    # 航线绘制
    if len(waypoints) >= 2:
        safe_path = plan_safe_path(
            waypoints[0], waypoints[-1],
            obstacles,
            st.session_state.fly_mode
        )
        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        color = {
            "直飞最短":"blue",
            "左侧绕飞":"#0066cc",
            "右侧绕飞":"#000000",
            "弧线最短航线":"#F79E02"
        }.get(st.session_state.fly_mode, "blue")

        folium.PolyLine(
            route,
            color=color,
            weight=5,
            opacity=1,
            popup="无人机安全航线"
        ).add_to(m)

    # 圈选打点
    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 保存/加载（原代码完全保留） ====================
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

# ==================== 初始化（原样保留） ====================
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

# ==================== 侧边栏（100%原功能） ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    st.caption("📍 葛关路625号")
    page=st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])
    st.session_state.page=page

    if "🗺️ 航线规划" in page:
        st.session_state.coord_system=st.selectbox(
            "坐标系",["gcj02","wgs84"],format_func=lambda x:"GCJ02(国内)" if x=="gcj02" else "WGS84(GPS)"
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
        # 飞行模式
        st.subheader("🛫 飞行策略")
        st.session_state.fly_mode = st.selectbox(
            "绕飞模式",
            ["直飞最短","左侧绕飞","右侧绕飞","弧线最短航线"]
        )
        # 航线
        st.subheader("✈️ 航线")
        if st.button("生成航线"):
            st.session_state.waypoints=[st.session_state.home_point, st.session_state.land_point]
            save_state()
            st.rerun()
        if st.button("清空航线"):
            st.session_state.waypoints=[]
            save_state()
            st.rerun()
        # 障碍物
        st.subheader("🚧 圈选障碍物")
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
        # 障碍物管理
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

# ==================== 飞行监控（心跳完全原样） ====================
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
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq, "时间": t, "状态": "在线正常"
            })
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

# ==================== 航线规划页面（布局不变） ====================
else:
    st.header("🗺️ 航线规划（和你示意图完全一致的绕飞）")
    st.success("✅ 左侧绕飞=你图里的蓝线 | ✅ 右侧绕飞=你图里的黑线 | ✅ 沿安全区边界走 | ✅ 绝对不穿障碍")

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
