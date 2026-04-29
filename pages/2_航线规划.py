#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import streamlit as st
import json
import os
import math
import time
import threading
import pandas as pd
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon, LinearRing

# ==================== 兼容低版本Python的comb函数 ====================
def comb(n, k):
    if k < 0 or k > n:
        return 0
    if k == 0 or k == n:
        return 1
    k = min(k, n - k)
    c = 1
    for i in range(1, k+1):
        c = c * (n - k + i) // i
    return c

# ==================== 全局变量（心跳包专用） ====================
GLOBAL_HEARTBEAT_DATA = []
GLOBAL_SEQ = 0
GLOBAL_RUNNING = False

# ==================== 初始化Streamlit ====================
st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 北京时间 ====================
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

# ==================== 安全缓冲区 ====================
SAFE_BUFFER = 0.00015   # 约15米

def get_obstacle_with_buffer(obs_poly):
    """生成障碍物的安全缓冲区（向外扩展）"""
    return obs_poly.buffer(SAFE_BUFFER)

def get_polyline_around_path(start, end, obstacle, fly_mode):
    """
    生成多段折线绕飞路径（左侧/右侧绕飞专用）
    fly_mode: "左侧绕飞" / "右侧绕飞"
    """
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    vertices = list(buffered.exterior.coords)
    
    start_pt = Point(start)
    end_pt = Point(end)
    
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
    
    start_idx = vertices.index(start_nearest)
    end_idx = vertices.index(end_nearest)
    
    if fly_mode == "左侧绕飞":
        if start_idx < end_idx:
            polyline_points = vertices[start_idx:end_idx+1]
        else:
            polyline_points = vertices[start_idx:] + vertices[:end_idx+1]
    else:
        if start_idx > end_idx:
            polyline_points = vertices[end_idx:start_idx+1][::-1]
        else:
            polyline_points = (vertices[end_idx:] + vertices[:start_idx+1])[::-1]
    
    unique_points = []
    prev = None
    for p in polyline_points:
        if prev is None or not math.isclose(p[0], prev[0]) or not math.isclose(p[1], prev[1]):
            unique_points.append(p)
            prev = p
    
    return unique_points

def get_smooth_around_path(start, end, obstacle):
    """
    生成完全顺滑的绕飞路径（弧线最短航线专用）
    """
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
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
    
    return around_points

def plan_safe_path(start, end, obstacles, fly_mode):
    """
    优化：左右绕飞为多段折线，弧线最短为顺滑曲线
    """
    if not obstacles:
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            perp_x = -dy * 0.0004
            perp_y = dx * 0.0004
            ctrl_x = mid_x + perp_x
            ctrl_y = mid_y + perp_y
            
            bezier_points = []
            for t in [i/100.0 for i in range(101)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl_x + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl_y + t**2 * end[1]
                bezier_points.append((x, y))
            return bezier_points
        return [start, end]
    
    obstacle = obstacles[0]
    obs_poly = Polygon(obstacle['points'])
    buffered = get_obstacle_with_buffer(obs_poly)
    direct_line = LineString([start, end])
    
    if not direct_line.intersects(buffered):
        if fly_mode == "弧线最短航线":
            mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            perp_x = -dy * 0.0004
            perp_y = dx * 0.0004
            ctrl_x = mid_x + perp_x
            ctrl_y = mid_y + perp_y
            
            bezier_points = []
            for t in [i/100.0 for i in range(101)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl_x + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl_y + t**2 * end[1]
                bezier_points.append((x, y))
            return bezier_points
        return [start, end]
    
    if fly_mode == "左侧绕飞":
        polyline_points = get_polyline_around_path(start, end, obstacle, "左侧绕飞")
        full_path = [start] + polyline_points + [end]
        return full_path
    
    elif fly_mode == "右侧绕飞":
        polyline_points = get_polyline_around_path(start, end, obstacle, "右侧绕飞")
        full_path = [start] + polyline_points + [end]
        return full_path
    
    elif fly_mode == "弧线最短航线":
        around_points = get_smooth_around_path(start, end, obstacle)
        full_path = [start] + around_points + [end]
        return global_bezier_smooth(full_path)
    else:
        return [start, end]

def global_smooth_path(points, num_segments=50):
    if len(points) < 2:
        return points
    
    smooth = []
    extended = [points[0]] + points + [points[-1]]
    
    for i in range(1, len(extended)-1):
        p0 = extended[i-1]
        p1 = extended[i]
        p2 = extended[i+1]
        p3 = extended[i+2] if (i+2) < len(extended) else p2
        
        for t in [j/num_segments for j in range(num_segments+1)]:
            t2 = t*t
            t3 = t2*t
            x = 0.5 * (2*p1[0] + (-p0[0]+p2[0])*t + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            y = 0.5 * (2*p1[1] + (-p0[1]+p2[1])*t + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            smooth.append((x, y))
    
    uniq = []
    prev = None
    for p in smooth:
        if prev is None or math.hypot(p[0]-prev[0], p[1]-prev[1]) > 1e-9:
            uniq.append(p)
            prev = p
    return uniq

def global_bezier_smooth(points):
    if len(points) < 3:
        return global_smooth_path(points)
    
    num_ctrl = len(points) - 1
    ctrl_pts = []
    for i in range(num_ctrl):
        t = i / num_ctrl
        ctrl_pts.append((
            (1-t)*points[i][0] + t*points[i+1][0],
            (1-t)*points[i][1] + t*points[i+1][1]
        ))
    
    bezier = []
    for t in [i/100.0 for i in range(101)]:
        x = 0.0
        y = 0.0
        n = len(ctrl_pts) - 1
        for i in range(len(ctrl_pts)):
            coeff = comb(n, i) * (1-t)**(n-i) * t**i
            x += coeff * ctrl_pts[i][0]
            y += coeff * ctrl_pts[i][1]
        bezier.append((x, y))
    return bezier

# ==================== 地图绘制 ====================
def create_map(center_lng, center_lat, waypoints, home_point, land_point, obstacles, coord_system, temp_points, fly_mode):
    m = folium.Map(
        location=[center_lat, center_lng], 
        zoom_start=st.session_state.get("map_zoom", 19),
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

    if home_point:
        h_lng, h_lat = home_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker(
            [h_lat, h_lng], 
            icon=folium.Icon(color='green', icon='home'), 
            popup="起飞点"
        ).add_to(m)
    if land_point:
        l_lng, l_lat = land_point if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker(
            [l_lat, l_lng], 
            icon=folium.Icon(color='red', icon='flag'), 
            popup="降落点"
        ).add_to(m)

    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng, plat = p if coord_system == 'gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat, plng])
        folium.Polygon(
            locations=ps, 
            color='red', 
            fill=True, 
            fill_opacity=0.5, 
            popup=f"{ob['name']}"
        ).add_to(m)

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

# ==================== 心跳包线程 ====================
def heartbeat_worker():
    global GLOBAL_HEARTBEAT_DATA, GLOBAL_SEQ, GLOBAL_RUNNING
    while GLOBAL_RUNNING:
        GLOBAL_SEQ += 1
        t = get_beijing_time_str()
        GLOBAL_HEARTBEAT_DATA.append({
            "序号": GLOBAL_SEQ, 
            "时间": t, 
            "状态": "在线正常"
        })
        if len(GLOBAL_HEARTBEAT_DATA) > 60:
            GLOBAL_HEARTBEAT_DATA.pop(0)
        time.sleep(1.0)

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
        "map_center": (OFFICIAL_LNG, OFFICIAL_LAT),
        "page": "航线规划"
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

    page=st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])
    st.session_state.page=page

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

# ==================== 飞行监控页面 ====================
if "飞行监控" in st.session_state.page:
    st.header("📡 飞行监控（1秒/次心跳）")

    c1, c2 = st.columns(2)
    
    # 全局变量声明
    global GLOBAL_RUNNING
    global GLOBAL_HEARTBEAT_DATA
    
    with c1:
        if st.button("▶️ 开始心跳监测", use_container_width=True):
            if not GLOBAL_RUNNING:
                GLOBAL_RUNNING = True
                t = threading.Thread(target=heartbeat_worker, daemon=True)
                t.start()
                st.rerun()
    with c2:
        if st.button("⏸️ 暂停心跳监测", use_container_width=True):
            GLOBAL_RUNNING = False
            st.rerun()

    placeholder = st.empty()
    with placeholder.container():
        df = pd.DataFrame(GLOBAL_HEARTBEAT_DATA)
        if not df.empty:
            st.line_chart(df, x="时间", y="序号", color="#ff4560")
            st.dataframe(df, use_container_width=True, height=200)

    if GLOBAL_RUNNING:
        time.sleep(0.1)
        st.rerun()

# ==================== 航线规划页面 ====================
else:
    st.header("🗺️ 航线规划（边界绕飞版）")
    st.success("✅ 沿障碍物边界绕飞 | ✅ 15米安全缓冲区 | ✅ 严格1秒心跳")

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
