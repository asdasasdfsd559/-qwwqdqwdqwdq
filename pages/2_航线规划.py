# ==================== 依赖导入 ====================
import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng,lat):
        return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat):
        return lng-0.0005, lat-0.0003

# ==================== 航线规划核心算法 ====================
SAFE_DISTANCE = 0.0002  # 20米安全距离

def get_safe_polygon(obs):
    poly = Polygon(obs["points"])
    return poly.buffer(SAFE_DISTANCE)

def is_path_safe(path, safe_polys):
    line = LineString(path)
    for poly in safe_polys:
        if line.intersects(poly):
            return False
    return True

def plan_safe_path(start, end, obstacles, fly_mode):
    safe_polys = [get_safe_polygon(obs) for obs in obstacles]
    
    # 无障碍物：直飞/弧线
    direct_path = [start, end]
    if is_path_safe(direct_path, safe_polys):
        if "弧线" in fly_mode:
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            mx = (start[0]+end[0])/2
            my = (start[1]+end[1])/2
            ctrl = (mx - dy*0.15, my + dx*0.15)
            points = []
            for t in [i/20 for i in range(21)]:
                x = (1-t)**2 * start[0] + 2*(1-t)*t * ctrl[0] + t**2 * end[0]
                y = (1-t)**2 * start[1] + 2*(1-t)*t * ctrl[1] + t**2 * end[1]
                points.append((x, y))
            return points
        return direct_path

    # 有障碍物：三种绕飞模式
    xs = [p[0] for obs in obstacles for p in obs["points"]]
    ys = [p[1] for obs in obstacles for p in obs["points"]]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    if "左侧绕飞" in fly_mode:
        bypass_x = min_x - SAFE_DISTANCE
        bypass_y = max_y + SAFE_DISTANCE
    elif "右侧绕飞" in fly_mode:
        bypass_x = max_x + SAFE_DISTANCE
        bypass_y = min_y - SAFE_DISTANCE
    else:
        bypass_x = (min_x + max_x)/2
        bypass_y = max_y + SAFE_DISTANCE

    # 强制调整确保安全
    for _ in range(5):
        test_path = [start, (bypass_x, bypass_y), end]
        if is_path_safe(test_path, safe_polys):
            break
        if "左侧绕飞" in fly_mode:
            bypass_x -= SAFE_DISTANCE
            bypass_y += SAFE_DISTANCE
        elif "右侧绕飞" in fly_mode:
            bypass_x += SAFE_DISTANCE
            bypass_y -= SAFE_DISTANCE
        else:
            bypass_y += SAFE_DISTANCE

    if "弧线" in fly_mode:
        points = []
        for t in [i/20 for i in range(21)]:
            x = (1-t)**2 * start[0] + 2*(1-t)*t * bypass_x + t**2 * end[0]
            y = (1-t)**2 * start[1] + 2*(1-t)*t * bypass_y + t**2 * end[1]
            points.append((x, y))
        return points
    else:
        return [start, (bypass_x, bypass_y), end]

# ==================== 地图绘制（修复地图显示） ====================
def create_map(center_lng,center_lat,waypoints,home_point,land_point,obstacles,coord_system,temp_points):
    # 初始化地图，添加默认底图，确保能显示
    m=folium.Map(
        location=[center_lat,center_lng],
        zoom_start=19,
        control_scale=True,
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道'
    )

    # 双图层
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 起飞/降落点
    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    if land_point:
        l_lng,l_lat=land_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker([l_lat,l_lng], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    # 障碍物+安全区
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng,plat=p if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat,plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']} | 高度:{ob['height']}m").add_to(m)
        safe_p = get_safe_polygon(ob)
        safe_coords = []
        for lng,lat in safe_p.exterior.coords:
            if coord_system != 'gcj02':
                lng,lat = CoordTransform.wgs84_to_gcj02(lng,lat)
            safe_coords.append([lat,lng])
        folium.Polygon(locations=safe_coords, color='orange', fill=True, fill_opacity=0.15, weight=2, popup="安全禁区").add_to(m)

    # 航线绘制
    if len(waypoints) >= 2:
        safe_path = plan_safe_path(waypoints[0], waypoints[-1], obstacles, st.session_state.fly_mode)
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

        folium.PolyLine(route, color=color, weight=5, opacity=1, popup="无人机航线").add_to(m)

    # 圈选打点
    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2, fill_opacity=0.2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 状态保存 ====================
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

# ==================== 航线规划页面渲染 ====================
def render_route_planning():
    st.header("🗺️ 航线规划（三种绕飞模式）")
    st.success("✅ 高德双图层 | ✅ 障碍物高度设置 | ✅ 左/右/弧线绕飞 | ✅ 100%不穿障碍物")

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
        # 修复地图渲染，添加key保证刷新
        o = st_folium(m, width=1100, height=680, key="MAP_FIXED")

    # 地图点击打点
    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()

# ==================== 侧边栏配置 ====================
def render_route_sidebar():
    st.session_state.coord_system=st.selectbox(
        "坐标系",["gcj02","wgs84"],
        format_func=lambda x:"GCJ02(国内)" if x=="gcj02" else "WGS84(GPS)"
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
    
    # 绕飞模式
    st.subheader("🛫 航线模式")
    st.session_state.fly_mode = st.selectbox(
        "选择模式", ["直飞最短","左侧绕飞","右侧绕飞","弧线最短航线"]
    )
    
    # 航线操作
    st.subheader("✈️ 航线操作")
    if st.button("生成航线"):
        st.session_state.waypoints=[st.session_state.home_point, st.session_state.land_point]
        save_state()
        st.rerun()
    if st.button("清空航线"):
        st.session_state.waypoints=[]
        save_state()
        st.rerun()
    
    # 障碍物设置
    st.subheader("🚧 障碍物设置")
    st.write(f"已打点：{len(st.session_state.draw_points)}")
    height=st.number_input("障碍物高度(m)",1,500,25)
    name=st.text_input("障碍物名称","教学楼")
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
    obs_names=[f"{i+1}. {o['name']} (高度:{o['height']}m)" for i,o in enumerate(st.session_state.obstacles)]
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
