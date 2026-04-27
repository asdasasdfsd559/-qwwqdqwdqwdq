import streamlit as st
import pandas as pd
import time
import json
import os
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon
from datetime import datetime, timezone, timedelta

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
        return lng - 0.0005, lat - 0.0005

# ==================== 避障算法（完全不变） ====================
SAFE_OFFSET = 0.0012

def get_safe_route(start, end, obstacles, mode):
    line = LineString([start, end])
    hit = None
    for obs in obstacles:
        try:
            poly = Polygon(obs["points"])
            if line.intersects(poly):
                hit = poly
                break
        except:
            continue
    if not hit:
        return [start, end]

    cx, cy = hit.centroid.x, hit.centroid.y
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    if mode == "left":
        wp = (cx + dy * SAFE_OFFSET, cy - dx * SAFE_OFFSET)
        return [start, wp, end]
    elif mode == "right":
        wp = (cx - dy * SAFE_OFFSET, cy + dx * SAFE_OFFSET)
        return [start, wp, end]
    else:
        path = []
        for t in [i/10 for i in range(11)]:
            p1x = cx + SAFE_OFFSET
            p1y = cy + SAFE_OFFSET
            x = (1-t)**2 * start[0] + 2*t*(1-t)*p1x + t**2 * end[0]
            y = (1-t)**2 * start[1] + 2*t*(1-t)*p1y + t**2 * end[1]
            path.append((x, y))
        return path

# ==================== 地图（2026最新街道图，完全不变） ====================
def create_map(center_lng, center_lat, route, home_point, obstacles, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=18, tiles=None)

    # 2026最新高德街道图
    folium.TileLayer(
        tiles="https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
        attr="© 高德地图 2026", name="街道地图"
    ).add_to(m)
    
    folium.TileLayer(
        tiles="https://webst02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&style=6",
        attr="© 高德卫星", name="超清卫星图"
    ).add_to(m)

    if home_point:
        folium.Marker([home_point[1], home_point[0]],
                      icon=folium.Icon(color="green", icon="home"), popup="起飞点").add_to(m)

    for ob in obstacles:
        pts = [[plat, plng] for plng, plat in ob["points"]]
        folium.Polygon(pts, color="red", fill=True, fill_opacity=0.4,
                       popup=f"{ob['name']} | {ob['height']}m").add_to(m)

    if len(route) >= 2:
        folium.PolyLine([[lat, lng] for lng, lat in route], color="#0066ff", weight=6).add_to(m)
        folium.Marker([route[-1][1], route[-1][0]],
                      icon=folium.Icon(color="red", icon="flag"), popup="终点").add_to(m)

    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=5, color="red", fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 初始化（修复变量缺失） ====================
if "initialized" not in st.session_state:
    st.session_state.clear()
    OFF_LNG = 118.749413
    OFF_LAT = 32.234097
    st.session_state.home_point = (OFF_LNG, OFF_LAT)
    st.session_state.a_point = (OFF_LNG, OFF_LAT)
    st.session_state.b_point = (OFF_LNG + 0.0012, OFF_LAT + 0.0008)
    st.session_state.obstacles = []
    st.session_state.draw_points = []
    st.session_state.last_click = None
    st.session_state.route = []
    st.session_state.avoid_mode = "arc"
    
    # 心跳初始化（全部定义，防止报错）
    st.session_state.running = False
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.last_packet_time = time.time()
    st.session_state.initialized = True

# ==================== 侧边栏（完全不变） ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    page = st.radio("功能菜单", ["📡 飞行监控", "🗺️ 航线规划"])

    if page == "🗺️ 航线规划":
        st.subheader("✈️ 航线起止点")
        st.session_state.a_point = (
            st.number_input("A点经度", value=st.session_state.a_point[0], format="%.6f"),
            st.number_input("A点纬度", value=st.session_state.a_point[1], format="%.6f")
        )
        st.session_state.b_point = (
            st.number_input("B点经度", value=st.session_state.b_point[0], format="%.6f"),
            st.number_input("B点纬度", value=st.session_state.b_point[1], format="%.6f")
        )

        st.subheader("🧭 避障模式")
        st.session_state.avoid_mode = st.radio(
            "", ["left 向左绕飞", "right 向右绕飞", "arc 最优弧线"]
        )

        if st.button("✅ 生成避障航线"):
            mode = st.session_state.avoid_mode.split(" ")[0]
            st.session_state.route = get_safe_route(
                st.session_state.a_point,
                st.session_state.b_point,
                st.session_state.obstacles,
                mode
            )
            st.rerun()

        if st.button("清空航线"):
            st.session_state.route = []
            st.rerun()

        st.subheader("🚧 障碍物设置")
        st.info(f"已打点：{len(st.session_state.draw_points)}")
        obs_name = st.text_input("障碍物名称", "教学楼")
        obs_height = st.number_input("高度(m)", 5, 500, 30)

        if st.button("保存障碍物"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({
                    "name": obs_name,
                    "height": obs_height,
                    "points": st.session_state.draw_points.copy()
                })
                st.session_state.draw_points = []
                st.rerun()

        if st.button("清空当前圈选"):
            st.session_state.draw_points = []
            st.rerun()

        st.subheader("🗑️ 删除障碍物")
        if st.session_state.obstacles:
            options = [f"{i+1}.{o['name']}({o['height']}m)" for i, o in enumerate(st.session_state.obstacles)]
            selected = st.selectbox("选择删除", options)
            if st.button("删除选中项"):
                idx = int(selected.split(".")[0]) - 1
                st.session_state.obstacles.pop(idx)
                st.rerun()

# ==================== 【无报错版】心跳自发自收 + 3秒超时 ====================
if page == "📡 飞行监控":
    st.header("📡 无人机心跳监控｜UTC+8")

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳模拟"):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停心跳模拟"):
            st.session_state.running = False

    # 自动每秒发包
    if st.session_state.running:
        now = time.time()
        if now - st.session_state.last_packet_time >= 1.0:
            st.session_state.seq += 1
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq,
                "时间": get_beijing_time_str(),
                "状态": "✅ 已接收"
            })
            st.session_state.last_packet_time = now
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
        time.sleep(0.1)
        st.rerun()

    # 超时判断（安全判断）
    current_time = time.time()
    time_diff = current_time - st.session_state.last_packet_time

    if time_diff > 3:
        st.error(f"⚠️ 心跳超时！{int(time_diff)}秒未收到数据包！")
    else:
        st.success(f"✅ 心跳正常 | 最后接收：{round(time_diff,1)}秒前")

    # 可视化
    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.subheader("心跳实时曲线")
        st.line_chart(df, x="时间", y="序号", use_container_width=True)
        st.subheader("心跳接收列表")
        st.dataframe(df, use_container_width=True)

# ==================== 航线规划（完全不变） ====================
else:
    st.header("🗺️ 航线规划｜智能避障系统")
    map_obj = create_map(
        118.749413, 32.234097,
        st.session_state.route,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.draw_points
    )
    map_output = st_folium(map_obj, width=1200, height=700, key="main_map")

    if map_output and map_output.get("last_clicked"):
        lat = map_output["last_clicked"]["lat"]
        lng = map_output["last_clicked"]["lng"]
        point = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != point:
            st.session_state.last_click = point
            st.session_state.draw_points.append(point)
            st.rerun()
