import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon

st.set_page_config(page_title="无人机智能化应用2421 Demo", layout="wide")

# ==================== 北京时间 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 初始化 ====================
if "initialized" not in st.session_state:
    st.session_state.clear()
    st.session_state.center_lat = 32.234097
    st.session_state.center_lng = 118.749413
    st.session_state.a_lat = 32.232322
    st.session_state.a_lng = 118.749000
    st.session_state.b_lat = 32.234343
    st.session_state.b_lng = 118.749000
    st.session_state.flight_height = 50
    st.session_state.obstacles = []
    st.session_state.draw_points = []
    st.session_state.last_click = None
    st.session_state.running = False
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.last_packet_time = time.time()
    st.session_state.initialized = True

# ==================== 地图 ====================
def create_map(center_lat, center_lng, a_lat, a_lng, b_lat, b_lng, obstacles, draw_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=17, tiles=None)

    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="ArcGIS World Imagery", name="卫星地图"
    ).add_to(m)

    folium.Marker(
        [a_lat, a_lng],
        icon=folium.Icon(color="red", icon="map-pin"),
        popup="起点A"
    ).add_to(m)

    folium.Marker(
        [b_lat, b_lng],
        icon=folium.Icon(color="green", icon="map-pin"),
        popup="终点B"
    ).add_to(m)

    for obs in obstacles:
        folium.Polygon(
            locations=obs["points"],
            color="red", fill=True, fill_opacity=0.4,
            popup=f"{obs['name']} | 高度:{obs['height']}m"
        ).add_to(m)

    if len(draw_points) >= 3:
        folium.Polygon(draw_points, color="red", fill=True, fill_opacity=0.2).add_to(m)
    for lat, lng in draw_points:
        folium.CircleMarker([lat, lng], radius=4, color="red", fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("⚙️ 控制面板")

    st.subheader("📍 起点A")
    a_lat = st.number_input("A点纬度", value=st.session_state.a_lat, format="%.6f", key="a_lat")
    a_lng = st.number_input("A点经度", value=st.session_state.a_lng, format="%.6f", key="a_lng")
    if st.button("设置A点", key="set_a"):
        st.session_state.a_lat = a_lat
        st.session_state.a_lng = a_lng
        st.rerun()

    st.subheader("📍 终点B")
    b_lat = st.number_input("B点纬度", value=st.session_state.b_lat, format="%.6f", key="b_lat")
    b_lng = st.number_input("B点经度", value=st.session_state.b_lng, format="%.6f", key="b_lng")
    if st.button("设置B点", key="set_b"):
        st.session_state.b_lat = b_lat
        st.session_state.b_lng = b_lng
        st.rerun()

    st.subheader("✈️ 飞行参数")
    st.session_state.flight_height = st.slider("设定飞行高度(m)", 10, 200, 50, key="height_slider")

    st.subheader("🚧 障碍物圈选")
    st.info(f"已打点：{len(st.session_state.draw_points)}")
    obs_name = st.text_input("障碍物名称", "教学楼", key="obs_name")
    obs_height = st.number_input("高度(m)", 1, 500, 30, key="obs_height")
    
    if st.button("保存障碍物", key="save_obs"):
        if len(st.session_state.draw_points) >= 3:
            st.session_state.obstacles.append({
                "name": obs_name,
                "height": obs_height,
                "points": st.session_state.draw_points.copy()
            })
            st.session_state.draw_points = []
            st.rerun()
    
    if st.button("清空当前圈选", key="clear_draw"):
        st.session_state.draw_points = []
        st.rerun()

# ==================== 主页面 ====================
st.title("📡 无人机智能化应用2421 Demo")
tab1, tab2 = st.tabs(["地图显示", "心跳监控"])

# --- 地图 ---
with tab1:
    st.subheader("🗺️ 地图显示（校园内A/B点与障碍物）")
    map_obj = create_map(
        st.session_state.center_lat,
        st.session_state.center_lng,
        st.session_state.a_lat,
        st.session_state.a_lng,
        st.session_state.b_lat,
        st.session_state.b_lng,
        st.session_state.obstacles,
        st.session_state.draw_points
    )
    map_output = st_folium(map_obj, width=1000, height=600, key="main_map")

    if map_output and map_output.get("last_clicked"):
        lat = map_output["last_clicked"]["lat"]
        lng = map_output["last_clicked"]["lng"]
        point = (round(lat, 6), round(lng, 6))
        if st.session_state.last_click != point:
            st.session_state.last_click = point
            st.session_state.draw_points.append(point)
            st.rerun()

# --- 心跳监控 ---
with tab2:
    st.subheader("📡 心跳包接收（UTC+8）")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始监测", key="start_hb"):
            st.session_state.running = True
            st.session_state.last_packet_time = time.time()
    with c2:
        if st.button("⏸️ 暂停监测", key="stop_hb"):
            st.session_state.running = False

    if st.session_state.running:
        now = time.time()
        if now - st.session_state.last_packet_time >= 1.0:
            st.session_state.seq += 1
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq,
                "时间": get_beijing_time_str(),
                "状态": "在线正常"
            })
            st.session_state.last_packet_time = now
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
        time.sleep(0.1)
        st.rerun()

    time_diff = time.time() - st.session_state.last_packet_time
    if time_diff > 3:
        st.error(f"⚠️ 心跳超时！{int(time_diff)}秒未收到数据包！")
    else:
        st.success(f"✅ 心跳正常 | 最后接收：{round(time_diff,1)}秒前")

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df, x="时间", y="序号")
        st.dataframe(df, use_container_width=True)
