import streamlit as st
import pandas as pd
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

# ==================== 避障参数【小幅绕行 距离正常】 ====================
SAFE_DISTANCE = 0.0006

def calculate_avoid_path(start, end, obstacles, mode):
    main_line = LineString([start, end])
    hit_obstacle = None

    for obs in obstacles:
        try:
            poly = Polygon(obs["points"])
            if main_line.intersects(poly):
                hit_obstacle = poly
                break
        except:
            continue

    if not hit_obstacle:
        return [start, end]

    cx, cy = hit_obstacle.centroid.x, hit_obstacle.centroid.y
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    # 左绕 近距离
    if mode == "left":
        waypoint = (cx + dy * SAFE_DISTANCE, cy - dx * SAFE_DISTANCE)
        return [start, waypoint, end]
    # 右绕 近距离
    elif mode == "right":
        waypoint = (cx - dy * SAFE_DISTANCE, cy + dx * SAFE_DISTANCE)
        return [start, waypoint, end]
    # 【完美平滑贝塞尔弧线】
    else:
        path = []
        ctrl_x = cx
        ctrl_y = cy - SAFE_DISTANCE * 1.8
        for t in [i/20 for i in range(21)]:
            x = (1-t)**2 * start[0] + 2*t*(1-t)*ctrl_x + t**2 * end[0]
            y = (1-t)**2 * start[1] + 2*t*(1-t)*ctrl_y + t**2 * end[1]
            path.append((x, y))
        return path

# ==================== 【国内永久可用瓦片】街道+卫星 必加载 ====================
def create_map(center_lng, center_lat, route, home_point, obstacles, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, tiles=None)

    # 天地图 街道图层 国内稳定
    folium.TileLayer(
        tiles="https://t3.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}",
        attr="© 天地图", name="街道地图"
    ).add_to(m)
    # 天地图 卫星图层 国内稳定
    folium.TileLayer(
        tiles="https://t3.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}",
        attr="© 天地图", name="超清卫星图"
    ).add_to(m)

    if home_point:
        folium.Marker(
            [home_point[1], home_point[0]],
            icon=folium.Icon(color="green", icon="home"), popup="起飞点"
        ).add_to(m)

    for ob in obstacles:
        pts = [[plat, plng] for plng, plat in ob["points"]]
        folium.Polygon(
            locations=pts, color="red", fill=True, fill_opacity=0.4,
            popup=f"{ob['name']} | 高度:{ob['height']}m"
        ).add_to(m)

    if len(route) >= 2:
        folium.PolyLine(
            [[lat, lng] for lng, lat in route],
            color="#0066ff", weight=6
        ).add_to(m)
        folium.Marker(
            [route[-1][1], route[-1][0]],
            icon=folium.Icon(color="red", icon="flag"), popup="终点"
        ).add_to(m)

    for lng, lat in temp_points:
        folium.CircleMarker([lat, lng], radius=5, color="red", fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 初始化（干净无脏数据）====================
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
    st.session_state.running = False
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.initialized = True

# ==================== 侧边栏 ====================
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
            st.session_state.route = calculate_avoid_path(
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

# ==================== 【完全正确心跳逻辑】====================
# 点击「开始监测」= 自动持续每秒发心跳
# 点击「暂停监测」= 立刻停止，完全符合你的使用逻辑
if page == "📡 飞行监控":
    st.header("📡 无人机心跳监控｜UTC+8")

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始监测"):
            st.session_state.running = True
    with c2:
        if st.button("⏸️ 暂停监测"):
            st.session_state.running = False

    # 开启状态下：全自动每秒新增心跳，无需手动
    if st.session_state.running:
        st.session_state.seq += 1
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq,
            "时间": get_beijing_time_str(),
            "设备状态": "在线正常"
        })
        if len(st.session_state.heartbeat_data) > 60:
            st.session_state.heartbeat_data.pop(0)
        time.sleep(1)
        st.rerun()

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df, x="时间", y="序号", use_container_width=True)
        st.dataframe(df, use_container_width=True)

# ==================== 航线规划页面 ====================
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
