import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import nearest_points

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 全局时区 ====================
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

# ==================== 三种避障算法：左绕 / 右绕 / 最优弧线最短 ====================
def offset_path(base_line, poly, offset_dist, side="right"):
    coords = list(base_line.coords)
    new_coords = []
    for i in range(len(coords)-1):
        p1 = Point(coords[i])
        p2 = Point(coords[i+1])
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        if side == "right":
            off_x = -dy * offset_dist
            off_y = dx * offset_dist
        elif side == "left":
            off_x = dy * offset_dist
            off_y = -dx * offset_dist
        else:
            off_x = 0
            off_y = 0
        new_coords.append((p1.x+off_x, p1.y+off_y))
    new_coords.append(coords[-1])
    return LineString(new_coords)

def get_avoid_route(start, end, obstacles, mode="best"):
    start_pt = Point(start)
    end_pt = Point(end)
    safe_dis = 0.0006
    path = [start]
    current = start_pt

    while True:
        direct = LineString([current, end_pt])
        collide_poly = None
        for obs in obstacles:
            try:
                poly = Polygon(obs["points"])
                if direct.intersects(poly) and not direct.touches(poly):
                    collide_poly = poly
                    break
            except:
                continue
        if not collide_poly:
            break

        if mode == "left":
            avoid_line = offset_path(direct, collide_poly, safe_dis, side="left")
        elif mode == "right":
            avoid_line = offset_path(direct, collide_poly, safe_dis, side="right")
        else:
            # 最优最短弧线避障
            pt_on_poly = nearest_points(collide_poly, direct)[0]
            mid_x = pt_on_poly.x
            mid_y = pt_on_poly.y
            avoid_line = LineString([current, (mid_x+safe_dis, mid_y+safe_dis), end_pt])

        next_pt = Point(avoid_line.coords[1])
        path.append((next_pt.x, next_pt.y))
        current = next_pt
        if len(path) > 15:
            break
    path.append(end)
    return path

# ==================== 地图创建（合法瓦片+双图层+完善显示） ====================
def create_map(center_lng, center_lat, waypoints, home_point, obstacles, coord_system, temp_points):
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=19,
        tiles=None,
        control_scale=True
    )
    # 合法高德街道
    folium.TileLayer(
        tiles="https://webrd02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&lang=zh_cn&size=1&scale=1",
        attr="© 高德地图",
        name="街道地图"
    ).add_to(m)
    # 合法超清卫星
    folium.TileLayer(
        tiles="https://webst01.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&style=6",
        attr="© 高德卫星",
        name="超清卫星图"
    ).add_to(m)

    # 起飞起点标记
    if home_point:
        h_lng, h_lat = home_point if coord_system=="gcj02" else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker(
            location=[h_lat, h_lng],
            icon=folium.Icon(color="green", icon="home"),
            popup="🏠 起飞起点"
        ).add_to(m)

    # 障碍物完整显示：名称+高度+红色遮罩
    for ob in obstacles:
        pts = [[plat, plng] for plng, plat in ob["points"]]
        pop_text = f"📌{ob['name']}\n📏高度：{ob['height']} m"
        folium.Polygon(
            locations=pts,
            color="#ff3333",
            fill=True,
            fill_color="#ff0000",
            fill_opacity=0.35,
            popup=pop_text
        ).add_to(m)

    # 最终航线绘制
    if len(waypoints) >= 2:
        route = st.session_state.final_route
        line_pts = [[lat, lng] for lng, lat in route]
        folium.PolyLine(
            locations=line_pts,
            color="#0066ff",
            weight=6,
            opacity=0.8
        ).add_to(m)
        folium.Marker(line_pts[-1], icon=folium.Icon(color="red", icon="flag"), popup="🚩 终点").add_to(m)

    # 实时圈选打点预览
    for lng, lat in temp_points:
        folium.CircleMarker(
            location=[lat, lng],
            radius=5,
            color="#ff0000",
            fill=True,
            fill_color="#ff0000"
        ).add_to(m)

    folium.LayerControl(collapsed=False).add_to(m)
    return m

# ==================== 持久化记忆 ====================
STATE_FILE = "ground_station_state.json"
def save_state():
    data = {
        "home_point": st.session_state.home_point,
        "a_point": st.session_state.a_point,
        "b_point": st.session_state.b_point,
        "coord_system": st.session_state.coord_system,
        "obstacles": st.session_state.obstacles,
        "draw_points": st.session_state.draw_points
    }
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

# ==================== 初始化 ====================
st.session_state.setdefault("page", "📡 飞行监控")
loaded_data = load_state()
OFF_LNG = 118.749413
OFF_LAT = 32.234097

default_cfg = {
    "home_point": (OFF_LNG, OFF_LAT),
    "a_point": (OFF_LNG, OFF_LAT),
    "b_point": (OFF_LNG+0.0011, OFF_LAT+0.0007),
    "coord_system": "gcj02",
    "obstacles": [],
    "draw_points": [],
    "last_click": None,
    "final_route": [],
    "avoid_mode": "best"
}
for k, v in default_cfg.items():
    if k not in st.session_state:
        st.session_state[k] = v
# 载入记忆数据
for key in loaded_data:
    st.session_state[key] = loaded_data[key]

# ==================== 侧边栏功能区 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**📍 南京科技职业学院**")
    page = st.radio("功能菜单", ["📡 飞行监控", "🗺️ 航线规划"])
    st.session_state.page = page

    if page == "🗺️ 航线规划":
        st.subheader("🏠 起飞起点设置")
        hlng = st.number_input("起点经度", value=st.session_state.home_point[0], format="%.6f")
        hlat = st.number_input("起点纬度", value=st.session_state.home_point[1], format="%.6f")
        if st.button("更新并保存起点"):
            st.session_state.home_point = (hlng, hlat)
            save_state()
            st.rerun()

        st.subheader("✈️ 航线起止点")
        alng = st.number_input("A起点经度", value=st.session_state.a_point[0], format="%.6f")
        alat = st.number_input("A起点纬度", value=st.session_state.a_point[1], format="%.6f")
        blng = st.number_input("B终点经度", value=st.session_state.b_point[0], format="%.6f")
        blat = st.number_input("B终点纬度", value=st.session_state.b_point[1], format="%.6f")
        st.session_state.a_point = (alng, alat)
        st.session_state.b_point = (blng, blat)

        # 三种绕飞模式选择
        st.subheader("🧭 避障模式选择")
        st.session_state.avoid_mode = st.radio(
            "航线策略",
            ["left｜向左绕飞", "right｜向右绕飞", "best｜最优最短弧线"],
            index=2
        )

        col1, col2 = st.columns(2)
        with col1:
            if st.button("✅ 生成避障航线", use_container_width=True):
                mode_key = st.session_state.avoid_mode.split("｜")[0]
                st.session_state.final_route = get_avoid_route(
                    st.session_state.a_point,
                    st.session_state.b_point,
                    st.session_state.obstacles,
                    mode=mode_key
                )
                save_state()
                st.rerun()
        with col2:
            if st.button("❌ 清空航线", use_container_width=True):
                st.session_state.final_route = []
                save_state()
                st.rerun()

        # 障碍物圈选+高度+名称+记忆
        st.subheader("🚧 障碍物圈选设置")
        st.info(f"已打点数量：{len(st.session_state.draw_points)} | 至少3个点保存")
        obs_name = st.text_input("障碍物名称", value="实训楼")
        obs_h = st.number_input("障碍物高度(m)", min_value=5, max_value=500, value=30)

        if st.button("保存障碍物（带记忆）"):
            if len(st.session_state.draw_points) >= 3:
                st.session_state.obstacles.append({
                    "name": obs_name,
                    "height": obs_h,
                    "points": st.session_state.draw_points.copy()
                })
                st.session_state.draw_points.clear()
                save_state()
                st.rerun()
            else:
                st.warning("点位不足3个，无法生成障碍物")

        if st.button("清空本次圈选打点"):
            st.session_state.draw_points.clear()
            st.rerun()

        # 单独删除单个障碍物
        st.subheader("📋 障碍物管理")
        if st.session_state.obstacles:
            opt_list = [
                f"{i+1}.{o['name']} ｜ 高度{o['height']}m"
                for i, o in enumerate(st.session_state.obstacles)
            ]
            del_sel = st.selectbox("选择删除项", opt_list)
            if st.button("删除选中障碍物"):
                del_idx = int(del_sel.split(".")[0]) - 1
                st.session_state.obstacles.pop(del_idx)
                save_state()
                st.rerun()
        if st.button("一键清空全部障碍物"):
            st.session_state.obstacles.clear()
            save_state()
            st.rerun()

# ==================== 飞行监控页面 ====================
if page == "📡 飞行监控":
    st.header("📡 无人机心跳监控｜UTC+8 北京时间")
    st.session_state.setdefault("heartbeat_data", [])
    st.session_state.setdefault("seq", 0)
    st.session_state.setdefault("running", False)

    c1, c2 = st.columns(2)
    with c1:
        st.session_state.running = st.button("▶️ 开始心跳监测")
    with c2:
        st.session_state.running = not st.button("⏸️ 暂停心跳监测")

    if st.session_state.running:
        st.session_state.seq += 1
        st.session_state.heartbeat_data.append({
            "序号": st.session_state.seq,
            "时间": get_beijing_time_str(),
            "设备状态": "在线正常"
        })
        if len(st.session_state.heartbeat_data) > 80:
            st.session_state.heartbeat_data.pop(0)
        time.sleep(1)
        st.rerun()

    df_heart = pd.DataFrame(st.session_state.heartbeat_data)
    if not df_heart.empty:
        st.line_chart(df_heart, x="时间", y="序号", use_container_width=True)
        st.dataframe(df_heart, height=180, use_container_width=True)

# ==================== 航线规划主页面 ====================
else:
    st.header("🗺️ 航线规划｜多模式智能避障系统")
    map_ins = create_map(
        OFF_LNG, OFF_LAT,
        st.session_state.final_route,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    map_res = st_folium(map_ins, width=1150, height=680, key="main_map")

    # 地图点击圈选障碍物（精准检测）
    if map_res and map_res.get("last_clicked"):
        click_lat = map_res["last_clicked"]["lat"]
        click_lng = map_res["last_clicked"]["lng"]
        click_pt = (round(click_lng, 6), round(click_lat, 6))
        if st.session_state.last_click != click_pt:
            st.session_state.last_click = click_pt
            st.session_state.draw_points.append(click_pt)
            save_state()
            st.rerun()
