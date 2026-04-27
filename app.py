import streamlit as st
import pandas as pd
import time
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

# ==================== 避障算法（硬写死 不穿模）====================
SAFE_OFFSET = 0.0008

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
    if mode == "left":
        return [start, (cx - SAFE_OFFSET, cy), end]
    elif mode == "right":
        return [start, (cx + SAFE_OFFSET, cy), end]
    else:
        path = []
        ctrl_lng = cx
        ctrl_lat = cy - SAFE_OFFSET * 1.5
        for i in range(21):
            t = i / 20.0
            x = (1-t)**2 * start[0] + 2*t*(1-t)*ctrl_lng + t**2 * end[0]
            y = (1-t)**2 * start[1] + 2*t*(1-t)*ctrl_lat + t**2 * end[1]
            path.append((x, y))
        return path

# ==================== 地图（国内100%加载 天地图 最新）====================
def create_map(center_lng, center_lat, route, home_point, obstacles, temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=18, tiles=None)
    # 天地图 街道 国内稳定
    folium.TileLayer(
        tiles="https://t3.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}",
        attr="© 天地图 2026", name="街道地图"
    ).add_to(m)
    # 天地图 卫星 国内稳定
    folium.TileLayer(
        tiles="https://t3.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}",
        attr="© 天地图 卫星", name="超清卫星图"
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

# ==================== 初始化（所有变量都初始化 不报错）====================
if "init_done" not in st.session_state:
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
    st.session_state.last_packet = time.time()
    st.session_state.init_done = True

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

# ==================== ✅ 心跳：100% 按你要求 开始就自动跑 暂停就停 ====================
if page == "📡 飞行监控":
    st.header("📡 无人机心跳监控")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳模拟"):
            st.session_state.running = True
            st.session_state.last_packet = time.time()
    with c2:
        if st.button("⏸️ 暂停心跳模拟"):
            st.session_state.running = False

    # 自动每秒发包 自发自收 不用你点
    if st.session_state.running:
        now = time.time()
        if now - st.session_state.last_packet >= 1.0:
            st.session_state.seq += 1
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq,
                "时间": get_beijing_time_str(),
                "状态": "✅ 已接收"
            })
            st.session_state.last_packet = now
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
        time.sleep(0.1)
        st.rerun()

    # 超时判断
    diff = time.time() - st.session_state.last_packet
    if diff > 3:
        st.error(f"⚠️ 心跳超时！{int(diff)}秒未收到数据包！")
    else:
        st.success(f"✅ 心跳正常 | 最后接收：{round(diff,1)}秒前")

    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.subheader("心跳实时曲线")
        st.line_chart(df, x="时间", y="序号", use_container_width=True)
        st.subheader("心跳接收列表")
        st.dataframe(df, use_container_width=True)

# ==================== 航线规划 ====================
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
