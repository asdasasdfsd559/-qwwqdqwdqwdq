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

# 北京时间
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# 坐标转换
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng,lat): return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat): return lng-0.0005, lat-0.0003

# --------------------------
# 核心：强硬避障 三种模式
# --------------------------
SAFE_DIST = 0.0009

# 左绕
def make_left_route(start, end, obs):
    c = obs.centroid
    return [
        start,
        (c.x - SAFE_DIST, c.y + SAFE_DIST),
        end
    ]

# 右绕
def make_right_route(start, end, obs):
    c = obs.centroid
    return [
        start,
        (c.x + SAFE_DIST, c.y - SAFE_DIST),
        end
    ]

# 弧线最优
def make_arc_route(start, end, obs):
    c = obs.centroid
    pts = []
    for t in [0, 0.15, 0.3, 0.5, 0.7, 0.85, 1]:
        x = (1-t)**2 * start[0] + 2*t*(1-t)*c.x + t*t * end[0]
        y = (1-t)**2 * start[1] + 2*t*(1-t)*c.y + t*t * end[1]
        pts.append((x, y))
    return pts

# 总判断：只要相交就强制绕行
def get_final_route(start, end, obstacles, mode):
    line = LineString([start, end])
    for d in obstacles:
        try:
            poly = Polygon(d["points"])
            if line.crosses(poly) or line.intersects(poly):
                if mode == "left":
                    return make_left_route(start, end, poly)
                elif mode == "right":
                    return make_right_route(start, end, poly)
                elif mode == "arc":
                    return make_arc_route(start, end, poly)
        except:
            continue
    # 无障碍物就直飞
    return [start, end]

# --------------------------
# 地图 正常显示 无报错
# --------------------------
def create_map(center_lng,center_lat,waypoints,home_point,obstacles,coord_system,temp_points):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=19, tiles=None)

    folium.TileLayer(
        tiles="https://webrd02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&lang=zh_cn&size=1",
        attr="© 高德地图",
        name="街道地图"
    ).add_to(m)
    folium.TileLayer(
        tiles="https://webst02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&style=6",
        attr="© 高德卫星",
        name="超清卫星"
    ).add_to(m)

    # 起点
    if home_point:
        h_lng,h_lat = home_point if coord_system=="gcj02" else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color="green",icon="home"),popup="起飞起点").add_to(m)

    # 障碍物
    for o in obstacles:
        p = [[plat,plng] for plng,plat in o["points"]]
        pop = f"{o['name']} | 高度：{o['height']}m"
        folium.Polygon(locations=p,color="red",fill=True,fill_opacity=0.35,popup=pop).add_to(m)

    # 航线
    if len(waypoints)>=2:
        folium.PolyLine([[lat,lng] for lng,lat in waypoints],color="#0066ff",weight=6).add_to(m)
        folium.Marker([waypoints[-1][1],waypoints[-1][0]],icon=folium.Icon(color="red",icon="flag")).add_to(m)

    # 圈选点
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng],radius=5,color="red",fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# --------------------------
# 持久化记忆
# --------------------------
STATE_FILE = "ground_station_state.json"
def save_state():
    data = {
        "home_point":st.session_state.home_point,
        "a_point":st.session_state.a_point,
        "b_point":st.session_state.b_point,
        "coord_system":st.session_state.coord_system,
        "obstacles":st.session_state.obstacles,
        "draw_points":st.session_state.draw_points
    }
    with open(STATE_FILE,"w",encoding="utf-8") as f:
        json.dump(data,f,ensure_ascii=False,indent=2)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE,"r",encoding="utf-8") as f:
            return json.load(f)
    return {}

# --------------------------
# 初始化
# --------------------------
st.session_state.clear()
off_lng = 118.749413
off_lat = 32.234097

st.session_state.home_point = (off_lng, off_lat)
st.session_state.a_point = (off_lng, off_lat)
st.session_state.b_point = (off_lng+0.0012, off_lat+0.0008)
st.session_state.coord_system = "gcj02"
st.session_state.obstacles = []
st.session_state.draw_points = []
st.session_state.last_click = None
st.session_state.route = []
st.session_state.mode = "arc"

loaded = load_state()
for k in loaded:
    st.session_state[k] = loaded[k]

# --------------------------
# 侧边栏
# --------------------------
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    page = st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])

    if page == "🗺️ 航线规划":
        st.subheader("起止点")
        st.session_state.a_point = (
            st.number_input("A经度",value=st.session_state.a_point[0],format="%.6f"),
            st.number_input("A纬度",value=st.session_state.a_point[1],format="%.6f")
        )
        st.session_state.b_point = (
            st.number_input("B经度",value=st.session_state.b_point[0],format="%.6f"),
            st.number_input("B纬度",value=st.session_state.b_point[1],format="%.6f")
        )

        st.subheader("避障模式")
        st.session_state.mode = st.radio("",["left 向左绕","right 向右绕","arc 弧线最优"])

        if st.button("✅ 生成航线",use_container_width=True):
            m = st.session_state.mode.split(" ")[0]
            st.session_state.route = get_final_route(
                st.session_state.a_point,
                st.session_state.b_point,
                st.session_state.obstacles,
                m
            )
            save_state()
            st.rerun()

        if st.button("清空航线"):
            st.session_state.route = []
            st.rerun()

        st.subheader("障碍物")
        st.info(f"已打点：{len(st.session_state.draw_points)}")
        name = st.text_input("名称","实训楼")
        h = st.number_input("高度(m)",1,500,30)

        if st.button("保存障碍物"):
            if len(st.session_state.draw_points)>=3:
                st.session_state.obstacles.append({
                    "name":name,"height":h,"points":st.session_state.draw_points.copy()
                })
                st.session_state.draw_points = []
                save_state()
                st.rerun()

        if st.button("清空本次打点"):
            st.session_state.draw_points = []

        # 单独删除
        st.subheader("管理障碍物")
        if st.session_state.obstacles:
            lst = [f"{i+1}.{x['name']}({x['height']}m)" for i,x in enumerate(st.session_state.obstacles)]
            sel = st.selectbox("选择删除",lst)
            if st.button("删除选中"):
                idx = int(sel.split(".")[0])-1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()

# 飞行监控
if page == "📡 飞行监控":
    st.header("📡 飞行监控")
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data = []
        st.session_state.seq = 0
    if st.button("刷新记录"):
        pass
    df = pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df,x="时间",y="序号")

# 地图页面
else:
    m = create_map(
        off_lng,off_lat,
        st.session_state.route,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    map_data = st_folium(m,width=1180,height=700,key="map123")

    # 地图点击圈选
    if map_data and map_data.get("last_clicked"):
        lat = map_data["last_clicked"]["lat"]
        lng = map_data["last_clicked"]["lng"]
        pt = (round(lng,6),round(lat,6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
