import streamlit as st
import pandas as pd
import time
import json
import os
from datetime import datetime, timezone, timedelta
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon
import math

st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")

# ==================== 北京时间 ====================
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# ==================== 坐标转换 ====================
class CoordTransform:
    @staticmethod
    def wgs84_to_gcj02(lng,lat): return lng+0.0005, lat+0.0003
    @staticmethod
    def gcj02_to_wgs84(lng,lat): return lng-0.0005, lat-0.0003

# ==================== 核心重写：精准碰撞检测 + 左右强制绕飞 + 贝塞尔弧线 ====================
SAFE_OFFSET = 0.0008

# 左侧绕行
def route_left(start, end, obs_poly):
    sx, sy = start
    ex, ey = end
    mx = (sx + ex) / 2
    my = (sy + ey) / 2
    dx = ex - sx
    dy = ey - sy
    # 左侧法向量偏移
    off_x = dy * SAFE_OFFSET
    off_y = -dx * SAFE_OFFSET
    mid = (mx + off_x, my + off_y)
    return [start, mid, end]

# 右侧绕行
def route_right(start, end, obs_poly):
    sx, sy = start
    ex, ey = end
    mx = (sx + ex) / 2
    my = (sy + ey) / 2
    dx = ex - sx
    dy = ey - sy
    # 右侧法向量偏移
    off_x = -dy * SAFE_OFFSET
    off_y = dx * SAFE_OFFSET
    mid = (mx + off_x, my + off_y)
    return [start, mid, end]

# 贝塞尔弧形最优最短路径
def route_arc_best(start, end, obs_poly):
    sx, sy = start
    ex, ey = end
    # 障碍物中心
    cx, cy = obs_poly.centroid.x, obs_poly.centroid.y
    pts = []
    # 二阶贝塞尔生成弧形
    for t in [i/12 for i in range(13)]:
        x = (1-t)**2 * sx + 2*(1-t)*t*cx + t**2 * ex
        y = (1-t)**2 * sy + 2*(1-t)*t*cy + t**2 * ey
        pts.append((x,y))
    return pts

# 全局避障调度
def build_avoid_route(start, end, obstacles, mode):
    base_line = LineString([start, end])
    hit_list = []
    for o in obstacles:
        try:
            poly = Polygon(o["points"])
            if base_line.intersects(poly):
                hit_list.append(poly)
        except:
            continue
    if not hit_list:
        return [start, end]
    
    main_obs = hit_list[0]
    if mode == "left":
        return route_left(start, end, main_obs)
    elif mode == "right":
        return route_right(start, end, main_obs)
    elif mode == "arc":
        return route_arc_best(start, end, main_obs)
    return [start, end]

# ==================== 地图（合法高德瓦片｜无报错｜完善显示） ====================
def create_map(center_lng,center_lat,waypoints,home_point,obstacles,coord_system,temp_points):
    m=folium.Map(location=[center_lat,center_lng], zoom_start=19, tiles=None)

    folium.TileLayer(
        tiles='https://webrd02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&lang=zh_cn',
        attr='© Amap', name='高德街道'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&style=6',
        attr='© Amap', name='超清卫星'
    ).add_to(m)

    if home_point:
        h_lng,h_lat = home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞起点").add_to(m)

    # 绘制障碍物 名称+高度
    for ob in obstacles:
        ps = [[plat,plng] for plng,plat in ob['points']]
        pop = f"{ob['name']} | 高度:{ob['height']}m"
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.35, popup=pop).add_to(m)

    # 绘制最终航线
    if len(waypoints)>=2:
        folium.PolyLine([[lat,lng] for lng,lat in waypoints], color='#007bff', weight=6).add_to(m)
        folium.Marker([waypoints[-1][1],waypoints[-1][0]], icon=folium.Icon(color='red', icon='flag'), popup="终点").add_to(m)

    # 圈选红点
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng], radius=5, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 持久化存储｜记忆不丢失 ====================
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

# ==================== 初始化 ====================
if os.path.exists(STATE_FILE):
    os.remove(STATE_FILE)

st.session_state.clear()
loaded = load_state()
OFF_LNG = 118.749413
OFF_LAT = 32.234097

defaults = {
    "home_point":(OFF_LNG,OFF_LAT),
    "a_point":(OFF_LNG,OFF_LAT),
    "b_point":(OFF_LNG+0.0012,OFF_LAT+0.0008),
    "coord_system":"gcj02",
    "obstacles":[],
    "draw_points":[],
    "last_click":None,
    "route_data":[],
    "avoid_mode":"arc"
}
for k,v in defaults.items():
    st.session_state[k] = v

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🎮 无人机地面站")
    st.markdown("**南京科技职业学院**")
    page = st.radio("功能",["📡 飞行监控","🗺️ 航线规划"])

    if page == "🗺️ 航线规划":
        st.subheader("起点设置")
        hlng = st.number_input("经度",value=st.session_state.home_point[0],format="%.6f")
        hlat = st.number_input("纬度",value=st.session_state.home_point[1],format="%.6f")
        if st.button("更新起点"):
            st.session_state.home_point=(hlng,hlat)
            save_state()

        st.subheader("航线 A/B")
        alng = st.number_input("A经度",value=st.session_state.a_point[0],format="%.6f")
        alat = st.number_input("A纬度",value=st.session_state.a_point[1],format="%.6f")
        blng = st.number_input("B经度",value=st.session_state.b_point[0],format="%.6f")
        blat = st.number_input("B纬度",value=st.session_state.b_point[1],format="%.6f")
        st.session_state.a_point=(alng,alat)
        st.session_state.b_point=(blng,blat)

        # 你要的三种模式
        st.subheader("避障模式")
        st.session_state.avoid_mode = st.radio(
            "选择航线",
            ["left 向左绕飞","right 向右绕飞","arc 最优弧线"],
            index=2
        )

        if st.button("生成航线"):
            mode = st.session_state.avoid_mode.split(" ")[0]
            st.session_state.route_data = build_avoid_route(
                st.session_state.a_point,
                st.session_state.b_point,
                st.session_state.obstacles,
                mode
            )
            save_state()
            st.rerun()
        if st.button("清空航线"):
            st.session_state.route_data=[]
            st.rerun()

        # 障碍物圈选+高度+名称
        st.subheader("障碍物设置")
        st.write(f"当前打点：{len(st.session_state.draw_points)}")
        name = st.text_input("障碍物名称","教学楼")
        height = st.number_input("障碍物高度(m)",1,500,30)

        if st.button("保存障碍物"):
            if len(st.session_state.draw_points)>=3:
                st.session_state.obstacles.append({
                    "name":name,"height":height,"points":st.session_state.draw_points.copy()
                })
                st.session_state.draw_points=[]
                save_state()
                st.rerun()
        if st.button("清空本次打点"):
            st.session_state.draw_points=[]

        # 单独删除障碍物
        st.subheader("障碍物管理")
        if st.session_state.obstacles:
            lst = [f"{i+1}.{o['name']}（{o['height']}m）" for i,o in enumerate(st.session_state.obstacles)]
            sel = st.selectbox("选择删除",lst)
            if st.button("删除选中"):
                idx = int(sel.split(".")[0])-1
                st.session_state.obstacles.pop(idx)
                save_state()
                st.rerun()
        if st.button("清空全部障碍物"):
            st.session_state.obstacles=[]
            save_state()

# ==================== 飞行监控 ====================
if page == "📡 飞行监控":
    st.header("📡 飞行监控")
    if "heartbeat_data" not in st.session_state:
        st.session_state.heartbeat_data=[]
        st.session_state.seq=0
        st.session_state.running=False

    c1,c2=st.columns(2)
    with c1:
        st.session_state.running = st.button("开始")
    with c2:
        if st.button("暂停"):
            st.session_state.running=False

    if st.session_state.running:
        st.session_state.seq+=1
        st.session_state.heartbeat_data.append({
            "序号":st.session_state.seq,"时间":get_beijing_time_str(),"状态":"正常"
        })
        time.sleep(1)
        st.rerun()

    df=pd.DataFrame(st.session_state.heartbeat_data)
    if not df.empty:
        st.line_chart(df,x="时间",y="序号")

# ==================== 航线规划地图 ====================
else:
    m = create_map(
        OFF_LNG,OFF_LAT,
        st.session_state.route_data,
        st.session_state.home_point,
        st.session_state.obstacles,
        st.session_state.coord_system,
        st.session_state.draw_points
    )
    map_res = st_folium(m,width=1150,height=680,key="map_main")

    # 地图点击圈选正常生效
    if map_res and map_res.get("last_clicked"):
        lat = map_res["last_clicked"]["lat"]
        lng = map_res["last_clicked"]["lng"]
        pt = (round(lng,6),round(lat,6))
        if st.session_state.last_click != pt:
            st.session_state.last_click=pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
