import streamlit as st
import pandas as pd
import time
import json
import os
import folium
from streamlit_folium import st_folium
from shapely.geometry import Point, LineString, Polygon
from datetime import datetime

st.set_page_config(layout="wide")

# ===================== 强制重置所有错乱状态 =====================
st.session_state.clear()
STATE_FILE = "obs_data.json"

# 心跳固定正确变量
heart_run = False
heart_log = []
obs_list = []
draw_points = []
route_list = []

# 读写障碍物
def load_obs():
    global obs_list
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            obs_list = json.load(f)

def save_obs():
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        json.dump(obs_list, f, ensure_ascii=False, indent=2)

load_obs()

# 北京时间
def get_now():
    return datetime.now().strftime("%H:%M:%S")

# ===================== 极简有效避障 =====================
SAFE = 0.0025

def get_route(start, end, mode):
    line = LineString([start, end])
    block = None
    for o in obs_list:
        try:
            p = Polygon(o["points"])
            if line.crosses(p) or line.intersects(p):
                block = p
                break
        except:
            continue
    if not block:
        return [start, end]

    c = block.centroid
    if mode == "left":
        return [start, (c.x-0.0025, c.y+0.0025), end]
    elif mode == "right":
        return [start, (c.x+0.0025, c.y-0.0025), end]
    else:
        return [start, (c.x, c.y-0.003), end]

# ===================== 地图 =====================
def build_map():
    m = folium.Map(location=[32.234097, 118.749413], zoom_start=18, tiles=None)
    folium.TileLayer("https://webrd02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}",attr="amap",name="街道").add_to(m)
    folium.TileLayer("https://webst02.is.autonavi.com/appmaptile?x={x}&y={y}&z={z}&style=6",attr="amap",name="卫星").add_to(m)

    # 画障碍物
    for o in obs_list:
        loc = [[lat,lng] for lng,lat in o["points"]]
        folium.Polygon(loc, color="red", fill=True, fill_opacity=0.3).add_to(m)
    # 画航线
    if len(route_list)>=2:
        folium.PolyLine([[lat,lng] for lng,lat in route_list], color="blue", weight=5).add_to(m)
    # 画选点
    for lng,lat in draw_points:
        folium.CircleMarker([lat,lng], radius=4, color="red", fill=True).add_to(m)
    folium.LayerControl().add_to(m)
    return m

# ===================== 侧边栏 =====================
with st.sidebar:
    st.title("无人机地面站")
    page = st.radio("功能", ["心跳监测", "航线规划"])

    if page == "航线规划":
        st.subheader("起止点")
        a = (st.number_input("A经度",value=118.749413,format="%.6f"),
             st.number_input("A纬度",value=32.234097,format="%.6f"))
        b = (st.number_input("B经度",value=118.751000,format="%.6f"),
             st.number_input("B纬度",value=32.234100,format="%.6f"))

        mode = st.radio("避障",["left左绕","right右绕","arc弧线"])
        key = mode[:4]

        if st.button("生成航线"):
            global route_list
            route_list = get_route(a,b,key)
            st.rerun()

        st.divider()
        st.subheader("障碍物圈选")
        st.text(f"选点：{len(draw_points)}")
        name = st.text_input("名称","建筑")
        h = st.number_input("高度",10,500,30)
        if st.button("保存障碍物"):
            if len(draw_points)>=3:
                obs_list.append({"name":name,"h":h,"points":draw_points.copy()})
                save_obs()
                draw_points.clear()
                st.rerun()

# ===================== 页面主体 =====================
if page == "心跳监测":
    st.header("心跳监测")
    col1,col2 = st.columns(2)
    with col1:
        if st.button("开始"):
            heart_run = True
    with col2:
        if st.button("暂停"):
            heart_run = False

    # 正确心跳逻辑，绝不反向
    if heart_run:
        heart_log.append({"时间":get_now(),"状态":"在线"})
        time.sleep(1)
        st.rerun()

    if heart_log:
        df = pd.DataFrame(heart_log)
        st.line_chart(df,x="时间")
else:
    m = build_map()
    res = st_folium(m,width=1200,height=700)
    if res["last_clicked"]:
        lat = res["last_clicked"]["lat"]
        lng = res["last_clicked"]["lng"]
        draw_points.append((round(lng,6),round(lat,6)))
        st.rerun()
