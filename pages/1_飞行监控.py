import streamlit as st
import pandas as pd
import time
import threading
from datetime import datetime, timezone, timedelta

st.set_page_config(page_title="无人机心跳监控", layout="wide")

BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# 初始化状态
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.running = False
    st.session_state.last_beat_time = time.time()

# 后台线程函数（每秒生成一条数据）
def heartbeat_worker():
    while st.session_state.running:
        now = time.time()
        if now - st.session_state.last_beat_time >= 1.0:
            st.session_state.seq += 1
            t = get_beijing_time_str()
            st.session_state.heartbeat_data.append({
                "序号": st.session_state.seq,
                "时间": t,
                "状态": "在线正常"
            })
            if len(st.session_state.heartbeat_data) > 60:
                st.session_state.heartbeat_data.pop(0)
            st.session_state.last_beat_time = now
        time.sleep(0.2)

# 控制线程
if "heartbeat_thread" not in st.session_state:
    st.session_state.heartbeat_thread = None

st.header("📡 无人机心跳模拟器（1秒自动心跳）")

col1, col2 = st.columns(2)
with col1:
    if st.button("▶️ 开始心跳", use_container_width=True):
        if not st.session_state.running:
            st.session_state.running = True
            st.session_state.last_beat_time = time.time()
            if st.session_state.heartbeat_thread is None or not st.session_state.heartbeat_thread.is_alive():
                st.session_state.heartbeat_thread = threading.Thread(target=heartbeat_worker, daemon=True)
                st.session_state.heartbeat_thread.start()
with col2:
    if st.button("⏸️ 暂停心跳", use_container_width=True):
        st.session_state.running = False

# 显示最新数据（每0.5秒页面自动刷新一次）
placeholder = st.empty()

# 使用 st.rerun() 实现定时刷新（不影响后台线程）
if st.session_state.running:
    # 每0.5秒刷新页面，更新显示
    time.sleep(0.5)
    st.rerun()

# 显示数据
df = pd.DataFrame(st.session_state.heartbeat_data)
with placeholder.container():
    if not df.empty:
        st.metric("累计心跳数", len(df))
        st.line_chart(df, x="时间", y="序号", color="#ff4560")
        st.dataframe(df, use_container_width=True, height=300)
    else:
        st.info("点击「开始心跳」模拟无人机心跳数据")
