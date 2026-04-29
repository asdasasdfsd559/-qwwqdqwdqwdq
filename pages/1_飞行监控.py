import streamlit as st
import pandas as pd
import random
from datetime import datetime, timezone, timedelta
from streamlit_autorefresh import st_autorefresh

st.set_page_config(page_title="飞行监控", layout="wide")

# 北京时间
BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

st.header("📡 飞行监控（自动每秒心跳，可暂停）")

# 初始化状态
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0
    st.session_state.running = True   # 是否正在生成心跳

# 控制按钮
col1, col2 = st.columns(2)
with col1:
    if st.button("⏸️ 暂停心跳", use_container_width=True):
        st.session_state.running = False
        st.rerun()
with col2:
    if st.button("▶️ 开始心跳", use_container_width=True):
        st.session_state.running = True
        st.rerun()

# 每秒自动刷新本页面（仅影响当前页面，地图不受影响）
st_autorefresh(interval=1000, key="heartbeat_auto")

# 只有在 running 为 True 时才添加新数据
if st.session_state.running:
    st.session_state.seq += 1
    current_time = get_beijing_time_str()
    battery = round(random.uniform(70, 100), 2)
    signal = round(random.uniform(60, 95), 2)
    temperature = round(random.uniform(20, 45), 1)
    satellites = random.randint(6, 12)

    st.session_state.heartbeat_data.append({
        "序号": st.session_state.seq,
        "时间": current_time,
        "电池(%)": battery,
        "信号(%)": signal,
        "温度(°C)": temperature,
        "卫星数": satellites
    })

    # 只保留最近60条
    if len(st.session_state.heartbeat_data) > 60:
        st.session_state.heartbeat_data.pop(0)

# 展示数据
df = pd.DataFrame(st.session_state.heartbeat_data)
st.metric("累计心跳数", len(df))
if not df.empty:
    st.line_chart(df, x="时间", y="序号", color="#ff4560")
    st.dataframe(df, use_container_width=True, height=300)
else:
    st.info("点击「开始心跳」开始生成数据")

# 显示当前状态
if st.session_state.running:
    st.success("✅ 心跳运行中（每秒更新）")
else:
    st.warning("⏸️ 心跳已暂停")
