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

st.header("📡 飞行监控（自动每秒心跳）")

# 初始化心跳数据
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0

# 每秒自动刷新本页面（仅影响当前页面，地图页面不受影响）
st_autorefresh(interval=1000, key="heartbeat_auto")

# 每次刷新时添加一条新数据
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
st.line_chart(df, x="时间", y="序号", color="#ff4560")
st.dataframe(df, use_container_width=True, height=300)
