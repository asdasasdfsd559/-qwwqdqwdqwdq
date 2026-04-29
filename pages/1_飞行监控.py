import streamlit as st
import pandas as pd
import random
from datetime import datetime, timezone, timedelta
from streamlit_autorefresh import st_autorefresh

BEIJING_TZ = timezone(timedelta(hours=8))
def get_beijing_time_str():
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

st.set_page_config(page_title="飞行监控", layout="wide")
st.header("📡 飞行监控（自动每秒心跳）")

if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
    st.session_state.seq = 0

st_autorefresh(interval=1000, key="hb")

st.session_state.seq += 1
st.session_state.heartbeat_data.append({
    "序号": st.session_state.seq,
    "时间": get_beijing_time_str(),
    "电池(%)": round(random.uniform(70, 100), 2),
    "信号(%)": round(random.uniform(60, 95), 2),
    "温度(°C)": round(random.uniform(20, 45), 1),
    "卫星数": random.randint(6, 12)
})
if len(st.session_state.heartbeat_data) > 60:
    st.session_state.heartbeat_data.pop(0)

df = pd.DataFrame(st.session_state.heartbeat_data)
st.metric("累计心跳数", len(df))
st.line_chart(df, x="时间", y="序号")
st.dataframe(df, use_container_width=True, height=300)
