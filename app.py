import streamlit as st
st.set_page_config(page_title="南京科技职业学院无人机地面站", layout="wide")
st.title("南京科技职业学院无人机地面站")
st.caption("📍 葛关路625号")

pg = st.navigation([
    st.Page("pages/1_飞行监控.py", title="飞行监控", icon="📡"),
    st.Page("pages/2_航线规划.py", title="航线规划", icon="🗺️"),
])
pg.run()
