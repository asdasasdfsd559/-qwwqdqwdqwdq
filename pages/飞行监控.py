import streamlit as st
import pandas as pd
import threading
import time
import socket
import json
import random
from datetime import datetime
from collections import deque
from streamlit_autorefresh import st_autorefresh

# ==================== 无人机心跳模拟器 ====================
class DroneHeartbeatSimulator:
    def __init__(self, host='127.0.0.1', port=8888):
        self.host = host
        self.port = port
        self.running = False
        self.udp_socket = None
        self.receive_thread = None
        self.send_thread = None
        self.heartbeat_data = deque(maxlen=1000)
        self.last_heartbeat_time = None
        self.sequence_number = 0
        self.packets_received = 0

    def start(self):
        if self.running:
            return True
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind((self.host, self.port))
            self.udp_socket.settimeout(1)
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.send_thread.start()
            return True
        except Exception as e:
            st.error(f"启动模拟器失败: {e}")
            return False

    def _send_loop(self):
        while self.running:
            try:
                heartbeat = {
                    'type': 'heartbeat',
                    'sequence': self.sequence_number,
                    'timestamp': datetime.now().isoformat(),
                    'unix_timestamp': time.time(),
                    'battery': round(random.uniform(70, 100), 2),
                    'signal_strength': round(random.uniform(60, 95), 2),
                    'temperature': round(random.uniform(20, 45), 1),
                    'gps_satellites': random.randint(6, 12)
                }
                msg = json.dumps(heartbeat)
                self.udp_socket.sendto(msg.encode(), (self.host, self.port))
                self.sequence_number += 1
                time.sleep(1)
            except:
                time.sleep(0.5)

    def _receive_loop(self):
        while self.running:
            try:
                data, _ = self.udp_socket.recvfrom(4096)
                hb = json.loads(data.decode())
                hb['receive_time'] = time.time()
                self.heartbeat_data.append(hb)
                self.last_heartbeat_time = time.time()
                self.packets_received += 1
            except socket.timeout:
                continue
            except:
                continue

    def stop(self):
        self.running = False
        if self.udp_socket:
            self.udp_socket.close()

    def get_latest(self):
        return self.heartbeat_data[-1] if self.heartbeat_data else None

    def get_dataframe(self):
        return pd.DataFrame(list(self.heartbeat_data))

    def get_status(self):
        if not self.last_heartbeat_time:
            return "等待心跳"
        diff = time.time() - self.last_heartbeat_time
        if diff > 3:
            return f"❌ 超时 ({diff:.1f}秒)"
        return f"✅ 在线 (延迟 {diff:.2f}秒)"

# ==================== Streamlit 页面 ====================
st.set_page_config(page_title="飞行监控", layout="wide")
st.header("📡 飞行监控（1秒自动刷新）")

if "simulator" not in st.session_state:
    st.session_state.simulator = DroneHeartbeatSimulator()
    st.session_state.sim_running = False

col1, col2 = st.columns(2)
with col1:
    if st.button("▶️ 启动心跳模拟器", use_container_width=True):
        if st.session_state.simulator.start():
            st.session_state.sim_running = True
            st.rerun()
with col2:
    if st.button("⏸️ 停止模拟器", use_container_width=True):
        st.session_state.simulator.stop()
        st.session_state.sim_running = False
        st.rerun()

if st.session_state.sim_running:
    st_autorefresh(interval=1000, key="heartbeat_refresh")

latest = st.session_state.simulator.get_latest()
if latest:
    st.metric("连接状态", st.session_state.simulator.get_status())
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("序号", latest['sequence'])
    c2.metric("电池电量", f"{latest['battery']:.1f}%")
    c3.metric("信号强度", f"{latest['signal_strength']:.1f}%")
    c4.metric("卫星数", latest['gps_satellites'])

    df = st.session_state.simulator.get_dataframe()
    if not df.empty:
        st.subheader("历史数据")
        st.line_chart(df.set_index('timestamp')[['battery', 'signal_strength']])
        st.dataframe(df[['sequence', 'timestamp', 'battery', 'signal_strength']].tail(20))
else:
    st.info("模拟器未启动或暂无数据，点击「启动心跳模拟器」")

if st.session_state.simulator.packets_received > 0:
    st.caption(f"已接收心跳包: {st.session_state.simulator.packets_received} 个")
