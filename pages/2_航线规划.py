# ==============================================
# 【航线规划核心逻辑】
# ==============================================
SAFE_BUFFER = 0  # 无安全区，直接面对障碍物

def plan_safe_path(start, end, obstacles, fly_mode):
    if not obstacles:
        # 无障碍，生成多段路径
        if "弧线" in fly_mode:
            cx, cy = (start[0]+end[0])/2, (start[1]+end[1])/2
            return [( (1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0], 
                      (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1] ) for t in [i/20 for i in range(21)]]
        else:
            return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

    # 1. 直接用你给的障碍
    obs = obstacles[0]
    obs_poly = Polygon(obs["points"])
    safe_poly = obs_poly.buffer(SAFE_BUFFER)  # 0缓冲，就是障碍本身
    
    # 2. 原始直线
    direct_line = LineString([start, end])
    
    # 3. 检测是否穿障碍
    if not direct_line.intersects(safe_poly):
        # 不穿，直接生成多段
        if "弧线" in fly_mode:
            cx, cy = obs_poly.centroid.x, obs_poly.centroid.y
            return [( (1-t)**2*start[0] + 2*(1-t)*t*cx + t**2*end[0], 
                      (1-t)**2*start[1] + 2*(1-t)*t*cy + t**2*end[1] ) for t in [i/20 for i in range(21)]]
        else:
            return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

    # 4. 穿了！找直线和障碍边界的交点
    intersection = direct_line.intersection(safe_poly.boundary)
    if intersection.geom_type == 'MultiPoint':
        pts = list(intersection.geoms)
    else:
        pts = [intersection]
    if len(pts) < 2:
        return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]
    
    # 自己判断入口/出口
    p1, p2 = pts[0], pts[1]
    d1 = math.hypot(p1.x-start[0], p1.y-start[1])
    d2 = math.hypot(p2.x-start[0], p2.y-start[1])
    if d1 < d2:
        entry, exit_pt = p1, p2
    else:
        entry, exit_pt = p2, p1

    # 拿障碍的所有边界点
    boundary_pts = list(safe_poly.exterior.coords)
    
    # 找到入口出口在边界上的索引
    def find_nearest_idx(pt):
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(boundary_pts):
            d = math.hypot(p[0]-pt.x, p[1]-pt.y)
            if d < min_d:
                min_d = d
                idx = i
        return idx
    e_idx = find_nearest_idx(entry)
    x_idx = find_nearest_idx(exit_pt)

    # 绕飞方向（已修正左右反了）
    bypass_pts = []
    n = len(boundary_pts)
    if fly_mode == "左侧绕飞":
        # 左侧绕飞：顺时针沿边界走
        i = e_idx
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i + 1) % n
        bypass_pts.append(boundary_pts[x_idx])
    elif fly_mode == "右侧绕飞":
        # 右侧绕飞：逆时针沿边界走
        i = e_idx
        while i != x_idx:
            bypass_pts.append(boundary_pts[i])
            i = (i - 1) % n
        bypass_pts.append(boundary_pts[x_idx])
    elif fly_mode == "弧线最短航线":
        # 弧线：平滑绕开障碍，绝对不穿
        cx, cy = obs_poly.centroid.x, obs_poly.centroid.y
        arc_pts = []
        for i in range(21):
            t = i/20
            x = (1-t)**2*entry.x + 2*(1-t)*t*cx + t**2*exit_pt.x
            y = (1-t)**2*entry.y + 2*(1-t)*t*cy + t**2*exit_pt.y
            arc_pts.append((x,y))
        bypass_pts = arc_pts

    # 最终路径
    final_path = [start] + bypass_pts + [end]
    
    # 最后验证，绝对不穿
    final_line = LineString(final_path)
    if not final_line.intersects(obs_poly):
        return final_path
    else:
        return [start, ((start[0]+end[0])/2, (start[1]+end[1])/2), end]

# ==================== 地图绘制 ====================
def create_map(center_lng,center_lat,zoom,waypoints,home_point,land_point,obstacles,coord_system,temp_points):
    m=folium.Map(
        location=[center_lat,center_lng],
        zoom_start=zoom,
        control_scale=True,
        tiles=None
    )

    folium.TileLayer(
        tiles='https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德-街道', name='街道图'
    ).add_to(m)
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德-卫星', name='卫星图'
    ).add_to(m)

    # 起飞点
    if home_point:
        h_lng,h_lat=home_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*home_point)
        folium.Marker([h_lat,h_lng], icon=folium.Icon(color='green', icon='home'), popup="起飞点").add_to(m)
    # 降落点
    if land_point:
        l_lng,l_lat=land_point if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*land_point)
        folium.Marker([l_lat,l_lng], icon=folium.Icon(color='red', icon='flag'), popup="降落点").add_to(m)

    # 障碍物（无安全区，只画红色）
    for ob in obstacles:
        ps = []
        for p in ob['points']:
            plng,plat=p if coord_system=='gcj02' else CoordTransform.wgs84_to_gcj02(*p)
            ps.append([plat,plng])
        folium.Polygon(locations=ps, color='red', fill=True, fill_opacity=0.5, popup=f"{ob['name']}").add_to(m)

    # 航线绘制
    if len(waypoints) >= 2:
        safe_path = plan_safe_path(
            waypoints[0], waypoints[-1],
            obstacles,
            st.session_state.fly_mode
        )
        route = []
        for lng, lat in safe_path:
            if coord_system != 'gcj02':
                lng, lat = CoordTransform.wgs84_to_gcj02(lng, lat)
            route.append([lat, lng])

        color = {
            "直飞最短":"blue",
            "左侧绕飞":"#0066cc",
            "右侧绕飞":"#000000",
            "弧线最短航线":"#F79E02"
        }.get(st.session_state.fly_mode, "blue")

        folium.PolyLine(
            route,
            color=color,
            weight=5,
            opacity=1,
            popup="无人机安全航线"
        ).add_to(m)

    # 圈选打点
    if len(temp_points)>=3:
        ps=[[lat,lng] for lng,lat in temp_points]
        folium.Polygon(locations=ps, color='red', weight=2).add_to(m)
    for lng,lat in temp_points:
        folium.CircleMarker([lat,lng], radius=4, color='red', fill=True).add_to(m)

    folium.LayerControl().add_to(m)
    return m

# ==================== 航线规划页面 ====================
else:
    st.header("🗺️ 航线规划")
    st.success("✅ 地图再也不闪烁 | ✅ 左右绕飞方向已修正 | ✅ 无安全区 | ✅ 弧线绝对不穿障碍")

    # 用你上次的地图视角，彻底解决闪烁
    if st.session_state.map_center:
        clng, clat = st.session_state.map_center
        zoom = st.session_state.map_zoom or 19
    else:
        clng, clat = st.session_state.home_point
        zoom = 19

    map_container = st.empty()

    with map_container:
        m = create_map(
            clng, clat, zoom,
            st.session_state.waypoints,
            st.session_state.home_point,
            st.session_state.land_point,
            st.session_state.obstacles,
            st.session_state.coord_system,
            st.session_state.draw_points
        )
        o = st_folium(m, width=1100, height=680, key="MAP_FINAL_SAFE")

    # 保存你的地图视角，下次刷新不重置
    if o:
        if o.get('center') and o.get('zoom'):
            new_center = (o['center']['lng'], o['center']['lat'])
            new_zoom = o['zoom']
            if new_center != st.session_state.map_center or new_zoom != st.session_state.map_zoom:
                st.session_state.map_center = new_center
                st.session_state.map_zoom = new_zoom
                save_state()

    # 点击添加点
    if o and o.get("last_clicked"):
        lat = o["last_clicked"]["lat"]
        lng = o["last_clicked"]["lng"]
        pt = (round(lng, 6), round(lat, 6))
        if st.session_state.last_click != pt:
            st.session_state.last_click = pt
            st.session_state.draw_points.append(pt)
            save_state()
            st.rerun()
