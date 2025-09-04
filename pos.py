import numpy as np

def pos(drone_pos,drone_vel,drone_angle,drop_time,detonate_delay,missile_pos):
    time_step = 0.001
    total_steps = int(20.0 / time_step)
    pos_matrix = np.zeros((total_steps, 2, 3))

    g = 9.8

    x0, y0, z0 = drone_pos
    if drone_angle != None:
        vx = drone_vel * np.cos(drone_angle)
        vy = drone_vel * np.sin(drone_angle)
    else:
        vx = -120
        vy = 0
    
    x0 = x0 + vx * (drop_time + detonate_delay)
    y0 = y0 + vy * (drop_time + detonate_delay)
    z0 = z0 - g * detonate_delay**2 * 0.5
    # 将元组拆分为单独变量
    mx, my, mz = missile_pos
    x1 = mx - mx * (drop_time + detonate_delay) * 300/np.linalg.norm(missile_pos)
    y1 = my - my * (drop_time + detonate_delay) * 300/np.linalg.norm(missile_pos)
    z1 = mz - mz * (drop_time + detonate_delay) * 300/np.linalg.norm(missile_pos)

    for i in range(total_steps):
        z0 -= 3 * time_step
        # 分别更新每个分量
        mx_step, my_step, mz_step = missile_pos
        x1 -= mx_step * time_step * 300/np.linalg.norm(missile_pos)
        y1 -= my_step * time_step * 300/np.linalg.norm(missile_pos)
        z1 -= mz_step * time_step * 300/np.linalg.norm(missile_pos)
        pos_matrix[i, 0] = [x0, y0, z0]
        pos_matrix[i, 1] = [x1, y1, z1]
    first = None
    last = None
    for i in range(total_steps):
        if test(pos_matrix[i, 0], pos_matrix[i, 1]):
            if first == None:
                first = i * time_step
            last = i * time_step
    if first == None:
        return None
    return [drop_time+detonate_delay+first, drop_time+detonate_delay+last]

def intersect(p1, p2, center):
    p1 = np.array(p1, dtype=np.float64)
    p2 = np.array(p2, dtype=np.float64)
    center = np.array(center, dtype=np.float64)
    radius = 10.0  
    
    v = p2 - p1 
    w = p1 - center 
    
    a = np.dot(v, v) 
    
    if a == 0:
        return np.dot(w, w) <= radius**2
    
    b = np.dot(v, w)
    t = -b / a
    
    if t < 0:
        closest = p1
    elif t > 1:
        closest = p2
    else:
        closest = p1 + t * v
    
    distance_squared = np.dot(closest - center, closest - center)
    return distance_squared <= radius**2
    

def test(p1,p2):
    center = (0, 200, 0)  
    radius = 7          
    height = 10         
    num_theta = 20    
    num_radial = 10       
    num_z_side = 8        

    test_points = []

    z_bottom = center[2]
    radii_bottom = np.linspace(0, radius, num_radial)
    thetas = np.linspace(0, 2 * np.pi, num_theta, endpoint=False)
    for r in radii_bottom:
        for theta in thetas:
            x = center[0] + r * np.cos(theta)
            y = center[1] + r * np.sin(theta)
            test_points.append((x, y, z_bottom))

    z_top = center[2] + height
    radii_top = np.linspace(0, radius, num_radial)
    for r in radii_top:
        for theta in thetas:
            x = center[0] + r * np.cos(theta)
            y = center[1] + r * np.sin(theta)
            test_points.append((x, y, z_top))

    z_side = np.linspace(center[2] + 1e-6, z_top - 1e-6, num_z_side)
    for z in z_side:
        for theta in thetas:
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)
            test_points.append((x, y, z))

    for p3 in test_points:
        if intersect(p2,p3,p1):
            pass
        else:
            return False
    return True


if __name__ == '__main__':
    print(pos(np.array([17800,0,1800]),120,None,1.5,3.6,np.array([20000,0,2000])))
