import time
import pygame
from dynamixel_sdk import *

# 1. 하드웨어 설정
DXL_ID = 3           
DEVICENAME = 'COM5'      
BAUDRATE = 57600       
PROTOCOL_VERSION = 2.0

UNIT_TO_MNM = 3.0  
UNIT_TO_RPM = 0.229

# 2. 제어 테이블 주소
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_VELOCITY_LIMIT   = 44    
ADDR_GOAL_VELOCITY    = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_LOAD     = 126

# 3. 제어 변수 수정
MAX_RPM_LIMIT = 55.0
MAX_UNIT_LIMIT = int(MAX_RPM_LIMIT / UNIT_TO_RPM) # 240

# [수정] 5 RPM 단위를 위해 약 22 unit씩 증감 (22 * 0.229 = 5.038 RPM)
VEL_STEP_UNIT = 22  

velocity_set_unit = 22  # 초기값 약 5 RPM
current_vel_unit = 0       
torque_history, time_history = [], []
p_pos, real_vel_rpm = 0, 0.0

# [수정] Y축 범위 -1000 ~ 1000
Y_MAX, Y_MIN = 1000, -1000

def twos_complement(val, byte_size):
    if byte_size == 2: return val if val < 32768 else val - 65536
    if byte_size == 4: return val if val < 2147483648 else val - 4294967296
    return val

# 포트 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 연결 실패!"); quit()

packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 1) 
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_VELOCITY_LIMIT, MAX_UNIT_LIMIT)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

pygame.init()
WIDTH, HEIGHT = 950, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SNU Robotics - 5 RPM Step Monitor")

font_data = pygame.font.SysFont("consolas", 18)
font_label = pygame.font.SysFont("malgungothic", 15)
font_axis = pygame.font.SysFont("arial", 12)

clock = pygame.time.Clock()
start_tick = pygame.time.get_ticks()

running = True
while running:
    screen.fill((30, 33, 45)) 
    curr_t = (pygame.time.get_ticks() - start_tick) / 1000.0
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            # 5 RPM 단위(22 unit) 조절
            if event.key == pygame.K_UP: 
                velocity_set_unit = min(MAX_UNIT_LIMIT, velocity_set_unit + VEL_STEP_UNIT)
            elif event.key == pygame.K_DOWN: 
                velocity_set_unit = max(0, velocity_set_unit - VEL_STEP_UNIT)

    keys = pygame.key.get_pressed()
    target_vel_unit = velocity_set_unit if keys[pygame.K_RIGHT] else (-velocity_set_unit if keys[pygame.K_LEFT] else 0)

    if target_vel_unit != current_vel_unit:
        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, int(target_vel_unit))
        current_vel_unit = target_vel_unit

    try:
        p_load_raw = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_LOAD)[0]
        torque_history.append(twos_complement(p_load_raw, 2) * UNIT_TO_MNM)
        time_history.append(curr_t)
        p_pos = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)[0] % 4096
        real_vel_rpm = twos_complement(packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)[0], 4) * UNIT_TO_RPM
    except: pass

    # --- Header UI ---
    pygame.draw.rect(screen, (48, 52, 63), (30, 20, 890, 160), border_radius=10)
    screen.blit(font_label.render("STATUS", True, (170, 170, 180)), (60, 40))
    screen.blit(font_data.render(f"Pos : {p_pos:5d} pul", True, (255, 120, 120)), (60, 80))
    screen.blit(font_data.render(f"Trq : {torque_history[-1] if torque_history else 0:6.1f} mNm", True, (100, 200, 255)), (60, 120))

    screen.blit(font_label.render("VELOCITY CONTROL (Step: ~5 RPM)", True, (170, 170, 180)), (400, 40))
    screen.blit(font_data.render(f"SET  : {(velocity_set_unit * UNIT_TO_RPM):5.1f} RPM ({velocity_set_unit:3d} unit)", True, (120, 255, 120)), (400, 80))
    screen.blit(font_data.render(f"REAL : {real_vel_rpm:5.2f} RPM", True, (120, 255, 120)), (400, 120))

    screen.blit(font_label.render("SYSTEM LIMIT", True, (170, 170, 180)), (720, 40))
    screen.blit(font_data.render(f"MAX : {MAX_RPM_LIMIT} RPM", True, (255, 200, 50)), (720, 80))

    # --- Graph Area ---
    graph_x, graph_y, g_w, g_h = 80, 220, 820, 520
    center_y = graph_y + (g_h // 2)
    pygame.draw.rect(screen, (22, 24, 28), (graph_x, graph_y, g_w, g_h), border_radius=5)
    
    # Y축 눈금 (-1000 ~ 1000, 200단위로 더 촘촘하게 표시)
    for val in range(Y_MIN, Y_MAX + 1, 200):
        grid_y = center_y - (val / Y_MAX) * (g_h // 2 - 40)
        color = (80, 85, 95) if val == 0 else (45, 49, 58)
        pygame.draw.line(screen, color, (graph_x, grid_y), (graph_x + g_w, grid_y), 1 if val != 0 else 2)
        screen.blit(font_axis.render(f"{val}", True, (120, 125, 135)), (graph_x - 45, grid_y - 7))

    time_range = max(5.0, curr_t)
    if len(torque_history) > 1:
        points = []
        h_factor = (g_h // 2 - 40) / Y_MAX
        for t, v in zip(time_history, torque_history):
            x = graph_x + (t / time_range * g_w)
            y = center_y - (v * h_factor)
            y = max(graph_y + 10, min(graph_y + g_h - 10, y))
            points.append((x, y))
        pygame.draw.lines(screen, (0, 150, 255), False, points[::max(1, len(points)//1200)], 2)

    pygame.display.flip()
    clock.tick(60)

packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 0)
pygame.quit(); portHandler.closePort()