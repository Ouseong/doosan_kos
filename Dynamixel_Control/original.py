import os
import time 
import keyboard
from datetime import datetime
from dynamixel_sdk import *

# --- 1. 하드웨어 및 주소 설정 ---
DXL_ID = 3                
DEVICENAME = 'COM5'       
BAUDRATE = 57600          
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_LOAD           = 126
ADDR_PROFILE_VELOCITY       = 112 

GRIPPER_MINIMUM_POSITION_VALUE = 2200 
GRIPPER_MAXIMUM_POSITION_VALUE = 3900 
GRIPPER_LOAD_THRESHOLD         = 300  

# --- 2. 유틸리티 함수 (에러 방지를 위해 루프보다 위에 배치) ---
def twos_complement_to_int(value, bit_width=16):
    if value >= 2**(bit_width - 1):
        value -= 2**bit_width
    return value

def set_goal_position(position):
    # int 형변환을 통해 안전하게 전송
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, int(position))

# --- 3. 초기화 프로세스 ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ COM5 연결 실패. 포트와 ID를 확인하세요."); quit()

# 토크 OFF 상태에서 운영 모드(Position) 및 속도 설정
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3) 
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PROFILE_VELOCITY, 40) # 부드러운 이동을 위해 40 설정
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

print(f"✅ ID {DXL_ID} 준비 완료! [A]: 확장, [S]: 축소, [ESC]: 종료")

# 데이터 저장 폴더 생성
now = datetime.now().strftime("%Y%m%d_%H%M%S")
if not os.path.exists("./data/"): os.makedirs("./data/")
filename = f"./data/{now}_data.txt"

# --- 4. 메인 제어 루프 ---
try:
    while True:
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN:
            if event.name == 'esc': break
            
            # 현재 위치 읽기
            pres_pos, _, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

            if event.name == 'a':
                # 'a'를 누르면 뒤로 가지 않고 무조건 최대치(4095)를 향해 전진
                target_pos = GRIPPER_MAXIMUM_POSITION_VALUE
                action_name = "확장"
            elif event.name == 's':
                # 's'를 누르면 무조건 최소치(2300)를 향해 전진
                target_pos = GRIPPER_MINIMUM_POSITION_VALUE
                action_name = "축소"
            else:
                continue

            # 명령 전송 및 즉시 피드백
            set_goal_position(target_pos)
            print(f"\n🚀 {action_name} 시작! (Target: {target_pos})")

            # --- 실시간 모니터링 및 즉각 정지 루프 ---
            while True:
                # 데이터 수집 (통신 우선)
                curr_pos, _, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                curr_load_raw, _, _ = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_LOAD)
                curr_load = twos_complement_to_int(curr_load_raw)
                
                # [실시간 출력] 한 줄에서 계속 업데이트
                print(f"   📊 현재 상태 - 위치: {curr_pos:4d} | 부하: {abs(curr_load):3d} / {GRIPPER_LOAD_THRESHOLD}", end='\r')

                # 로그 기록
                with open(filename, "a") as f:
                    f.write(f"[{datetime.now().isoformat()}] Pos:{curr_pos} Load:{curr_load}\n")

                # 멈춤 조건 1: 과부하 감지 (물체 파지 또는 한계점 도달)
                if abs(curr_load) > GRIPPER_LOAD_THRESHOLD:
                    # 즉시 현재 위치를 목표로 재설정하여 브레이크
                    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, curr_pos)
                    print("\n" + "="*55)
                    
                    if curr_pos >= GRIPPER_MAXIMUM_POSITION_VALUE - 20:
                        print("✨ [알림] 기구의 최대 확장 한계점에 도달했습니다.")
                    elif curr_pos <= GRIPPER_MINIMUM_POSITION_VALUE + 20:
                        print("✨ [알림] 기구의 최대 폐쇄 한계점에 도달했습니다.")
                    else:
                        print(f"⚠️ [주의] 과부하 감지! 저항이 느껴져서 {action_name}을 즉시 중단했습니다.")
                    break

                # 멈춤 조건 2: 목표 지점 도달
                if abs(target_pos - curr_pos) < 20:
                    print("\n" + "="*55)
                    print(f"✅ [완료] {action_name} 동작을 정상적으로 마쳤습니다.")
                    break

                # 중간에 다른 키 입력 시 즉각 반응을 위한 미세 대기
                if keyboard.is_pressed('space'): # 스페이스바 누르면 긴급 정지
                    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, curr_pos)
                    print("\n🛑 [중단] 사용자에 의해 강제 정지되었습니다.")
                    break
                
                time.sleep(0.01)

finally:
    # 종료 시 토크 해제
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("\n👋 프로그램을 종료합니다.")