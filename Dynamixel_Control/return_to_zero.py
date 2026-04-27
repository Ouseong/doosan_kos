import time
from dynamixel_sdk import *

# 1. 하드웨어 설정 (ID 확인 필수!)
DXL_ID = 3           # 이전 대화에서 사용하시던 ID는 3이었습니다. 1로 안되면 3으로 바꾸세요.
DEVICENAME = 'COM5'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# 2. 제어 테이블 주소
ADDR_OPERATING_MODE   = 11   # 작동 모드 설정 주소
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_LOAD     = 126
ADDR_PROFILE_VELOCITY = 112

# 3. 안전 및 목표 설정
GRIPPER_LOAD_THRESHOLD = 150 # 부하 임계값 (너무 낮으면 시작하자마자 멈추니 약간 올림)
TARGET_POS = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def twos_complement_to_int(value, bit_width=16):
    if value >= 2**bit_width: value -= 2**bit_width
    if value >= 2**(bit_width - 1): value -= 2**bit_width
    return value

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 포트 연결 실패. COM 포트 번호를 확인하세요.")
    quit()

# --- [중요] 모터 초기 세팅 ---
# 1. 일단 토크 OFF (설정 변경을 위해)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

# 2. 위치 제어 모드(3)로 강제 설정
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3)

# 3. 속도 설정 (너무 느리면 100 정도로 올리세요)
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PROFILE_VELOCITY, 50)

# 4. 토크 ON
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

print(f"🚀 [ID:{DXL_ID}] 0점 복귀 시작...")

try:
    # 0으로 이동 명령
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, TARGET_POS)

    while True:
        # 현재 위치와 부하 읽기
        pres_pos, res1, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        pres_load, res2, _ = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_LOAD)
        
        if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
            actual_load = abs(twos_complement_to_int(pres_load))
            print(f"📍 Pos: {pres_pos:4d} | Load: {actual_load:3d}", end='\r')

            # 안전 정지
            if actual_load > GRIPPER_LOAD_THRESHOLD:
                packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pres_pos)
                print(f"\n⚠️ 부하 감지로 정지.")
                break

            # 도착 확인
            if abs(TARGET_POS - pres_pos) < 20:
                print(f"\n✨ 도착 완료.")
                break
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n중단됨.")
finally:
    portHandler.closePort()