import pybullet as p            # PyBullet: 물리 시뮬레이션 라이브러리
import pybullet_data            # PyBullet에서 제공하는 기본 모델 경로
import time                     # 시간 지연용
import numpy as np              # 수치 계산용 (평균 등)

# PyBullet GUI 모드로 시뮬레이터 실행
p.connect(p.GUI)

# 기본 리소스 경로 설정 (URDF 파일 등)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 중력 설정 (지구 중력: -9.8 m/s^2)
p.setGravity(0, 0, -9.8)

# 시뮬레이션 시간 간격 설정 (240 FPS)
time_step = 1. / 240.
p.setTimeStep(time_step)

# 바닥(Plane) 생성
plane_id = p.loadURDF("plane.urdf")


# [1] 드론 생성 함수 (회전 여부 포함)
def create_drone(initial_height=1.5, angular_velocity=0.0):
    """
    드론(직육면체 모형)을 생성하고, 회전 속도 설정
    :param initial_height: 드론이 낙하 시작할 높이
    :param angular_velocity: 회전 속도 (z축 기준)
    :return: 드론 객체 ID
    """
    # 충돌 모양: 박스(10cm x 10cm x 5cm)
    box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05])

    # 물리 객체(질량 포함) 생성
    drone_id = p.createMultiBody(
        baseMass=1.0,  # 1kg
        baseCollisionShapeIndex=box_shape,
        basePosition=[0, 0, initial_height]
    )

    # 초기 회전 속도 설정 (Z축 회전)
    p.resetBaseVelocity(drone_id, angularVelocity=[0, 0, angular_velocity])

    return drone_id


# [2] 시뮬레이션 실행 및 충돌 시 충격량 측정
def simulate_and_get_impulse(drone_id, max_steps=1000):
    """
    드론이 바닥에 충돌할 때 발생한 충격량 측정
    :param drone_id: 드론 객체 ID
    :param max_steps: 시뮬레이션 반복 횟수 제한
    :return: 측정된 충격량 합계
    """
    impulses = []
    has_collided = False

    for step in range(max_steps):
        p.stepSimulation()           # 시뮬레이션 1단계 진행
        time.sleep(time_step)        # 실제 시간과 맞추기 위한 지연 (없애도 무방)

        # 드론과 바닥 사이 충돌이 발생했는지 확인
        contact_points = p.getContactPoints(bodyA=drone_id, bodyB=plane_id)

        # 충돌이 발생했으면 충격량 저장
        if contact_points and not has_collided:
            has_collided = True
            for contact in contact_points:
                impulse = contact[9]  # contact[9] = 충격량 (normal impulse)
                impulses.append(impulse)
            break  # 첫 충돌만 측정하고 종료

    return sum(impulses)  # 충돌 지점들의 충격량 합계 반환


# [3] 실험 반복 실행 함수
def run_experiments(rotation_value, trials=5):
    """
    동일한 조건으로 여러 번 실험해 평균 충격량 계산
    :param rotation_value: 드론 회전 속도 값 (rad/s)
    :param trials: 반복 실험 횟수
    :return: 충격량 리스트
    """
    results = []

    for i in range(trials):
        # 드론 생성 및 회전 설정
        drone_id = create_drone(angular_velocity=rotation_value)

        # 시뮬레이션 실행 후 충격량 측정
        impulse = simulate_and_get_impulse(drone_id)
        results.append(impulse)

        # 시뮬레이션 초기화 후 재설정
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # 바닥 다시 로드

    return results


# [4] 회전 여부 실험 비교 실행
if __name__ == "__main__":
    print("▶ 회전하지 않은 드론 실험 중...")
    no_rotation_impulses = run_experiments(rotation_value=0.0)

    print("▶ 회전하는 드론 실험 중...")
    rotation_impulses = run_experiments(rotation_value=10.0)

    # 평균값 출력
    print("\n📊 실험 결과 요약")
    print(f"회전 X 평균 충격량: {np.mean(no_rotation_impulses):.2f}")
    print(f"회전 O 평균 충격량: {np.mean(rotation_impulses):.2f}")