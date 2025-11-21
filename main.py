import pybullet as p
import pybullet_data
import time
import math

# 드론 생성: 몸체 + 날개
def create_drone(base_position, wing_count=4, wing_scale=1.0):
    # 몸체: 단순 원통
    base_radius = 0.2
    base_thickness = 0.05
    base_mass = 1.0

    base_col = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=base_radius,
        height=base_thickness
    )
    base_vis = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=base_radius,
        length=base_thickness,
        rgbaColor=[0.5, 0.5, 0.5, 1]
    )

    drone = p.createMultiBody(
        baseMass=base_mass,
        baseCollisionShapeIndex=base_col,
        baseVisualShapeIndex=base_vis,
        basePosition=base_position,
        baseOrientation=[0, 0, 0, 1]
    )

    # 날개: 박스 모양 (랜딩기어 + 암 역할)
    wing_length = 0.3 * wing_scale
    wing_width  = 0.05 * wing_scale
    wing_thickness = 0.01
    wing_mass = 0.05  # 너무 무겁지 않게 (본체보다 훨씬 가볍게)

    wing_col = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[wing_length/2, wing_width/2, wing_thickness/2]
    )
    wing_vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[wing_length/2, wing_width/2, wing_thickness/2],
        rgbaColor=[0.2, 0.2, 1, 1]
    )

    wing_ids = []

    for i in range(wing_count):
        angle = i * (360.0 / wing_count)
        rad = math.radians(angle)
        dx = 0.25 * math.cos(rad)
        dy = 0.25 * math.sin(rad)

        # 날개를 몸체 중심 근처에 생성
        wing_start_pos = [base_position[0], base_position[1], base_position[2]]
        wing_id = p.createMultiBody(
            baseMass=wing_mass,
            baseCollisionShapeIndex=wing_col,
            baseVisualShapeIndex=wing_vis,
            basePosition=wing_start_pos,
            baseOrientation=[0, 0, 0, 1]
        )
        wing_ids.append(wing_id)

        # 몸체와 날개를 고정 조인트로 "용접"
        # parentFramePosition: 몸체 기준 날개의 상대 위치
        # z를 -0.03으로 내려서 날개가 몸체보다 살짝 아래로 튀어나오게 (랜딩기어 느낌)
        p.createConstraint(
            parentBodyUniqueId=drone,
            parentLinkIndex=-1,
            childBodyUniqueId=wing_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[dx, dy, -0.03],
            childFramePosition=[0.0, 0.0, 0.0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=[0, 0, 0, 1]
        )

    return drone, wing_ids

# 실험 실행
def run_experiment_in_gui(label, wing_count=4, wing_scale=1.0, sim_time=3.0):
    # 이전 시뮬레이션 초기화
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # 카메라 위치 설정
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )

    # 바닥 생성
    plane_id = p.loadURDF("plane.urdf")

    # 드론 생성 (높이 1.5에서 낙하)
    drone, wing_ids = create_drone([0, 0, 1.5], wing_count, wing_scale)

    # 물리 엔진 설정
    p.setPhysicsEngineParameter(
        fixedTimeStep=1.0 / 240.0,
        numSolverIterations=50
    )

    # 화면에 현재 실험 이름 띄우기
    p.addUserDebugText(
        text=label,
        textPosition=[0, 0, 2],
        textSize=1.5,
        lifeTime=sim_time
    )

    print(f"\n=== {label} 시작 (날개 {wing_count}개, 스케일 {wing_scale}) ===")

    max_core_impact = 0.0     # 몸체가 바닥과 직접 접촉하며 받은 최대 충격력
    max_total_impact = 0.0    # 몸체 + 날개 전체가 바닥에 준 최대 충격력(참고용)
    contact_happened = False

    steps = int(sim_time / (1.0 / 240.0))

    for _ in range(steps):
        p.stepSimulation()

        # 1) 몸체 vs 바닥
        core_contacts = p.getContactPoints(bodyA=drone, bodyB=plane_id)
        core_force_sum = 0.0
        if core_contacts:
            contact_happened = True
            for c in core_contacts:
                core_force_sum += c[9]  # normal force

        if core_force_sum > max_core_impact:
            max_core_impact = core_force_sum

        # 2) 날개들 vs 바닥 (전체 충격 계산용)
        total_force = core_force_sum
        for wid in wing_ids:
            w_contacts = p.getContactPoints(bodyA=wid, bodyB=plane_id)
            if w_contacts:
                contact_happened = True
                for c in w_contacts:
                    total_force += c[9]

        if total_force > max_total_impact:
            max_total_impact = total_force

        time.sleep(1.0 / 240.0)  # 실제 시간과 비슷하게 진행

    print(f"[{label}] 본체 기준 최대 충격력: {max_core_impact:.3f}")
    #print(f"[{label}] 전체(몸체+날개) 최대 충격력: {max_total_impact:.3f}")
    if not contact_happened:
        print(f"[{label}] ⚠ 바닥과의 접촉이 거의 없었습니다. sim_time/높이를 늘려보세요.")

    # 실험 끝난 화면 잠깐 유지
    time.sleep(1.0)

    # 본체 기준 최대 충격력 리턴
    return max_core_impact, max_total_impact

if __name__ == "__main__":
    # GUI 한 번만 열기
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 4가지 실험 케이스
    experiments = [
        ("실험 1: 4개 날개, 기본 크기", 4, 1.0),
        ("실험 2: 5개 날개, 기본 크기", 5, 1.0),
        ("실험 3: 4개 날개, 큰 날개(1.5배)", 4, 1.5),
        ("실험 4: 4개 날개, 작은 날개(0.5배)", 4, 0.5),
    ]

    results = []

    for label, wing_count, wing_scale in experiments:
        core, total = run_experiment_in_gui(
            label=label,
            wing_count=wing_count,
            wing_scale=wing_scale,
            sim_time=3.0  # 시뮬레이션 시간
        )
        results.append((label, wing_count, wing_scale, core, total))

    # 최종 요약 출력
    print("\n======================")
    print("      실험 결과 요약")
    print("======================")
    for label, wing_count, wing_scale, core, total in results:
        print(f"{label} (날개 {wing_count}개, 스케일 {wing_scale})")
        print(f"  - 본체 기준 최대 충격력: {core:.3f}")
        #print(f"  - 전체(몸체+날개) 최대 충격력: {total:.3f}")
    #print("\n※ 가설 검증에는 '본체 기준 최대 충격력' 값을 사용하면 됨.")
    #print("GUI 창은 그대로 두고, 화면 비교용 / 시연용으로 활용하세요. (종료: 터미널에서 Ctrl + C)")
    print("종료: 터미널에서 Ctrl + C")

    # GUI 창 유지
    while True:
        time.sleep(0.1)
