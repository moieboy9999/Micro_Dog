# Micro Dog (한국어 요약)

- Dynamixel XL330-M288-T 12개로 움직이는 0.96 kg, 12자유도 4족 로봇이다.
- 두뇌는 Raspberry Pi Zero 2 W이고, 무릎은 평행사변형 4절링크로 구동한다.
- 이 저장소에는 설계 전체가 들어 있다.
  - Inventor 원본
  - STEP
  - 링크별 메시
  - 실측 질량·관절 한계를 반영한 URDF
  - Isaac Sim/Isaac Lab용 USD
  - URDF 재생성 스크립트

자세한 내용은 영문 [README](README.md)와 [docs/specs.md](docs/specs.md)를 보라.

| 항목 | 값 |
|---|---|
| 총 질량 | 0.960 kg (저울 실측) |
| 크기 | 334.5 × 165.5 × 145.2 mm |
| 자유도 | 12 (다리당 roll / pitch / knee) |
| 다리 | 허벅지 95.0 mm, 종아리 97.0 mm, 발 반지름 13.0 mm |
| 서 있는 높이 | 0.170 m |

**판본**
- `urdf/`, `meshes/`, `step/`, `usd/`, `params/`는 **2026-08-23 CAD 추출본**이다. 시뮬레이션과 학습에 쓴 판본이다.
- `cad/inventor/`는 **2026-09-21 최신 설계**이고, 몸 커버가 추가돼 있다.
- 몸 커버는 시뮬 모델에 포함되지 않았다. CAD 질량은 속이 꽉 찬 밀도 기준 43.4 g이고, 출력물은 이보다 가볍다([docs/body_cover.md](docs/body_cover.md)).

**불러오기**
- Isaac에서는 `usd/micro_dog.usda`를 그대로 쓰면 된다.
- URDF를 직접 변환하면 반드시 flatten해야 한다. 안 하면 발 접촉이 보고되지 않는다.

**Inventor**
- `cad/inventor/micro_dog.ipj`를 프로젝트로 설정한 뒤 `cad/inventor/전체.iam`을 연다.
- Inventor 2027 교육용으로 만든 파일이라 교육용 표시가 붙어 있다.
- Raspberry Pi Zero 2 W 모델은 제3자 모델이라 빠져 있다. 따로 받아서 `몸/시스템/Raspberry Pi Zero 2 W.ipt`로 저장하면 조립이 복원된다.

**BOM(부품 목록)**
- 추후 업데이트 예정이다. 전자부품, 베어링, 체결류, 인쇄 부품 수량, 필라멘트 사용량을 담는다.

**라이선스**
- 하드웨어·문서: CC BY-NC-SA 4.0
- `tools/` 스크립트: GPL-3.0-or-later
- 출처: [ATTRIBUTION.md](ATTRIBUTION.md)
