#!/usr/bin/env python3
"""
i2c 과제
 - mpu6050에서 6축 가속도/자이로 값 읽는 코드 작성
 - mpu6050 레지스터 중, PWR_MGMT_1에 있는 sleep bit를 설정하여
   장치를 sleep/wake 상태를 toggle 하는 코드 작성

spi 과제
 - MRC522 모듈로 REQA(7비트 프레임) 전송 후 ATQA(2바이트) 응답 수신하는 코드
   작성
 - MRC522 모듈의 TX_CONTROL 레지스터에 안테나 드라이브 on/off하는 코드 작성

ssh/sftp 과제
 - SSH로 원격에서 architercure 문자열 받아오기
"""

import subprocess
from typing import Dict, Optional, Tuple

from smbus2 import SMBus
import spidev


# =========================
# I2C (MPU6050)
# =========================
MPU_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_ACCEL_YOUT_H = 0x3D
REG_ACCEL_ZOUT_H = 0x3F
REG_GYRO_XOUT_H = 0x43
REG_GYRO_YOUT_H = 0x45
REG_GYRO_ZOUT_H = 0x47


def _read_word(bus: SMBus, addr: int, reg_h: int) -> int:
    """MPU6050 16bit signed read"""
    hi = bus.read_byte_data(addr, reg_h)
    lo = bus.read_byte_data(addr, reg_h + 1)
    val = (hi << 8) | lo
    return val - 65536 if val & 0x8000 else val


def read_imu() -> Dict[str, int]:
    """
    과제 1)
    - MPU6050에서 가속도/자이로 6축 값을 읽어서 dict로 반환
    - {'ax':..., 'ay':..., 'az':..., 'gx':..., 'gy':..., 'gz':...}
    """
    with SMBus(1) as bus:
        bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)  # sleep 해제

        ax = _read_word(bus, MPU_ADDR, REG_ACCEL_XOUT_H)
        ay = _read_word(bus, MPU_ADDR, REG_ACCEL_YOUT_H)
        az = _read_word(bus, MPU_ADDR, REG_ACCEL_ZOUT_H)
        gx = _read_word(bus, MPU_ADDR, REG_GYRO_XOUT_H)
        gy = _read_word(bus, MPU_ADDR, REG_GYRO_YOUT_H)
        gz = _read_word(bus, MPU_ADDR, REG_GYRO_ZOUT_H)

    return {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}


def wake_device() -> Tuple[int, int]:
    """
    과제 2): SLEEP <-> WAKE
    - PWM_MGMT_1 전체 register value (before, after) 반환
    """
    with SMBus(1) as bus:
        before = bus.read_byte_data(MPU_ADDR, REG_PWR_MGMT_1) & 0xFF
        after = before ^ (1 << 6)  # sleep 비트 토글
        bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, after)
        verify = bus.read_byte_data(MPU_ADDR, REG_PWR_MGMT_1) & 0xFF
    return before, verify


# =========================
# SPI (RC522)
# =========================
RC522_CMD = 0x01
RC522_COMM_IRQ = 0x04
RC522_ERROR = 0x06
RC522_FIFO_DATA = 0x09
RC522_FIFO_LEVEL = 0x0A
RC522_BIT_FRAMING = 0x0D
RC522_TX_CONTROL = 0x14

CMD_IDLE = 0x00
CMD_TRANSCEIVE = 0x0C

REQA = 0x26  # 7비트 프레임


def _rc522_addr(reg: int, read: bool) -> int:
    a = (reg << 1) & 0x7E
    return a | 0x80 if read else a


class RC522:
    def __init__(self) -> None:
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1_000_000
        self.spi.mode = 0

    def close(self) -> None:
        self.spi.close()

    def write_reg(self, reg: int, val: int) -> None:
        self.spi.xfer2([_rc522_addr(reg, False), val & 0xFF])

    def read_reg(self, reg: int) -> int:
        return self.spi.xfer2([_rc522_addr(reg, True), 0x00])[1] & 0xFF

    def set_bits(self, reg: int, mask: int) -> None:
        self.write_reg(reg, self.read_reg(reg) | mask)

    def clear_bits(self, reg: int, mask: int) -> None:
        self.write_reg(reg, self.read_reg(reg) & (~mask & 0xFF))

    def antenna_on(self, on: bool) -> None:
        if on:
            self.set_bits(RC522_TX_CONTROL, 0x03)
        else:
            self.clear_bits(RC522_TX_CONTROL, 0x03)

    def transceive_7bit(self, b: int, loops: int = 50) -> bytes:
        self.write_reg(RC522_CMD, CMD_IDLE)
        self.write_reg(RC522_COMM_IRQ, 0x7F)
        self.write_reg(RC522_FIFO_LEVEL, 0x80)
        self.write_reg(RC522_FIFO_DATA, b & 0xFF)
        self.write_reg(RC522_BIT_FRAMING, 0x07)
        self.write_reg(RC522_CMD, CMD_TRANSCEIVE)
        self.set_bits(RC522_BIT_FRAMING, 0x80)

        for _ in range(loops):
            irq = self.read_reg(RC522_COMM_IRQ)
            if irq & 0x30:
                break

        if self.read_reg(RC522_ERROR) & 0x13:
            return b""

        n = self.read_reg(RC522_FIFO_LEVEL)
        return bytes(self.read_reg(RC522_FIFO_DATA) for _ in range(n))


def rfid_poll_once() -> Tuple[bool, Optional[bytes]]:
    """
    과제 3)
      - RC522로 REQA 전송 → ATQA(2바이트) 응답이면 태그 존재
      - (present, atqa_bytes) 반환
    """
    r = RC522()
    try:
        atqa = r.transceive_7bit(REQA)
        if len(atqa) == 2:
            return True, atqa
        return False, None
    finally:
        r.close()


def rfid_set_antenna(on: bool) -> int:
    """
    과제 4)
      - 모듈 레지스터 쓰기 예: 안테나 ON/OFF
      - 쓰고 나서 읽어서 상태(int) 반환 (TX_CONTROL 레지스터)
    """
    r = RC522()
    try:
        r.antenna_on(on)
        return r.read_reg(RC522_TX_CONTROL)
    finally:
        r.close()


# =========================
# SSH
# =========================
def ssh_get_arch() -> str:
    """
    SSH로 원격에서 architecture 문자열을 받아와 반환.
    - 반드시 user_host / cmd를 채워야 함 (비워두면 ValueError)
    - SSH 실패(returncode!=0)면 RuntimeError
    - 출력이 비정상이거나 arm64 계열이 아니면 AssertionError
    """
    user_host = "pi@raspberrypi.local"  # 실제 Pi 계정/주소로 변경
    cmd = "uname -m"

    result = subprocess.run(
        ["ssh", user_host, cmd],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if result.returncode != 0:
        raise RuntimeError(f"SSH failed: {result.stderr.strip()}")

    arch = (result.stdout or "").strip()
    if not arch:
        raise ValueError("원격 arch 문자열이 비어 있습니다.")
    if arch not in ("aarch64", "arm64"):
        raise AssertionError(f"arm64가 아닙니다: got {arch!r}")
    return arch
