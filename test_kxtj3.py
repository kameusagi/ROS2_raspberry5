from smbus2 import SMBus
import time

# KXTJ3 I2C address (0x0E or 0x0F)
ADDR = 0x0E
WHO_AM_I = 0x0F # 期待値は 0x35

with SMBus(1) as bus:
    # 接続確認
    who = bus.read_byte_data(ADDR, WHO_AM_I)
    print(f"WHO_AM_I: {hex(who)} (Expected: 0x35)")

    if who == 0x35:
        # センサを起動モードに設定 (CTRL_REG1)
        # 0x80: PC1 bitを1にして動作開始
        time.sleep(0.1)
        bus.write_byte_data(ADDR, 0x1B, 0x80)
        
        print("Reading data... Press Ctrl+C to stop")
        while True:
            # X, Y, Z軸のデータを読み取り (簡易版)
            data = bus.read_i2c_block_data(ADDR, 0x06, 6)
            # 12-bitデータなどの整形が必要ですが、まずは値が動くか確認
            print(f"Raw Data: {data}")
            time.sleep(0.5)