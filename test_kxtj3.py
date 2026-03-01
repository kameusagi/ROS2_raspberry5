from smbus2 import SMBus
import time

ADDR = 0x0E
WHO_AM_I = 0x0F
CTRL_REG1 = 0x1B
DATA_OUT_X_L = 0x06

def main():
    try:
        with SMBus(1) as bus:
            # 1. 接続確認（リトライ付き）
            try:
                who = bus.read_byte_data(ADDR, WHO_AM_I)
                print(f"WHO_AM_I: {hex(who)}")
                time.sleep(0.1)
            except OSError:
                print("初期通信でエラー。配線の接触不良を疑ってください。")
                return

            if who == 0x35:
                # 2. センサを一度スタンバイにしてから設定（重要）
                # PC1 bitを0にしてスタンバイに
                bus.write_byte_data(ADDR, CTRL_REG1, 0x00)
                time.sleep(0.1)

                # 3. 動作モード設定 (12-bit, ±2g, 起動)
                # 0x80 (PC1=1) | 0x40 (RES=1:High Resolution)
                bus.write_byte_data(ADDR, CTRL_REG1, 0xC0) 
                time.sleep(0.2) # 起動完了まで長めに待つ

                print("Reading data... Press Ctrl+C to stop")
                
                while True:
                    try:
                        # X軸だけを「一気」に2バイト読む
                        data_x = bus.read_i2c_block_data(ADDR, 0x06, 2)
                        xl = data_x[0]
                        xh = data_x[1]
                        print(f"X raw: {xh:02x}{xl:02x}")
                    except OSError:
                        print("X軸読み取りエラー")
                    time.sleep(0.5)
            else:
                print("デバイスが見つかりましたが、KXTJ3ではありません。")

    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()