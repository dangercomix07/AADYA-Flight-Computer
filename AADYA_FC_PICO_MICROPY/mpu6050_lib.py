# Simple micropython libraray for MPU6050
# https://github.com/adamjezek98/MPU6050-ESP8266-MicroPython

class accel():
    def __init__(self, i2c_client, addr=0x68):
        self.iic = i2c_client
        self.addr = addr
        self.a = [0.0, 0.0, 0.0]
        self.g = [0.0, 0.0, 0.0]
        self.temp = 0.0
        self._a_cal = [0.0, 0.0, 0.0]
        self._g_cal = [0.0, 0.0, 0.0]
        self.iic.writeto(self.addr, bytearray([107, 0]))

    def calibrate(self, n, g):
        a_cal = [0.0, 0.0, 0.0]
        g_cal = [0.0, 0.0, 0.0]
        for j in range(n):
            self.get_values()
            a_cal[0] += self.a[0]
            g_cal[0] += self.g[0]
            a_cal[1] += self.a[1]
            g_cal[1] += self.g[1]
            a_cal[2] += self.a[2] - g
            g_cal[2] += self.a[2]
        for i in range(3):
            self._a_cal[i] = a_cal[i] / n
            self._g_cal[i] = g_cal[i] / n 

    def get_raw_values(self):
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        self.a[0] = (self.bytes_toint(raw_ints[0], raw_ints[1]))*20 / 32768.0
        self.a[1] = (self.bytes_toint(raw_ints[2], raw_ints[3]))*20 / 32768.0
        self.a[2] = (self.bytes_toint(raw_ints[4], raw_ints[5]))*20 / 32768.0
        self.temp = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        self.g[0] = (self.bytes_toint(raw_ints[8], raw_ints[9])) / 32768.0
        self.g[1] = (self.bytes_toint(raw_ints[10], raw_ints[11])) / 32768.0
        self.g[2] = (self.bytes_toint(raw_ints[12], raw_ints[13])) / 32768.0
        # -32768 to 32767

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        from time import sleep
        while 1:
            print(self.get_values())
            sleep(0.05)
