from machine import I2C, Pin
import utime
import time
import struct
from lcd_api import LcdApi  # Убедитесь, что lcd_api.py загружен на ESP32
from pico_i2c_lcd import I2cLcd  # Убедитесь, что pico_i2c_lcd.py загружен на ESP32

# Настройка I2C
I2C_ADDR = 0x27  # Адрес I2C вашего дисплея (проверьте его с помощью сканера I2C)
BME280_ADDR = 0x76  # Адрес I2C BME280
I2C_NUM_ROWS = 2  # Количество строк на дисплее
I2C_NUM_COLS = 16  # Количество столбцов на дисплее

# Инициализация I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22))  # Пины SDA и SCL могут отличаться в зависимости от вашей платы

# Инициализация дисплея
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

# Очистка дисплея
lcd.clear()

SHT21_ADDR = 0x40
CMD_MEASURE_TEMP = 0xE3
CMD_MEASURE_HUM = 0xF5


# Чтение сенсора
def read_sensor(cmd):
    i2c.writeto(SHT21_ADDR, bytes([cmd]))
    time.sleep(0.1)
    data = i2c.readfrom(SHT21_ADDR, 3)
    return (data[0] << 8) | data[1]


# Получение температуры
def get_temperature():
    raw_temp = read_sensor(CMD_MEASURE_TEMP)
    return -46.85 + (175.72 * raw_temp / 65536)


# Получение влажности
def get_humidity():
    raw_hum = read_sensor(CMD_MEASURE_HUM)
    return -6.0 + 125.0 * (raw_hum / 65536.0)


# Запись в регистр BME280
def write_bme280_reg(reg, value):
    i2c.writeto_mem(BME280_ADDR, reg, bytes([value]))


# Чтение регистра BME280
def read_bme280_reg(reg, lenght=1):
    return i2c.readfrom_mem(BME280_ADDR, reg, lenght)


# Чтение калибровочных данных
def read_bme280_calibration():
    data = read_bme280_reg(0x88, 24)
    calib = {
        'dig_T1': struct.unpack('<H', data[0:2])[0],
        'dig_T2': struct.unpack('<h', data[2:4])[0],
        'dig_T3': struct.unpack('<h', data[4:6])[0],
        'dig_P1': struct.unpack('<H', data[6:8])[0],
        'dig_P2': struct.unpack('<h', data[8:10])[0],
        'dig_P3': struct.unpack('<h', data[10:12])[0],
        'dig_P4': struct.unpack('<h', data[12:14])[0],
        'dig_P5': struct.unpack('<h', data[14:16])[0],
        'dig_P6': struct.unpack('<h', data[16:18])[0],
        'dig_P7': struct.unpack('<h', data[18:20])[0],
        'dig_P8': struct.unpack('<h', data[20:22])[0],
        'dig_P9': struct.unpack('<h', data[22:24])[0]
    }
    return calib


# Настройка BME280
def setup_bme280():
    write_bme280_reg(0xF2, 0x01)
    write_bme280_reg(0xF4, 0x27)
    write_bme280_reg(0xF5, 0xA0)


# Чтение необработанных данных
def read_bme280_raw():
    data = read_bme280_reg(0xF7, 6)
    raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    return raw_pressure, raw_temp


# Функция вычисления давления
def compensate_pressure(raw_pressure, raw_temp, calib):
    var1 = ((((raw_temp >> 3) - (calib['dig_T1'] << 1))) * calib['dig_T2']) >> 11
    var2 = (((((raw_temp >> 4) - calib['dig_T1']) * ((raw_temp >> 4) - calib['dig_T1'])) >> 12) * calib['dig_T3']) >> 14
    t_fine = var1 + var2

    var1 = (t_fine >> 1) - 64000
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * calib['dig_P6']
    var2 = var2 + ((var1 * calib['dig_P5']) << 1)
    var2 = (var2 >> 2) + (calib['dig_P4'] << 16)
    var1 = (((calib['dig_P3'] * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((calib['dig_P2'] * var1) >> 1)) >> 18
    var1 = ((32768 + var1) * calib['dig_P1']) >> 15

    if var1 == 0:
        return 0  # Избегаем деления на 0

    pressure = (((1048576 - raw_pressure) - (var2 >> 12)) * 3125)
    pressure = (pressure // var1) * 2
    var1 = (calib['dig_P9'] * (((pressure >> 3) * (pressure >> 3)) >> 13)) >> 12
    var2 = ((pressure >> 2) * calib['dig_P8']) >> 13
    pressure = pressure + ((var1 + var2 + calib['dig_P7']) >> 4)

    return pressure / 100.0  # Давление в гПа


# Инициализация датчика BME280
try:
    setup_bme280()
    calib_data = read_bme280_calibration()
    print("BME280 успешно инициализирован")
except Exception as bme_error:
    print("BME280 не инициализирован", bme_error)


# Функция для отображения текста
def display_text(text, row=0, col=0):
    lcd.move_to(col, row)
    lcd.putstr(text)


display_text(f"Temp is", row=0, col=0)  # Вывод текста на первую строку
display_text(f"Pressure", row=1, col=0)  # Вывод текста на вторую строку
while True:
    try:
        raw_pressure, raw_temp = read_bme280_raw()
        pressure = (compensate_pressure(raw_pressure, raw_temp, calib_data) / 1000)
        temp = get_temperature()
        print(f"Temperature is {temp}")
        print(f"Pressure is {pressure}")
        display_text(f"{temp:.2f}", row=0, col=10)  # Вывод текста на первую строку
        display_text(f"{pressure:.2f}", row=1, col=10)  # Вывод текста на вторую строку
        time.sleep(1)
    except Exception as error:
        print("Ошибка чтения температуры:", error)
        lcd.clear()  # Очистка экрана
        display_text(f"Sensor lost", row=0, col=0)  # Вывод текста на первую строку
        display_text(f"Output old value", row=1, col=0)  # Вывод текста на первую строку
        time.sleep(1)
        lcd.clear()  # Очистка экрана
        display_text(f"Old_Temp is {temp:.1f}", row=0, col=0)  # Вывод текста на первую строку
        display_text(f"Old_Pressure {pressure:.1f}", row=1, col=0)  # Вывод текста на первую строку
        time.sleep(1)
        lcd.clear()
