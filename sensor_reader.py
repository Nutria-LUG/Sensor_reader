# -*- coding: utf-8 -*-
"""
Sensor reader for BME280 Temp sensor and SDS011 PM sensor
"""

from BME280.Adafruit_BME280 import *
from SDS011.sds011 import SDS011
from SDS011.sds011 import report_mode
from SDS011.sds011 import work_state


def main():

    # sensor_tmp = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
    # degrees = sensor_tmp.read_temperature()
    # pascals = sensor_tmp.read_pressure()
    # hectopascals = pascals / 100
    # humidity = sensor_tmp.read_humidity()
    # print(f"log: temperature {degrees}")
    # print(f"log: pressure {pascals}")
    # print(f"log: humidity {humidity}")

    try:
        sensor_pm = SDS011("COM4")
        # sensor_pm = SDS011("/dev/ttyS1")
        sensor_pm.report_mode = report_mode["Passive"]
        time.sleep(5) # Investigate and test this sleep time
        pm10, pm25 = sensor_pm.request()
        if pm25 is not None and pm10 is not None:
            print("log: pm10 {}".format(pm10))
            print("log: pm25 {}".format(pm25))
        sensor_pm.work_state = work_state["Sleeping"]
    except BaseException as e:
        print("err: {}".format(e.message))


if __name__ == "__main__":
    main()
