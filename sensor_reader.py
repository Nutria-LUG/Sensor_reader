# -*- coding: utf-8 -*-
"""
Sensor reader for BME280 Temp sensor and SDS011 PM sensor
"""

from Adafruit_BME280 import *
from SDS011.sds011 import SDS011
from SDS011.sds011 import report_mode
from SDS011.sds011 import work_state

# Sensor codes:
SENSOR_PM10        = 0
SENSOR_PM25        = 1
SENSOR_TEMPERATURE = 2
SENSOR_HUMIDITY    = 3
SENSOR_PRESSION    = 4

def main():
    sensor_tmp = BME280(
        t_mode=BME280_OSAMPLE_8,
        p_mode=BME280_OSAMPLE_8,
        h_mode=BME280_OSAMPLE_8,
        standby=BME280_STANDBY_250,
        filter=BME280_FILTER_off, address=0x77)
    degrees = sensor_tmp.read_temperature()
    pascals = sensor_tmp.read_pressure()
    humidity = sensor_tmp.read_humidity()

    print("log: {} {}".format(SENSOR_TEMPERATURE, degrees))
    print("log: {} {}".format(SENSOR_PRESSION, pascals))
    print("log: {} {}".format(SENSOR_HUMIDITY, humidity))

    try:
        sensor_pm = SDS011("/dev/ttyS1")
        # sensor_pm = SDS011("/dev/ttyS1")
        sensor_pm.report_mode = report_mode["Passive"]
        time.sleep(5) # Investigate and test this sleep time
        pm10, pm25 = sensor_pm.request()
	time.sleep(5)
        if pm25 is not None and pm10 is not None:
            print("log: {} {}".format(SENSOR_PM10, pm10))
            print("log: {} {}".format(SENSOR_PM25, pm25))

        sensor_pm.work_state = work_state["Sleeping"]
    except BaseException as e:
        print("err: {}".format(e.message))


if __name__ == "__main__":
    main()
