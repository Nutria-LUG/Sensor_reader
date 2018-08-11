install:
	python setup.py install
	chmod +x sensor-reader.py
	cp sensor-reader.py /usr/bin/sensor-reader

uninstall:
	rm /usr/bin/sensor-reader
	rm -rf /usr/local/lib/python2.7/dist-packages/sensor_reader-1.0.egg-info
	rm -rf /usr/local/lib/python2.7/dist-packages/SDS011
	rm -rf /usr/local/lib/python2.7/dist-packages/BME280

.PHONY: install uninstall
