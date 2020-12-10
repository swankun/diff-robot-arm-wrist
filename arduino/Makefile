FQBN = arduino:avr:uno

all:
	arduino-cli compile --fqbn $(FQBN) $(notdir $(CURDIR)).ino

upload:
	arduino-cli upload -b $(FQBN) -p /dev/ttyACM0 $(CURDIR)

clean:
	rm -rf $(CURDIR)/build
