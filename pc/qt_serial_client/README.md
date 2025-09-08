Qt6 console client for ESP32 TLV320DAC3100 UART protocol.

Build (Windows, Qt 6):
- Open a VS Native Tools + Qt env, then:

```
mkdir build
cd build
cmake -G "NMake Makefiles" -DCMAKE_PREFIX_PATH=%QT_PREFIX% ..
nmake
```

Run:
```
qt_serial_client COM5 ping
qt_serial_client COM5 detect
qt_serial_client COM5 read 0 0x1B
qt_serial_client COM5 write 1 0x23 0x44
```

Where COM5 is your ESP32 serial port.
