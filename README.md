# ESP-IDF A2DP Sink to SSM2518 (PMOD AMP3)

Proyecto para ESP-IDF 5.4.1 que:
- Inicia Bluetooth Clásico como A2DP Sink.
- Decodifica audio y lo envía por I2S al SSM2518 (PMOD AMP3).
- Detecta el SSM2518 por I2C (0x34). Si existe, configura modo programable; si no, usa Stand Alone.

## Pines por defecto
- I2C: SDA=GPIO21, SCL=GPIO22
- I2S: BCLK=GPIO26, LRCK/WS=GPIO25, DOUT=GPIO27, MCLK=No usado

Ajusta en `main/main.c` si tu cableado es distinto.

## Conexión al PMOD AMP3
Consulta el manual del PMOD AMP3 para mapear BCLK/LRCLK/SDATA y alimentación 3V3/GND. El SSM2518 puede operar sin MCLK usando PLL interna.

## Compilar y flashear
Requisitos: ESP-IDF 5.4.1 instalado y exportado en tu PowerShell.

```powershell
# Desde la carpeta del proyecto
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```
Reemplaza `COMx` por tu puerto. Sal del monitor con `Ctrl+]`.

## Emparejamiento
- El dispositivo se anuncia como: "ESP32 A2DP SSM2518".
- Pon tu teléfono o PC en modo scan, empareja y reproduce audio.

## Notas
- Este ejemplo configura I2S en 16-bit estéreo y ajusta 44.1 kHz como valor por defecto cuando el stream inicia. Si tu fuente usa 48 kHz, puedes ajustar la tasa dinámicamente.
- La configuración de registros del SSM2518 es representativa. Revisa la hoja de datos para refinar PLL y ganancias según tu diseño.
