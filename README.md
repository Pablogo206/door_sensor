# Instalacion del programa

1. Copia el codigo en plantilla
2. idf.py set-target esp32
3. idf.py menuconfig
  - Component config/Bluetooth -> Flecha a la derecha para que aparezca Bluetooth marcado:
    - [x] Bluetooth
4. idf.py -p COMX flash (Si haces flash se hace un build si lo necesita)
5. ver con 232Analyzer