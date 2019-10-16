# wemosd1-mpu6050
Konsep penggunaan wemosd1 untuk http dan websocket server dan sensor mpu6050
Webserver menggunakan library threejs.org

Perangkat Keras:
1. Wemos D1 mini (4MB)
2. mpu6050

Perangkat Lunak:
1. Arduino dengan tambahan:

a. board library: esp8266 versi 2.5.2

Website: https://github.com/esp8266/Arduino

Fungsi: untuk wifi, http, dan websocket server library

b. SPIFFS plugin: ESP8266FS versi 0.4.0 

Website: https://github.com/esp8266/arduino-esp8266fs-plugin/releases

Fungsi: untuk mengunggah data file html dan javascript yang dibutuhkan sebagai simulasi


Koneksi pin:

mpu6050         Wemos D1

VCC             3.3V

SDA             D2

SCL             D1

AD0             D3

INT             D5


Cara:

1. Unggah file wemosd1-mpu6050-ino.ino ke wemosd1 menggunakan program Arduino. 

   Gunakan setting Flash Size = 4M(3M SPIFFS)

2. Klik Tools->ESP8266 Sketch Data Upload.

3. Koneksikan wifi komputer ke wemosd1 AP ("yourWifiApName" dengan kata kunci: "wifipass").

4. Cek ip address komputer.

5. Gunakan web browser (Chrome, dll.) untuk mengakses ip wemosd1 dengan port 8080.
