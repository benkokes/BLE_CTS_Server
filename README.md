

Summary:
The intention of this project is to provide an accurate time/date to BLE perhiperals.  The project leverages the BLE Current Time Service, but is a server, versus a client. The Peripheral-isde(not this code) is based of the Nordic NRF52-series perhiperal CTS example.  

General Operation:
The server constantly listens for connectable advertising packets. The scan result is filtered to a particular exposed name on the advertising packet. Once that match is made, the ESP32 central connects to the NRF52 perhiperal and serves NTP time and date.  NTP time is fetched once a day and stored to the on-board RTC. Perhiperal initiates disconnect after transfer of time information, then the central resumes scanning for appropriate advertising perhiperals. NTP Data is retrieved over wired ethernet connection.


Hardware:
Main board(central role): Olimex ESP32-Gateway, revision E. This utilizes an Espressif-32 module

