# Steps
+ `leanmake` to build Lean car controller
+ `sudo rfcomm connect hci0 00:20:10:08:56:DA`
  - yields e.g. `Connected /dev/rfcomm0 to 00:20:10:08:56:DA on channel 1`
+ `sudo ./build/bin/balance-car /dev/rfcomm0 115200`
