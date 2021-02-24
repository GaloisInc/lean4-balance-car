# lean4-balance-car
Lean4 port of an Arduino balance car controller. Specifically, the balance car transmits sensor readings via a bluetooth serial connection to a Lean4 controller running on a Raspberry Pi 4, which replies with commands to keep the car balanced.

<img src="https://github.com/GaloisInc/lean4-balance-car/blob/images/overview.png?raw=true" alt="Architectural Overview" width="600"/>

[Here](https://github.com/GaloisInc/lean4-balance-car/blob/images/lean4-balance-car.mov?raw=true) is a short video of the car balancing using this setup.

# Balance Car Hardware

Yahboom Coding Robot Car Balance Robot Electronics Programmable Kit for Adult Support C Language (UNO R3 Include)
https://www.amazon.com/Yahboom-Compatible-Electronics-Programmable-Education/dp/B07FL2QR1V

# Important Files

+ Arduino Depencencies
  - The `arduino-deps` directory contains the Arduino support libraries
    used by the balance car code. They need to be manually installed
    in the location the Arduino IDE looks for such dependencies.

+ Arduino Balance Car files
  - `ArduinoBalanceCar/ArduinoBalanceCar.ino` contains code which when run on
    the Arduino will allow the car to balance. If has a debug mode (see the commented 
    out `#define DEBUG`) which can be run to causes the car to (1) wait for an initial
    signal to begin balancing and (2) relay the sampled data and some computational
    results accross the serial connection to aid in debugging.

+ Arduino Lean Balance Car files
  - `LeanBalanceCar/LeanBalanceCar.ino` contains code meant to be run on the Arduino
    to communicate with an external Lean process over serial to get commands on
    when to drive the motors. It waits until an initial byte is received before it
    starts sending the 5ms interrupt-driven readings from the sensors over the
    serial connection.

+ Lean controller code
  - `BalanceCar.lean` is the lean program meant to control the balance car when it is
    running the aforementioned LeanBalanceCar.ino program. It depends on some C code
    in `Serial.cpp` for the low-level details of the serial port communication.
    + N.B., there is a `debugMode`  boolean flag in `BalanceCar.lean` which
      makes the Lean executable, _instead_ of try to communicate over a serial connection
      to control the car, take raw data files containing the content emitted
      from the `ArduinoBalanceCar.ino` in `DEBUG` mode and compares the Lean
      computed `pwm1`/`pwm2` values against what the car computed.

+ C Debugging Code
  - `balance-car.c` is a copy-and-paste of the code in `ArduinoBalanceCar.ino`
    modified to simulate the car calculations given a file containing 
    the raw data emitted from `ArduinoBalanceCar.ino` in `DEBUG` mode. (It should
    be trivial to build and run with any C compiler -- it has no local dependencies.)

+ Sampled Debug Data
  - `raw-debut-data*.txt` files contain sampled debug data gathered from `ArduinoBalanceCar.ino`
    in `DEBUG` mode.

## Building the Lean4 controller

1. Ensure [elan](https://github.com/Kha/elan) is installed (a tool for
   like `rustup` or `ghcup` but for `lean`).

2. Run `leanmake` in the root directory of the repo.

3. If successful, the binary `build/bin/balance-car` should exist.

## Pairing the BalanceCar with the RPi4

+ `bluetoothctl`
  + `scan on`
  + turn on car
  + `pair 00:20:10:08:56:DA` (code 1234)
  + `trust 00:20:10:08:56:DA`
+ `sdptool add --channel=1 SP`
+ `sudo rfcomm connect hci0 00:20:10:08:56:DA`
  + Reports `Connected /dev/rfcomm0 to 00:20:10:08:56:DA on channel 1` and
    blocks until the process killed or the connection terminates.

## Running the Car against the Lean Controller

1. Ensure `LeanBalanceCar/LeanBalanceCar.ino` is loaded onto the Arduino.

2. Build the Lean `balance-car` binary on the RPi4.

3. Turn the Car on.

4. On the RPi4, run `sudo rfcomm connect hci0 00:20:10:08:56:DA` (assuming the
   car has already been paired with the RPi4, as described in a previous section).
   This will block this particular terminal.

5. In another terminal, run `sudo ./build/bin/balance-car /dev/rfcomm0 115200`
   to connect the Lean4 controller to the car.
