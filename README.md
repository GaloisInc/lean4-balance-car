# lean4-balance-car
Lean4 port of Arduino balance car controller

# Balance Car Hardware

Yahboom Coding Robot Car Balance Robot Electronics Programmable Kit for Adult Support C Language (UNO R3 Include)
https://www.amazon.com/Yahboom-Compatible-Electronics-Programmable-Education/dp/B07FL2QR1V

# Files

+ Arduino files
  - `BalanceCar/BalanceCar.ino` (actual code running on the car)
    + Code to initialize the car and send motion readings over serial every 5ms while 
      while waiting for commands to come over serial on which motors to move.
  - `BalanceCar/orig/tweaked_bst_abc/tweaked_bst_abc.ino`
    * A simplified version of the balance car's _original_ arduino code -- keeps the car upright/balanced
  - `BalanceCar/orig/tweaked_bst_abc/c/balance-car.c`
    * modified version of `tweaked_bst_abc.ino` for testing purposes and comparison with the Lean code
  - `arduino-deps`
    * Arduino support libraries needed by the balance car
+ Lean controller code
  - `BalanceCar.lean` (code running on the RPi4 to control the car)
    * Lean code which reads data from the balance car over a serial port, perform calculations,
      then send back commands to car to keep it balanced.
  - `Serial.cpp`
    * C code `BalanceCar.lean` relies on for serial TX/RX with the car

## Lean4 controller build instructions

1. Ensure [elan](https://github.com/Kha/elan) is installed (a tool for
   like `rustup` or `ghcup` but for `lean`).

2. Run `leanmake` in the root directory of the repo.

```

```
