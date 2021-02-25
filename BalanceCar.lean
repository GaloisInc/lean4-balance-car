-- Port of roughly the following:
-- https://gitlab-int.galois.com/andrew/lean4-robotics-car-libs/-/blob/master/BalanceCar/examples/bst_abc/bst_abc.ino

import Lean.Data.Json

namespace Float

instance : Coe Int Float := ⟨Float.ofInt⟩

@[macroInline]
def constrain (n low high : Float) : Float :=
  if n < low then low
  else if n > high then high
  else n

def pi : Float := 3.141592653589793238462643


@[extern "fabs"] constant abs : Float → Float

end Float

/-- Common baud rates for serial ports. -/
inductive BaudRate
  | bps1200
  | bps2400
  | bps4800
  | bps9600
  | bps19200
  | bps38400
  | bps57600
  | bps115200

namespace BaudRate

/-- Converts a BaudRate into the uint16 value it is defined as
    in C (i.e., see the C `speed_t` type and associated C macros). -/
def toUInt16 : BaudRate → UInt16
  | bps1200    => 9    -- 0000011
  | bps2400    => 11   -- 0000013
  | bps4800    => 12   -- 0000014
  | bps9600    => 13   -- 0000015
  | bps19200   => 14   -- 0000016
  | bps38400   => 15   -- 0000017
  | bps57600   => 4097 -- 0010001
  | bps115200  => 4098 -- 0010002

def ofNat? : Nat → Option BaudRate
  | 1200   => some bps1200
  | 2400   => some bps2400
  | 4800   => some bps4800
  | 9600   => some bps9600
  | 19200  => some bps19200
  | 38400  => some bps38400
  | 57600  => some bps57600
  | 115200 => some bps115200
  | _      => none

end BaudRate


-- Arduino related constants
def pin1 : UInt8 := 1
def pin2 : UInt8 := 2
def pin3 : UInt8 := 3
def pin4 : UInt8 := 4
def pinPWMA : UInt8 := 5
def pinPWMB : UInt8 := 6

-- Some car-related constants
def qAngle : Float := 0.001
def qGyro  : Float := 0.005
def rAngle : Float := 0.5
def p0 : Float := 0.0 -- Angle balance point
def c0 : Float := 1
def k1 : Float := 0.05
def angle0 : Float := 0 -- Mechanical balance angle
def sampleIntervalMs : Float := 5 -- was timeChange
def dt : Float := sampleIntervalMs * 0.001
def kp : Float := 38.0
def kd : Float := 0.58
def kpTurn : Float := 28.0
def kdTurn : Float := 0.29
def kpSpeed : Float := 3.1
def kiSpeed : Float := 0.05
def kdSpeed : Float := 0.0



structure Controller where
  forward   : Int -- Forward variable
  reverse   : Int -- Back variable
  turnLeft  : Bool -- Turn left sign
  turnRight : Bool -- Turn right sign
  spinLeft  : Bool -- Rotate left sign
  spinRight : Bool -- Right rotation sign


-- See also https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp 
-- for a... cleaner setup than the one we're mirroring at the moment

structure BalanceCar where
  ctrl : Controller
  countLeft  : Int
  countRight : Int
  pulseLeft  : Int
  pulseRight : Int
  stopLeft : Int
  stopRight : Int
  angleOut : Float
  pwm1 : Float
  pwm2 : Float
  speedFilter : Float
  positions : Float
  resetPosition : Bool
  turnSpinDelay : Nat
  turnMax : Int
  turnMin : Int
  turnOut : Float
  speedPI : Float -- calculated by `updateSpeedPI`
  turnSpin : Float -- calculated by `updateTurnSpin`
  speedPIDelay : Nat
  gyroX : Float
  gyroY : Float
  gyroZ : Float
  (angle angle6 qBias angleDot : Float)
  p00 : Float
  p01 : Float
  p10 : Float
  p11 : Float
  -- ^ Posteriori estimate covariance matrix for the Kalman filter,
  --   represented as a pair of matrix rows (where each row is also a pair)
  pDot0 : Float
  pDot1 : Float
  pDot2 : Float
  pDot3 : Float



def Controller.initial : Controller :=
    ⟨0,0,false,false,false,false⟩ 


namespace BalanceCar

-- Kalman Filter
def kFilter (car : BalanceCar) (angleM gyroM : Float) : BalanceCar := do
  let mut car := car
  car := {car with angle := car.angle + (gyroM - car.qBias) * dt}
  let angleErr : Float := angleM - car.angle
  car := {car with pDot0 := qAngle - car.p01 - car.p10}
  car := {car with pDot1 := - car.p11}
  car := {car with pDot2 := - car.p11}
  car := {car with pDot3 := qGyro}
  car := {car with p00 := car.p00 + car.pDot0 * dt}
  car := {car with p01 := car.p01 + car.pDot1 * dt}
  car := {car with p10 := car.p10 + car.pDot2 * dt}
  car := {car with p11 := car.p11 + car.pDot3 * dt}
  let PCt_0 : Float := c0 * car.p00
  let PCt_1 : Float := c0 * car.p10
  let E : Float := rAngle + c0 * PCt_0
  let K_0 : Float := PCt_0 / E
  let K_1 : Float := PCt_1 / E
  let t_1 : Float := c0 * car.p01
  car := {car with p00 := car.p00 - K_0 * PCt_0}
  car := {car with p01 := car.p01 - K_0 * t_1}
  car := {car with p10 := car.p10 - K_1 * PCt_0}
  car := {car with p11 := car.p11 - K_1 * t_1}
  car := {car with angle := car.angle + K_0 * angleErr} -- optimal angle
  car := {car with qBias := car.qBias + K_1 * angleErr}
  car := {car with angleDot := gyroM - car.qBias}
  car

def angleTest (car : BalanceCar) (ax ay az gx gy gz : Int) : BalanceCar := do
  let mut car := car
  let angle := (Float.atan2 ay az) * 57.3
  car := {car with gyroX := ((Float.ofInt gx) - 128.1) / 131.0}
  car := car.kFilter angle car.gyroX
  let gz : Int := if (gz > 32768) then gz - 65536 else gz
  let angleAx : Float := (Float.atan2 ax az) * 180.0 / Float.pi
  car := {car with gyroY := Float.div (-gy) 131.0}
  car := {car with gyroZ := Float.div (-gz) 131.0}
  car := {car with angle6 := k1 * angleAx + (1 - k1) * (car.angle6 + car.gyroY * dt)}
  car

-- 50ms speed loop control delay
def speedPIDelayCount : Nat := 10
-- every 10ms, enter rotation control
def turnSpinDelayCount : Nat := 2


def initial : BalanceCar := {
  ctrl := Controller.initial,
  countLeft  := 0,
  countRight := 0,
  pulseLeft  := 0,
  pulseRight := 0,
  stopLeft := 0,
  stopRight := 0,
  angleOut := 0,
  pwm1 := 0,
  pwm2 := 0,
  speedFilter := 0,
  positions := 0,
  turnSpinDelay := turnSpinDelayCount,
  turnMax := 0,
  turnMin := 0,
  turnOut := 0,
  resetPosition := false,
  speedPI := 0,
  turnSpin := 0,
  speedPIDelay := speedPIDelayCount,
  gyroX := 0,
  gyroY := 0,
  gyroZ := 0,
  angle := 0,
  angle6 := 0,
  qBias := 0,
  angleDot := 0,
  p00 := 1,
  p01 := 0,
  p10 := 0,
  p11 := 1,
  pDot0 := 0,
  pDot1 := 0,
  pDot2 := 0,
  pDot3 := 0
}



-- Update cars speed PI (Proportional Integral)
def updateSpeedPI (car : BalanceCar) : BalanceCar := do
  let mut car := car
  let speeds : Float := Float.ofInt (car.pulseLeft + car.pulseRight)
  car := {car with pulseLeft := 0, pulseRight := 0}
  car := {car with speedFilter := car.speedFilter * 0.7 + speeds * 0.3}
  car := {car with positions := car.positions + car.speedFilter + car.ctrl.forward + car.ctrl.reverse}
  car := {car with positions := Float.constrain car.positions (-3550) 3550}
  car := {car with speedPI := kiSpeed * (p0 - car.positions) + kpSpeed * (p0 - car.speedFilter)}
  car := {car with positions := if car.resetPosition then 0 else car.positions}
  car


def updateTurnSpin (car : BalanceCar) : BalanceCar := do
  let mut car := car
  let mut turnSpeed : Float := 0
  let mut rotationRatio : Float := 0
  if (car.ctrl.turnLeft || car.ctrl.turnRight || car.ctrl.spinLeft || car.ctrl.spinRight) then do
    -- Judge the current speed before rotating to enhance the adaptability of the car.
    turnSpeed := Float.ofInt (car.pulseRight + car.pulseLeft)
    if (turnSpeed < 0) then do
      turnSpeed := -turnSpeed
    if (car.ctrl.turnLeft || car.ctrl.turnRight) then do
      car := {car with turnMax := 3}
      car := {car with turnMin := -3}
    if (car.ctrl.spinLeft || car.ctrl.spinRight) then do
      car := {car with turnMax := 10}
      car := {car with turnMin := -10}
    rotationRatio := Float.constrain (5.0 / turnSpeed) 0.5 5.0
  else do
    rotationRatio := 0.5
  if (car.ctrl.turnLeft || car.ctrl.spinLeft) then do
    car := {car with turnOut := car.turnOut + rotationRatio}
  else if (car.ctrl.turnRight|| car.ctrl.spinRight) then do
    car := {car with turnOut := car.turnOut - rotationRatio}
  else do
    car := {car with turnOut := 0}
  car := {car with turnOut := Float.constrain car.turnOut car.turnMin car.turnMax}
  car := {car with turnSpin := (- car.turnOut) * kpTurn - car.gyroZ * kdTurn}
  car


-- speedoutput and rotationoutput are values on the BalanceCar (speedPI and turnSpin)
def pwma (car : BalanceCar) : BalanceCar := do
  let mut car := car
  -- Left motor PWM output value
  car := {car with pwm1 := Float.constrain ((- car.angleOut) - car.speedPI - car.turnSpin) (-255) 255}
  -- Right motor PWM output value
  car := {car with pwm2 := Float.constrain ((- car.angleOut) - car.speedPI + car.turnSpin) (-255) 255}
  if car.angle > 30.0 || car.angle < (Float.neg 30.0) then do
    -- If the angle is too large, stop the motor
    car := {car with pwm1 := 0}
    car := {car with pwm2 := 0}
  if (car.angle6 > 10 || car.angle6 < -10)
     && !car.ctrl.turnLeft 
     && !car.ctrl.turnRight
     && !car.ctrl.spinLeft
     && !car.ctrl.spinRight
     && car.ctrl.forward == 0
     && car.ctrl.reverse == 0 then
    if car.stopLeft + car.stopRight > 1500 || car.stopLeft + car.stopRight < -3500 then do
      car := {car with pwm1 := 0}
      car := {car with pwm2 := 0}
      car := {car with resetPosition := true}
  else do
    car := {car with stopLeft := 0}
    car := {car with stopRight := 0}
    car := {car with resetPosition := false}
  car


def countPulse (car : BalanceCar) : BalanceCar := do
  let mut car := car
  let mut pulseLeft := car.countLeft
  let mut pulseRight := car.countRight
  car := {car with countLeft := 0,
                   countRight := 0}
  if (car.pwm1 < 0.0) && (car.pwm2 < 0.0) then do
    -- Judgment of the direction of movement of the trolley.
    -- When moving backwards (PWM means the motor voltage is negative), 
    -- the number of pulses is negative.
    pulseRight := -pulseRight
    pulseLeft := -pulseLeft
  else if (car.pwm1 < 0.0) && (car.pwm2 > 0.0) then do
    -- Judgment of the direction of movement of the trolley.
    -- When moving forward (PWM, that is, the motor voltage is
    -- positive), the number of pulses is negative.
    pulseLeft := -pulseLeft
  else if (car.pwm1 > 0.0) && (car.pwm2 < 0.0) then do
    -- Judgment of the direction of movement of the trolley.
    -- Rotate left, the number of right pulses is negative,
    -- and the number of left pulses is positive.
    pulseRight := -pulseRight
  car := {car with stopLeft := car.stopLeft + pulseLeft}
  car := {car with stopRight := car.stopRight + pulseRight}
  car := {car with pulseLeft := car.pulseLeft + pulseLeft}
  car := {car with pulseRight := car.pulseRight + pulseRight}
  car
  


-- Interrupt triggered every 5ms to steer car via the two motors
def update (car : BalanceCar) (ax ay az gx gy gz : Int) : BalanceCar := do
  let mut car : BalanceCar := car
  car := countPulse car
  car := car.angleTest ax ay az gx gy gz
  car := {car with angleOut := kp * (car.angle + angle0) + kd * car.gyroX}  
  car := {car with turnSpinDelay := car.turnSpinDelay - 1}
  if car.turnSpinDelay == 0 then do
    car := car.updateTurnSpin
    car := {car with turnSpinDelay := turnSpinDelayCount}
  car := {car with speedPIDelay := car.speedPIDelay - 1}
  if car.speedPIDelay == 0 then do
    car := car.updateSpeedPI
    car := {car with speedPIDelay := speedPIDelayCount}
  car := car.pwma
  pure car


end BalanceCar

inductive Cmd
  | forward
  | reverse
  | turnLeft
  | turnRight
  | spinLeft
  | spinRight
  | stop

open Cmd

def commandCar (car : BalanceCar) : Cmd → BalanceCar
  | forward   => {car with ctrl := {car.ctrl with forward := 700}}
  | reverse   => {car with ctrl := {car.ctrl with reverse := -700}}
  | turnLeft  => {car with ctrl := {car.ctrl with turnLeft := true}}
  | turnRight => {car with ctrl := {car.ctrl with turnRight := true}}
  | spinLeft  => {car with ctrl := {car.ctrl with spinLeft := true}}
  | spinRight => {car with ctrl := {car.ctrl with spinRight := true}}
  | stop      => {car with ctrl := Controller.initial}



@[extern "lean_initialize_serial_port"]
constant initializeSerialPort (portPath : @& String) (baudRate : UInt16) : IO Unit
-- @[extern "lean_digital_pin_write"] 
-- constant digitalPinWrite (pin : UInt8) (value : Bool) : IO Unit
-- @[extern "lean_analog_pin_write"]
-- constant analogPinWrite  (mode1 mode2 : UInt8) (value1 value2 : Float) : IO Unit
@[extern "lean_start_car"] constant startCar : IO Unit
@[extern "lean_drive_car"]
constant driveCar (mode1 mode2 : Bool) (value1 value2 : Float) : IO Unit
@[extern "lean_rx_int16_as_int"] 
constant rxInt16AsInt : IO Int
@[extern "lean_rx_long_as_int"] 
constant rxLongAsInt : IO Int
@[extern "lean_wait_for_header"] 
constant waitForHeader : IO Unit

-- @[extern "lean_rx_char_as_uint32"] 
-- constant rxCharAsUInt32 : IO UInt32
-- def rxChar : IO Char := do
--   let n ← rxCharAsUInt32
--   if h : isValidChar n
--   then pure ⟨n, h⟩
--   else do
--     IO.eprintln "WARNING: could not interpret nat is char."
--     pure ' '


partial -- intended to loop indefinitely
def controlLoop (car : BalanceCar) : IO Unit := do
  waitForHeader
  let l ← rxLongAsInt
  let r ← rxLongAsInt
  let ax ← rxInt16AsInt
  let ay ← rxInt16AsInt
  let az ← rxInt16AsInt
  let gx ← rxInt16AsInt
  let gy ← rxInt16AsInt
  let gz ← rxInt16AsInt
  let mut car := {car with countLeft := l, countRight := r}
  car := car.update ax ay az gx gy gz
  let (mode1, pwm1) := if car.pwm1 >= 0.0 
                       then (true, car.pwm1)
                       else (false, (-car.pwm1))
  let (mode2, pwm2) := if car.pwm2 >= 0.0
                       then (true, car.pwm2)
                       else (false, (-car.pwm2))
  driveCar mode1 mode2 pwm1 pwm2
  controlLoop car


private def printSupportedBaudRates : IO Unit :=
  IO.println   "  Supported baud rates: 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200."



-- - - - - - - - - - - - - - - - - - - - - - - - -
-- Debugging/Simulation Code
-- - - - - - - - - - - - - - - - - - - - - - - - -

structure DebugSample where
  iteration : Int
  countLeft : Int
  countRight : Int
  ax : Int
  ay : Int
  az : Int
  gx : Int
  gy : Int
  gz : Int
  pwm1 : Float
  pwm2 : Float

namespace DebugSample

open Lean

def parseInt (lineNum : Nat) (str : String) : IO Int :=
  match Json.parse str with
  | Except.ok js => 
    match js.getInt? with
    | none => throw $ IO.userError s!"Expected an Int on line {toString lineNum} but got {str}"
    | some n => pure n
  | Except.error e => throw $ IO.userError s!"Expected an Int on line {toString lineNum} but got {str}"

def parseFloat (lineNum : Nat) (str : String) : IO Float :=
  match str.split (λ x => x == '.') with
  | [lhs,rhs] => do
    let n ← if lhs == "0"
            then do
              let rhs' := rhs.dropWhile (λ c => c == '0')
              if rhs' == "" then pure 0 else parseInt lineNum rhs'
            else if lhs == "-0" 
              then do
                let rhs' := rhs.dropWhile (λ c => c == '0')
                if rhs' == "" then pure 0 else parseInt lineNum ("-"++rhs')
            else parseInt lineNum (lhs ++ rhs)
    let len := rhs.length
    pure $ (Float.ofInt n) / (Float.ofNat (10 ^ len))
  | _ => throw $ IO.userError s!"Expected a simple decimal literal on line {toString lineNum} but got {str}"

end DebugSample

section
open DebugSample

def parseSamples (dataFile : String) : IO (Array DebugSample) := do
  IO.println s!"Parsing samples from {dataFile}"
  let mut lastIter := -1
  let mut samples := #[]
  let lines ← IO.FS.lines dataFile 
  for l in lines, lNum in [0:lines.size] do
    match (l.split Char.isWhitespace).filter (λ s => s != "") with
    | [i,l,r,ax,ay,az,gx,gy,gz,pmw1,pmw2] => do
      let i := ← parseInt lNum i
      let l ← parseInt lNum l
      let r ← parseInt lNum r
      let ax ← parseInt lNum ax
      let ay ← parseInt lNum ay
      let az ← parseInt lNum az
      let gx ← parseInt lNum gx
      let gy ← parseInt lNum gy
      let gz ← parseInt lNum gz
      let pwm1 ← parseFloat lNum pmw1
      let pwm2 ← parseFloat lNum pmw2
      if i != (lastIter + 1) then
        throw $ IO.userError s!"expected iteration {toString (lastIter+1)} but got {toString i}"
      lastIter := i
      let s :=
        {iteration := i,
         countLeft := l,
         countRight := r,
         ax := ax,
         ay := ay,
         az := az,
         gx := gx,
         gy := gy,
         gz := gz,
         pwm1 := pwm1,
         pwm2 := pwm2}
      samples := samples.push s
    | _ =>
      throw $ IO.userError s!"expected 11 space separated numbers but got {l}"
  pure samples

end



-- A control loop that compares our Lean implementation to values reported from
-- the BalanceCar using the original C/Arduino code.
def debugControlLoop (initCar : BalanceCar) (samples : Array DebugSample) : IO Unit := do
  let ε := 2.5
  IO.println s!"Running simulation on {toString samples.size} samples (i.e., {toString (samples.size / 200)} seconds of data)..."
  let mut car := initCar
  for s in samples do
    car := {car with countLeft := s.countLeft, countRight := s.countRight}
    car := car.update s.ax s.ay s.az s.gx s.gy s.gz
    if Float.abs (s.pwm1 - car.pwm1) > ε then
      throw $ IO.userError s!"[iteration {toString s.iteration}] Expected pwm1 approx. {toString s.pwm1} but got {toString car.pwm1}"
    if Float.abs (s.pwm2 - car.pwm2) > ε then
      throw $ IO.userError s!"[iteration {toString s.iteration}] Expected pwm2 approx. {toString s.pwm2} but got {toString car.pwm2}"
  IO.println s!"All calculated PWM values were within {toString ε} of their expected value!"

def debugMain (args : List String) : IO Unit := do
  if args == [] then do
    throw $ IO.userError "Expected one or more data files as command line arguments."
  for dataFile in args do
    IO.println s!"Data file `{dataFile}`"
    debugControlLoop BalanceCar.initial (← parseSamples dataFile)


def debugMode := false


def main (args : List String) : IO Unit :=
  if debugMode then debugMain args
  else do
    let invalidBaudRate : String → IO Unit := λ rate => do
      IO.println $ "Invalid baude rate: " ++ rate
      IO.println   "Supported baud rates: 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200."
    match args with
    | [port, rate] =>
      match String.toNat? rate with
      | some n =>
        match BaudRate.ofNat? n with
        | some bps => do
          initializeSerialPort port bps.toUInt16
          startCar
          controlLoop BalanceCar.initial
        | none => invalidBaudRate rate
      | none => invalidBaudRate rate
    | _ => do
      IO.println "usage: `balance-car PORT BAUDRATE`"
      IO.println "e.g., `balance-car /dev/ttyS0 115200`"
      IO.println "Supported baud rates: 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200."
