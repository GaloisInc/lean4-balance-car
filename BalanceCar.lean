-- Port of roughly the following:
-- https://gitlab-int.galois.com/andrew/lean4-robotics-car-libs/-/blob/master/BalanceCar/examples/bst_abc/bst_abc.ino

namespace Float

instance : Coe Int Float := ⟨Float.ofInt⟩

@[extern "fabs"] constant abs : Float → Float

@[macroInline]
def constrain (n low high : Float) : Float :=
  if n < low then low
  else if n > high then high
  else n

def pi : Float := 3.1415926535897932384626433832795

end Float

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
def kpSpeed : Float := 28.0
def kiSpeed : Float := 0.05
def kdSpeed : Float := 0.0



structure Controller where
  forward   : Int -- Forward variable
  reverse   : Int -- Back variable
  turnLeft  : Bool -- Turn left sign
  turnRight : Bool -- Turn right sign
  spinLeft  : Bool -- Rotate left sign
  spinRight : Bool -- Right rotation sign

structure Gyro := (x y z : Float)

structure FourByOne (α : Type) := (val0 val1 val2 val3 : α)
structure TwoByTwo (α : Type) := (val00 val01 val10 val11 : α)


structure KalmanFilter :=
  (gyro : Gyro)
  (accelz angle angle6 angleErr qBias angleDot : Float)
  (p : TwoByTwo Float) 
  -- ^ Posteriori estimate covariance matrix for the Kalman filter,
  --   represented as a pair of matrix rows (where each row is also a pair)
  (pDot : FourByOne Float)

-- See also https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp 
-- for a... cleaner setup than the one we're mirroring at the moment

structure BalanceCar where
  ctrl : Controller
  filter : KalmanFilter
  countLeft  : Nat
  countRight : Nat
  pulseLeft  : Int
  pulseRight : Int
  stopLeft : Int
  stopRight : Int
  angleOut : Float
  pwm1 : Float
  pwm2 : Float
  speedFilter : Float
  positions : Float
  usePositions : Bool
  turnMax : Int
  turnMin : Int
  turnOut : Float
  speedPI : Float -- calculated by `updateSpeedPI`
  turnSpin : Float -- calculated by `updateTurnSpin`
  speedPIDelay : Nat



def KalmanFilter.initial : KalmanFilter := {
  gyro := {x := 0, y := 0, z := 0},
  accelz := 0,
  angle := 0,
  angle6 := 0,
  angleErr := 0,
  qBias := 0,
  angleDot := 0,
  p := ⟨1, 0, 
        0, 1⟩,
  pDot := ⟨0, 0, 0, 0⟩,
}

namespace KalmanFilter


-- Kalman Filter
def kFilter (k : KalmanFilter) (angleM gyroM : Float) : KalmanFilter := do
  let mut angle : Float := k.angle + (gyroM - k.qBias) * dt
  let angleErr : Float := angleM - angle
  let pDot0 : Float := qAngle - k.p.val01 - k.p.val10
  let pDot1 : Float := - k.p.val11
  let pDot2 : Float := - k.p.val11
  let pDot3 : Float := qGyro
  let mut p00 : Float := k.p.val00 + pDot0 * dt
  let mut p01 : Float := k.p.val01 + pDot1 * dt
  let mut p10 : Float := k.p.val10 + pDot2 * dt
  let mut p11 : Float := k.p.val11 + pDot3 * dt
  let PCt_0 : Float := c0 * p00
  let PCt_1 : Float := c0 * p10
  let E : Float := rAngle + c0 * PCt_0
  let K_0 : Float := PCt_0 / E
  let K_1 : Float := PCt_1 / E
  let t_1 : Float := c0 * p01
  p00 := p00 - K_0 * PCt_0
  p01 := p01 - K_0 * t_1
  p10 := p10 - K_1 * PCt_0
  p11 := p11 - K_1 * t_1;
  angle := angle + K_0 * angleErr; -- optimal angle
  let qBias := k.qBias + K_1 * angleErr;
  let angleDot : Float := gyroM - qBias; -- optimal angular velocity
  { k with
    angle := angle,
    angleErr := angleErr,
    qBias := qBias,
    p := ⟨p00, p01, p10, p11⟩,
    pDot := ⟨pDot0, pDot1, pDot2, pDot3⟩
  }

def angleTest (k : KalmanFilter) (ax ay az gx gy gz : Int) : KalmanFilter :=
  let angle : Float := (Float.atan2 ay az) * 57.3
  let gyroX : Float := ((Float.ofInt gx) - 128.1) / 131.0
  let k : KalmanFilter := kFilter k angle gyroX
  let gz : Int := if (gz > 32768) then gz - 65536 else gz
  let angleAx : Float := (Float.atan2 ax az) * 180.0 / Float.pi
  let gyroY : Float := Float.div (-gy) 131.0
  let gyroZ : Float := Float.div (-gz) 131.0
  { k with
    gyro := ⟨gyroX, gyroY, gyroZ⟩,
    accelz := Float.div az 16.4,
    angle6 := k1 * angleAx + (1 - k1) * (k.angle6 + gyroY * dt)
  }

end KalmanFilter




def Controller.initial : Controller :=
    ⟨0,0,false,false,false,false⟩ 


namespace BalanceCar

-- 50ms speed loop control delay
def speedPIDelayCount : Nat := 10

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
  turnMax := 0,
  turnMin := 0,
  turnOut := 0,
  usePositions := false,
  speedPI := 0,
  turnSpin := 0,
  filter := KalmanFilter.initial,
  speedPIDelay := speedPIDelayCount
}


-- Update cars speed PI (Proportional Integral)
def updateSpeedPI (car : BalanceCar) : BalanceCar := do
  let speeds : Float := Float.ofInt (car.pulseLeft + car.pulseRight)
  let speedFilter : Float := car.speedFilter * 0.7 + speeds * 0.3
  let mut positions : Float := car.positions + speedFilter + car.ctrl.forward + car.ctrl.reverse
  positions := Float.constrain positions (-3550) 3550
  positions := if car.usePositions then 0 else positions
  { car with 
    pulseRight := 0,
    pulseLeft := 0,
    speedFilter := speedFilter,
    positions := positions,
    speedPI := kiSpeed * (p0 - positions) + kpSpeed * (p0 - speedFilter),
    speedPIDelay := speedPIDelayCount
  }


def updateTurnSpin (car : BalanceCar) : BalanceCar := do
  let mut turnSpeed : Float := 0
  let mut rotationRatio : Float := 0
  let mut turnOut := car.turnOut
  let mut turnMin : Int := car.turnMin
  let mut turnMax : Int := car.turnMax
  if (car.ctrl.turnLeft || car.ctrl.turnRight || car.ctrl.spinLeft || car.ctrl.spinRight) then do
    -- Judge the current speed before rotating to enhance the adaptability of the car.
    turnSpeed := Float.ofInt (car.pulseRight + car.pulseLeft)
    if (turnSpeed < 0) then do
      turnSpeed := -turnSpeed
    if (car.ctrl.turnLeft || car.ctrl.turnRight) then do
     turnMax := 3
     turnMin := -3
    if (car.ctrl.spinLeft || car.ctrl.spinRight) then do
     turnMax := 10
     turnMin := -10
    rotationRatio := Float.constrain (5 / turnSpeed) 0.5 5
  else do
    rotationRatio := 0.5
  if (car.ctrl.turnLeft || car.ctrl.spinLeft) then do
    turnOut := turnOut + rotationRatio
  else if (car.ctrl.turnRight|| car.ctrl.spinRight) then do
    turnOut := turnOut - rotationRatio
  else do
    turnOut := 0
  turnOut := Float.constrain turnOut turnMin turnMax
  { car with
    turnOut := turnOut,
    turnMin := turnMin,
    turnMax := turnMax,
    turnSpin := (0 - turnOut) * kpTurn - car.filter.gyro.z * kdTurn
  }


-- speedoutput and rotationoutput are values on the BalanceCar (speedPI and turnSpin)
def pwma (car : BalanceCar) : BalanceCar := do
  let mut usePositions := car.usePositions
  let mut stopLeft := car.stopLeft
  let mut stopRight := car.stopRight
  -- Left motor PWM output value
  let mut pwm1 := Float.constrain ((- car.angleOut) - car.speedPI - car.turnSpin) (-255) 255
  -- Right motor PWM output value
  let mut pwm2 := Float.constrain ((- car.angleOut) - car.speedPI + car.turnSpin) (-255) 255
  if car.filter.angle > 30 || car.filter.angle < -30 then do
    -- If the angle is too large, stop the motor
    pwm1 := 0
    pwm2 := 0
  if car.filter.angle6 > 10 || car.filter.angle6 < -10 
     && !car.ctrl.turnLeft 
     && !car.ctrl.turnRight
     && !car.ctrl.spinLeft
     && !car.ctrl.spinRight
     && car.ctrl.forward == 0
     && car.ctrl.reverse == 0 then
    if car.stopLeft + car.stopRight > 1500 || car.stopLeft + car.stopRight < -3500 then do
      pwm1 := 0
      pwm2 := 0
      usePositions := true
    else do
      stopLeft := 0
      stopRight := 0
      usePositions := false
  { car with 
    usePositions := usePositions,
    stopLeft := stopLeft,
    stopRight := stopRight,
    pwm1 := pwm1,
    pwm2 := pwm2
  }


-- IN1M,IN2M,IN3M,IN4M,PWMA,PWMB
def IN1M : UInt32 := 7
def IN2M : UInt32 := 6
def IN3M : UInt32 := 13
def IN4M : UInt32 := 12
def PWMA : UInt32 := 9
def PWMB : UInt32 := 10


def incLeft (car : BalanceCar) : IO BalanceCar :=
  {car with countLeft := car.countLeft + 1}

def incRight (car : BalanceCar) : IO BalanceCar :=
  {car with countRight := car.countRight + 1}


def countPulse (car : BalanceCar) : BalanceCar := do
  let mut pulseLeft : Int := car.countLeft
  let mut pulseRight : Int := car.countRight
  if (car.pwm1 < 0) && (car.pwm2 < 0) then do
    -- Judgment of the direction of movement of the trolley.
    -- When moving backwards (PWM means the motor voltage is negative), 
    -- the number of pulses is negative.
    pulseRight := -pulseRight
    pulseLeft  := -pulseLeft
  else if (car.pwm1 < 0) && (car.pwm2 > 0) then do
    -- Judgment of the direction of movement of the trolley.
    -- When moving forward (PWM, that is, the motor voltage is
    -- positive), the number of pulses is negative.
    pulseLeft := -pulseLeft
  else if (car.pwm1 > 0) && (car.pwm2 < 0) then do
    -- Judgment of the direction of movement of the trolley.
    -- Rotate left, the number of right pulses is negative,
    -- and the number of left pulses is positive.
    pulseRight := -pulseRight
  { car with
    countLeft  := 0,
    countRight := 0,
    stopLeft   := car.stopLeft + pulseLeft,
    stopRight  := car.stopRight + pulseRight,
    pulseLeft  := car.pulseLeft + pulseLeft,
    pulseRight := car.pulseRight + pulseRight
  }
  


-- Interrupt triggered every 5ms to steer car via the two motors
def update (car : BalanceCar) (ax ay az gx gy gz : Int) : BalanceCar := do
  let mut car : BalanceCar := car
  car := countPulse car
  car := {car with filter := car.filter.angleTest ax ay az gx gy gz}
  car := {car with angleOut := kp * (car.filter.angle + angle0) + kd * car.filter.gyro.x}
  car := car.updateTurnSpin
  if car.speedPIDelay == 0 then do
    car := car.updateSpeedPI
  else do
    car := {car with speedPIDelay := car.speedPIDelay - 1}
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



-- TODO implement these two IO actions in C
@[extern "lean_digital_pin_write"] 
constant digitalPinWrite (pin : UInt8) (value : Bool) : IO Unit
@[extern "lean_analog_pin_write"]
constant analogPinWrite  (pin : UInt8) (value : Float) : IO Unit
@[extern "lean_rx_char_as_uint32"] 
constant rxCharAsUInt32 : IO UInt32
@[extern "lean_rx_int16_as_int"] 
constant rxInt16AsInt : IO Int

def rxChar : IO Char := do
  let n ← rxCharAsUInt32
  if h : isValidChar n
  then pure ⟨n, h⟩
  else do
    IO.eprintln "WARNING: could not interpret nat is char."
    pure ' '


unsafe 
def controlLoop (car : BalanceCar) : IO Unit := do
  match (← rxChar) with
  -- control interrupt
  | 'i' => do
    let ax ← rxInt16AsInt
    let ay ← rxInt16AsInt
    let az ← rxInt16AsInt
    let gx ← rxInt16AsInt
    let gy ← rxInt16AsInt
    let gz ← rxInt16AsInt
    let car := car.update ax ay az gx gy gz
    if car.pwm1 >= 0 then do
      digitalPinWrite pin2 false
      digitalPinWrite pin1 true
      analogPinWrite pinPWMA car.pwm1
    else do
      digitalPinWrite pin2 true
      digitalPinWrite pin1 false
      analogPinWrite pinPWMA (-car.pwm1)
    if car.pwm2 >= 0 then do
      digitalPinWrite pin4 false
      digitalPinWrite pin3 true
      analogPinWrite pinPWMB car.pwm2
    else do
      digitalPinWrite pin4 true
      digitalPinWrite pin3 false
      analogPinWrite pinPWMB (-car.pwm2)
    controlLoop car
  -- left/right counter interrupt
  | 'l' => controlLoop {car with countLeft := car.countLeft + 1}
  | 'r' => controlLoop {car with countLeft := car.countRight + 1}
  | _ => controlLoop car
  -- TODO support manual control of car (commandCar) in addition to balancing


unsafe
def main (args : List String) : IO Unit := do
  -- TODO setup connection?
  controlLoop BalanceCar.initial


