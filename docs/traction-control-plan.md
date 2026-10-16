# Closed-loop traction control for rusEFI on uaEFI (FSAE): implementation plan

Status: plan only, nothing implemented yet.
Code base: rusEFI `master` @ `6cf917c59e68` (2026-10-16), imported into this repo (see `MR2-FIRMWARE.md`).
Target: uaEFI (STM32F4) or uaEFI Pro (STM32F7) in a Formula SAE car with an electronic throttle body and four wheel speed sensors.

File references below are `path:line` against that base commit.

---

## 1. Goal and scope

Regulate driven-wheel slip to a target that depends on speed and driver-selected level, using four wheel speeds. Act through:
- spark retard
- spark cut
- electronic throttle closing

Requirements:
- Tunable in TunerStudio and logged on the SD card.
- Fail-safe: any sensor fault or doubt disables TC, which means no torque reduction. TC may never open the throttle beyond the driver's demand.
- The existing open-loop TC tables keep working, so the change can be upstreamed to rusEFI.

Out of scope for v1: yaw/stability control, torque vectoring, ABS, and redesigning launch control. TC hands off to the existing launch control and does not replace it.

## 2. Assumptions to confirm

| # | Assumption | Why it matters |
|---|---|---|
| A1 | Rear-wheel drive, chain to a limited-slip diff or spool; front wheels undriven | Front wheels give the reference (vehicle) speed. LSD/open diff vs. spool decides whether "driven speed" is max(RL,RR) or mean(RL,RR) |
| A2 | Motorcycle engine. Single cylinder or inline four? | Spark-cut granularity. A single at 10,000 rpm fires 83 times/s (12 ms between sparks), an I4 333 times/s. For a single, retard and ETB matter more and the cut pattern must be deterministic |
| A3 | Intake restrictor (20 mm gasoline / 19 mm E85) downstream of the throttle | Above a certain throttle opening, airflow is choked by the restrictor, so ETB authority has a dead band (section 5.5) |
| A4 | uaEFI (F4, 1 MB) or uaEFI Pro (F7, 2 MB)? | F4 flash is already tight. `firmware/config/boards/hellen/uaefi/board.mk:21` says "1mb is not enough for everything we have" |
| A5 | 3-wire Hall wheel sensors with open-collector/push-pull output, 20-40 teeth (rotor bolts or tone ring) | Connects directly to uaEFI Hall/digital inputs. 2-wire active (current-output) sensors need an external interface |
| A6 | Same tire size front and rear, about 18 in OD | Per-axle circumference, per-wheel trim |
| A7 | Brake pressure sensor or brake switch available; sequential gearbox with flat-shift cut | Inhibit TC under braking, freeze it during shift cut |
| A8 | Crank on VR2 (C17), cam on Hall 1 (C5), the board defaults | Determines which inputs are free for wheel sensors (section 5.1) |
| A9 | Driver wants an on/off switch and a level knob (or levels from a CAN dash) | Driver interface (section 5.6) |

## 3. What rusEFI already has

### 3.1 Existing traction-control scaffolding (open loop)

- **Slip source `SensorType::WheelSlipRatio`** (`firmware/controllers/sensors/sensor_type.h`), registered by one of:
  - `firmware/init/sensor/init_aux_speed_sensor.cpp`: `auxSpeed1 * auxSpeed1Multiplier / auxSpeed2` (or `/ VehicleSpeed`). Only two frequency inputs (`AUX_SPEED_SENSOR_COUNT 2`, `rusefi_config.txt:1378`).
  - `firmware/controllers/can/can_vss.cpp`: front/rear axle ratio decoded from a few OEM ABS frames (Hyundai, Nissan).
  - The two sources define the ratio differently (aux1/aux2 vs. front/rear). Neither has a four-wheel model or per-wheel calibration.
- **Three 6x6 tables** indexed by slip ratio x vehicle speed (`rusefi_config.txt:139-140`, `1608-1609`, `1785`, `1793-1794`):
  - `tractionControlTimingDrop` -> added to advance in `getRunningAdvance()` (`firmware/controllers/algo/ignition/ignition_state.cpp:82-88`).
  - `tractionControlIgnitionSkip` -> `engineState.tractionControlSparkSkip` -> soft or hard `SoftSparkLimiter` (`firmware/controllers/algo/engine2.cpp:30-43`, `launch_control.cpp:205-238`, selected by `useHardSkipInTraction`).
  - `tractionControlEtbDrop` -> added (in %) to the ETB target in `EtbController::getSetpointEtb()` (`firmware/controllers/actuators/electronic_throttle.cpp:346-353`).
- TunerStudio: three table editors plus a slip gauge (`firmware/tunerstudio/tunerstudio.template.ini:1216-1229`, `2083-2085`, `5647-5653`). Guard flag `ts_show_traction_control` (`rusefi_config.txt:704`).
- Unit tests: `unit_tests/tests/actuators/test_etb.cpp:875`, `unit_tests/tests/ignition_injection/test_ignition_state.cpp:88,121`, `unit_tests/tests/launch/test_launch_target_skip_ratio.cpp`.

### 3.2 Gaps

- No target, no closed loop, no integrator or ramp-out. Intervention simply follows the instantaneous slip.
- No enable switch, no inhibit conditions (braking, low speed, launch, shift), no driver level.
- No fault handling. `Sensor::getOrZero(WheelSlipRatio)` turns a missing sensor into slip 0, which clamps to the first table row.
- Additive ETB drop: a -20 % cell closes a 15 % throttle completely, and the drop is applied before the ETB rev limiter.

### 3.3 Bugs found while reading (fix with test-first commits, section 6, Phase 3)

1. **Spark-skip units.** `tractionControlIgnitionSkip` is declared as `"%", 0..100` (`rusefi_config.txt:1794`), but it is added raw into `targetSkipRatio`. `SoftSparkLimiter::shouldSkip()` compares that ratio to a random number in [0,1) (`launch_control.cpp:230-238`). So any cell >= 1 means 100 % cut, and on the soft limiter >= 0.5 already means 100 %. Launch control divides its percentages by 100; the TC path does not.
2. **Speed-bin units.** `tractionControlSpeedBins` is labelled `"RPM"` (`rusefi_config.txt:1609`), but it is indexed by `VehicleSpeed` (km/h), and TunerStudio plots it against `vehicleSpeedKph`.

### 3.4 Infrastructure to reuse

| Need | Existing piece |
|---|---|
| Edge timestamps for wheel sensors | `efiExtiEnablePin()` (`firmware/hw_layer/digital_input/digital_input_exti.cpp`). The ISR timestamps edges, and callbacks run in a lower-priority hand-off IRQ. **Limit: 16 EXTI lines, one GPIO port per pin number** |
| Frequency -> value sensor | `FrequencySensor` (`firmware/controllers/sensors/frequency_sensor.cpp`), `VehicleSpeedConverter` |
| CAN-fed wheel speeds with no new firmware | Lua `Sensor.new("<SensorType name>"):set(v)` publishes any `SensorType` (`LuaSensor`, `firmware/controllers/lua/lua_hooks.cpp:510`) |
| Module lifecycle (200 Hz fast / 20 Hz slow / config change / engine stop) | `EngineModule` (`firmware/controllers/core/engine_module.h`), `firmware/controllers/modules/*` with `MODULES_LIST` |
| Timing actuation | Advance is computed at 200 Hz in `EngineState::periodicFastCallback()` (`engine2.cpp:117-118`) |
| Spark-cut actuation | Decided per spark event in `spark_logic.cpp:657-662` via `softSparkLimiter` / `hardSparkLimiter` |
| Throttle actuation | ETB loop at 500 Hz (`ETB_LOOP_FREQUENCY`, `electronic_throttle_impl.h:25`) |
| Brake / clutch / flat-shift inputs | `brakePedalPin`, `clutchUpPin`, `clutchDownPin`, `torqueReductionTriggerPin`, `ShiftTorqueReductionController`, `LaunchControlBase` |
| Gear | `GearDetector` -> `SensorType::DetectedGear` (`firmware/controllers/modules/gear_detector/`) |
| Lateral/longitudinal g | uaEFI has an on-board LIS2DH12 (`ONBOARD_MEMS_TYPE=LIS2DH12`, `firmware/hw_layer/sensors/accelerometer.cpp`: 50 Hz ODR, +/-4 g) |
| Logging | Live-data structs (`firmware/integration/LiveData.yaml`) -> TunerStudio channels and SD log; `sdCardLogFrequency` up to 250 Hz |
| Config compatibility | `applyDefaultsOrFixAfterBurn()` (`firmware/controllers/algo/defaults/default_base_engine.cpp:130`), `docs/calibration-compatibility.md` |

## 4. Architecture

```
 wheel sensor edges (EXTI)            or CAN frames -> Lua Sensor:set()
          |                                   |
          v                                   v
  WheelSpeedSensor x4  ------------>  SensorType::WheelSpeedFL/FR/RL/RR  [km/h]
  (per-edge timestamps, multi-tooth average, decay to zero between edges)
          |
          v   200 Hz
  SlipEstimator
     vRef   = f(FL, FR)           (mean | min | max)
     vDrv   = f(RL, RR)           (mean for spool, max for LSD/open)
     dv     = vDrv - vRef [km/h]; slip = dv / max(vRef, vFloor)
     aRef, aDrv = filtered d/dt  [g]
     valid  = all four fresh and plausible
          |
          v   200 Hz
  TractionController
     dvTarget = max(slipTarget(level, speed[, gear]) * vRef, minSlipSpeed)
     e        = dv - dvTarget
     request  = clamp(FF(aDrv - aRef) + Kp*e + I, 0, maxRequest)   [%]
     arming / inhibit state machine, integrator ramp-out
          |
          v
  ReductionAllocator           request -> three curves
     retardDeg   -> getRunningAdvance()            (200 Hz)
     cutRatio    -> spark limiter, per spark event
     etbReduce%  -> EtbController::getSetpointEtb() (500 Hz), multiplicative cap
```

Code layout (new):
```
firmware/controllers/modules/traction_control/
    traction_control.mk          # MODULES_* registration, like gear_detector.mk
    traction_control.h/.cpp      # EngineModule: wiring, config, live data
    traction_control_state.txt   # live-data struct -> LiveData.yaml
    slip_estimator.h/.cpp        # pure math, no HAL: unit-testable
    traction_controller.h/.cpp   # pure math, no HAL: unit-testable
firmware/controllers/sensors/wheel_speed_sensor.h/.cpp
firmware/init/sensor/init_wheel_speed_sensors.cpp
unit_tests/tests/traction_control/*.cpp
```

Sketch of the public surface:

```cpp
struct WheelSpeeds { float fl, fr, rl, rr; bool valid; };

class SlipEstimator {
public:
	struct Result { float vRef, vDrv, slipSpeed, slipRatio, aRefG, aDrvG; bool valid; };
	Result update(const WheelSpeeds& w, float dtSec);
};

class TractionController {
public:
	// returns torque reduction request 0..100 %
	float update(const SlipEstimator::Result& s, const TcConditions& c, float dtSec);
	void reset();
};

class TractionControl : public EngineModule, public traction_control_state_s {
public:
	void onFastCallback() override;                 // 200 Hz
	void onConfigurationChange(engine_configuration_s const*) override;
	void onEngineStop() override;
	angle_t getTimingRetard() const;    // deg, >= 0
	float getSparkCutRatio() const;     // 0..1
	float getEtbReduction() const;      // 0..1 of the driver's throttle target
};
```

Ordering note: `Engine::periodicFastCallback()` runs `engineState.periodicFastCallback()` (which computes advance) **before** the modules' `onFastCallback()` (`firmware/controllers/algo/engine.cpp:601-611`). If TC lives purely in `onFastCallback()`, timing reacts one 5 ms tick late. That is acceptable for v1. If logs show it matters, call `TractionControl::update()` explicitly at the top of `Engine::periodicFastCallback()`.

## 5. Component design

### 5.1 Wheel speed sensing

**uaEFI input candidates.** The pin -> MCU mapping comes from `firmware/config/boards/hellen/uaefi/connectors/*.yaml` and `hellen_mm100_meta.h`. Confirm it against the uaEFI schematic before wiring, especially pull-up voltage and input RC filter cut-off.

| uaEFI pin | Conditioning (per yaml) | MCU | EXTI line | Board default |
|---|---|---|---|---|
| C17 | VR2 (MAX9924) / Hall | PE1 | 1 | crank |
| C18 | VR1 discrete, "for low count wheel" | PE0 | 0 | free |
| C5 | Hall 1, 4.7k pull-up | PE12 | 12 | cam 1 |
| C6 | Hall 2, 4.7k pull-up | PE13 | 13 | cam 2 (free with one cam) |
| C7 | Hall 3 / VSS, 4.7k pull-up | PE14 | 14 | VSS |
| D5 | Digital / flex, 10k pull-up | PE11 | 11 | free |
| C9 | Button 3, 10k pull-down (BT3 option) | PE15 | 15 | free, needs pull-up for OC sensor |
| D10 | Button 2 / A/C, 10k pull-down (BT2 option) | PA6 | 6 | free, needs pull-up |
| C15 | AUX3 analog, 500k pull-down | PA7 | 7 | free, analog front end |
| D2 | Button 1, 10k pull-down | PB1 | 1 | **conflicts with crank (EXTI 1)** |
| D1 | AUX1 analog, 500k pull-down | PA0 | 0 | **conflicts with VR1 (EXTI 0)** |

Proposed allocation: FL = C6 (PE13), FR = C7 (PE14), RL = D5 (PE11), RR = C9 (PE15) with an added pull-up. C18/VR1 is the alternative for a VR wheel sensor. Crank stays on C17, cam on C5. If the flex input is needed, move one wheel to D10 or C18.

**Frequency budget.** f = v / (pi * D) * N.
- 18 in tire (circumference 1.436 m), 24 teeth: 130 km/h -> 603 Hz; 2 km/h -> 9.3 Hz (108 ms between edges).
- Four wheels at top speed is about 2.4 k edges/s, alongside the crank trigger. Fine for the EXTI hand-off queue, but measure it (Phase 1).
- The Hall inputs' RC filter was designed for cam signals. Confirm its cut-off is well above 600 Hz.

**Why not reuse `FrequencySensor` as is.** It computes 1/period from a single edge, then runs a biquad once per edge. The effective filter cut-off therefore scales with wheel speed, and tooth-spacing errors from stamped rotors show up as noise. With a 500 ms timeout and no decay, a wheel that stops keeps reporting its last speed for half a second, which is bad at launch.

**`WheelSpeedSensor` (new, derives from `StoredValueSensor` or `FrequencySensor`):**
- ISR/hand-off: push the edge timestamp into a small ring buffer, nothing else.
- 200 Hz consumer: speed = k teeth / (t_last - t_last-k). Pick k so the window is at least `wheelSpeedMinWindowMs` (e.g. 10 ms), capped at one revolution (N teeth), which averages out tooth-spacing error.
- Between edges: if `now - t_last` > the last period, cap speed at the value implied by the elapsed time. That gives a monotonic decay toward zero. Report 0 after `wheelSpeedTimeoutMs`.
- Plausibility: reject a sample implying more than about 3 g change against the previous value (configurable), and count rejects for diagnostics.
- New `SensorType`s: `WheelSpeedFL`, `WheelSpeedFR`, `WheelSpeedRL`, `WheelSpeedRR`, plus `VehicleReferenceSpeed` and `DrivenWheelSpeed` for logging and Lua.
  - `sensor_type.h` asks for specific sensors to go above the auxiliary block, but Lua `getSensorByIndex()` uses raw enum indices (`lua_hooks.cpp:116`). Check with upstream whether shifting indices is acceptable or whether to append at the end.
- Per-wheel config: input pin, tooth count. Per axle: rolling circumference. Per wheel: trim in 0.1 % steps (tire wear, stagger).
- Lifecycle: init/deinit in `sensorStartUpOrReconfiguration()` / `stopSensors()` (`firmware/init/sensor/init_sensors.cpp:102-171`), like the aux speed sensors, so a Burn re-applies pins live (`docs/hardware-reinit-and-power-cycle.md`).
- EXTI conflicts already raise `firmwareError` with a clear message (`digital_input_exti.cpp`). Add a `validateConfigOnStartUpOrBurn()` check so the conflict is reported at Burn time, with the human-readable field label (`[tag:user_facing_ini_labels]`).

**CAN alternative.** If wheel speeds come from another node or the dash, a Lua script with `Sensor.new("WheelSpeedFL")` (timeout via `setTimeout`) feeds the same sensors. Native TC then works unchanged. Expect added latency of one CAN period plus a Lua tick, and note that uaEFI F4 Lua RAM is small (`uaefi/board.mk:72-74` already strips Lua features).

**Vehicle speed.** Add `vehicleSpeedSource` (existing VSS input / front reference / CAN) so dash, speed-based tables, and `GearDetector` keep working. `GearDetector::getDriveshaftRpm()` uses `VehicleSpeed`. With a front-wheel reference, the detected gear flickers while the rears spin, so have it prefer `DrivenWheelSpeed` when present.

### 5.2 Slip estimation (200 Hz)

- `vRef` = mean(FL, FR) by default (option: min, max).
- `vDrv` = mean(RL, RR) for a spool; max(RL, RR) for LSD or open diff, so an inside-wheel flare counts.
- `dv` = vDrv - vRef [km/h].
- `slip` = dv / max(vRef, vFloor), with vFloor about 5 km/h. This is also published as the legacy `WheelSlipRatio`, using a defined direction (vDrv/vRef), so the open-loop tables and gauge keep working.
- `aRef`, `aDrv` = filtered derivatives in g. `aRef` is the car's longitudinal acceleration measured by the undriven wheels. Logged against slip, it gives the tire's mu-slip curve directly, which is how to pick slip targets (Phase 2).
- Only one provider may register `WheelSlipRatio` (`Register()` is fatal on duplicates). The config must make the source explicit: aux-speed, CAN, or four-wheel.
- **Cornering geometry.** In a kinematic turn the front axle centre runs on a larger radius than the rear axle centre: R_f = sqrt(R^2 + L^2). The fronts therefore read fast and TC sees negative slip.
  - L = 1.55 m: about 6-7 % at hairpin radii (R ~ 4-4.5 m) and about 1.5 % at R = 9 m.
  - v1 accepts this, so TC is slightly permissive in hairpins.
  - v2 compensates with R ~ v^2 / a_lat from the on-board accelerometer (needs a mounting-orientation setting) or a steering-angle sensor.
- **Validity.** All four wheels fresh and plausible. `vRef` rate below a plausible limit (e.g. 2.5 g); otherwise hold the last good `vRef` briefly, then declare a fault. On a fault TC is inhibited, the reason is logged, and there is no intervention.

### 5.3 Slip target

- `tcSlipTarget[level][speedBin]`: 8 levels x 8 speed bins, 0.1 % resolution. Level 0 means off.
- `tcMinSlipSpeed` (km/h): dvTarget = max(target * vRef, minSlipSpeed). This keeps low speed and launch stable where the ratio is ill-conditioned.
- Phase 4: per-gear multiplier curve (from `DetectedGear` or a gear position sensor).
- Starting point: racing slicks typically peak around 8-15 % longitudinal slip. Validate with logged aRef-vs-slip from straight-line runs before trusting any number.

### 5.4 Controller

- Error `e = dv - dvTarget` [km/h]. Positive means too much slip.
- `request = clamp(FF + Kp*e + I, 0, tcMaxRequest)` [% torque reduction].
- **FF (spin onset):** when `aDrv - aRef` exceeds `tcSpinAccelThreshold` (e.g. about 1.0-1.5 g for an FSAE car), add `curve(excess)`. The rears accelerating faster than the car can is the earliest sign of spin, before the slip error builds.
- **I:**
  - Integrates `Ki*e` only while armed and while (e > 0 or I > 0). Clamped to [0, tcMaxRequest] (anti-windup).
  - Decay when e < 0 is rate-limited by `tcRampOutRate` [%/s], so torque returns smoothly instead of re-spinning the tires.
- **D (optional):** on filtered dv rate.
- **Gain scheduling:** multiplier vs. speed (later vs. gear).
- **Arming and inhibit state machine.** One logged enum reason:
  - `ModeOff`, `SwitchOff`, `LevelZero`, `SensorFault`, `BelowMinSpeed`, `BelowMinPedal`, `BelowMinRpm`, `Braking`, `ClutchIn`, `ShiftCut`, `LaunchActive`, `Cranking`, `Armed`, `Active`.
  - `ShiftCut` freezes the integrator.
  - `LaunchActive` lets launch control own the cut. On release, TC arms with a configurable pre-load so the hand-off does not dump torque.
  - Any disarm ramps the request out. Engine stop resets the controller.
- Purpose-built rather than `PidIndustrial` (`firmware/util/math/efi_pid.h`). The conditional integration, ramp-out and feed-forward are the substance of TC and should be explicit and testable. Gains may still use `pid_s` for TunerStudio familiarity.

### 5.5 Torque-reduction allocation

| Actuator | Latency | Authority | Side effects |
|---|---|---|---|
| Spark retard | Next spark (advance updated at 200 Hz) | Moderate, limited before misfire | Exhaust gas and valve temperature |
| Spark cut | Next spark event | 0-100 % in steps of one event | Unburnt fuel -> afterfire. Random patterns are noisy at low event rates |
| ETB closing | Motor + plenum fill, tens to ~150 ms | Large, but **dead band at high openings with a restrictor** | Smooth; cuts fuel use and afterfire |

- Three 8-point curves map request -> `retardDeg`, `cutRatio` and `etbReduction`. Example shape: retard first (0-40 %), cut from about 30 %, ETB from about 20 % to bleed off sustained excess so the cut can recover.
- **Timing:** in `getRunningAdvance()`, `advance -= tc.getTimingRetard()` in closed-loop mode. The legacy table path stays in open-loop mode.
- **Cut:**
  - Feed `tc.getSparkCutRatio()` (0..1) into the existing limiter (`EngineState::updateSparkSkip()`), soft or hard per `useHardSkipInTraction`.
  - Add a deterministic error-diffusion pattern option (`acc += ratio; skip if acc >= 1; acc -= 1`). With a single cylinder, random skipping at 20 % over a handful of events is far off target.
  - Fix bug 3.3-1 first.
- **ETB:** in `EtbController::getSetpointEtb()`, after the ETB rev limiter and before the min/max clamp: `target = std::min(target, target * (1 - tc.getEtbReduction()))`.
  - This never exceeds driver demand. Fault, limp and autotune paths are untouched.
  - Phase 4: express the reduction in airflow terms. Use a throttle% -> relative-airflow curve, or `ThrottleModel`, and invert it, so a reduction near the restrictor-choked region actually removes airflow.
- **No fuel cut in v1.** Port-injection wall wetting gives a lean spike on recovery. It could be added later as an option for I4 engines.

### 5.6 Driver interface

- **Enable switch:** `tcEnableSwitchPin` + mode. If unassigned, TC is enabled whenever the mode is closed loop.
- **Level:** `tcLevelSource` = fixed (`tcFixedLevel`) / analog rotary knob (ADC channel; levels split evenly between two voltages) / Lua.
  - New hook `setTractionControlLevel(n)`, so a CAN dash can set it.
  - Also `setTractionControlInhibit(bool)`.
  - Every live-data field is readable from Lua via `getOutput()`.
- **Outputs:** optional `tcActiveLightPin`. Fill the existing Haltech dash TC bits (`firmware/controllers/can/can_dash_haltech.dbc:178-192`) and any rusEFI dash frame the team uses.

### 5.7 Configuration and TunerStudio

New fields go at the insertion anchor in `firmware/integration/rusefi_config.txt` (~line 2001, "historically 'unusedOftenChangesDuringFirmwareUpdate' was here"). **Zero must mean "off/neutral"** so existing tunes are unaffected.

Draft field list (about 150 bytes):
```
tractionControlMode              enum: Off(0), OpenLoopTables, Monitor, ClosedLoop
wheelSpeedInputPin[4]            brain_input_pin_e
wheelSpeedToothCount[4]          uint8
wheelCircumferenceFrontMm/RearMm uint16
wheelSpeedTrim[4]                int8, 0.1 %
wheelSpeedMinWindowMs, wheelSpeedTimeoutMs
tcReferenceSpeedMode, tcDrivenSpeedMode, vehicleSpeedSource   enums
tcMinSpeed, tcMinSlipSpeed, tcMinPedal, tcMinRpm
tcSlipTargetSpeedBins[8], tcSlipTarget[8 levels][8]
tcKp, tcKi, tcKd, tcMaxRequest, tcRampOutRate, tcSpinAccelThreshold, tcLaunchPreload
tcRequestBins[8], tcRetardCurve[8], tcCutCurve[8], tcEtbReductionCurve[8]
tcEnableSwitchPin + mode, tcLevelSource, tcLevelAdcChannel, tcLevelVoltLow/High, tcFixedLevel
bits: tcInhibitOnBrake, tcInhibitOnClutch, tcDeterministicCut
tcActiveLightPin
```

Migration in `applyDefaultsOrFixAfterBurn()`:
- An old tune with non-zero legacy tables and `tractionControlMode == 0` gets `OpenLoopTables`, so its behaviour is preserved.
- The skip-unit fix (3.3-1) is a behaviour change for those tunes. Call it out in the upstream PR.

TunerStudio, under a "Traction control" menu (`@@if_ts_show_traction_control`):
- **Wheel speed sensors:** pins, teeth, circumference, trims, four live gauges.
- **TC settings:** mode, enable conditions, gains.
- **Slip target table.**
- **Allocation curves.**
- **Driver controls.**
- **Gauges and indicators:** TC active, inhibit reason, request.
- Tooltips follow `[tag:user_facing_ini_labels]`.

### 5.8 Live data and logging

New `traction_control_state.txt`, registered in `LiveData.yaml`:
- four wheel speeds, vRef, vDrv, dv, slip, target, error
- P, I, FF terms; request
- retardDeg, cutRatio, etbReduction
- state/inhibit enum, level, aRef, aDrv, sample-reject counters

Log at 100-250 Hz on the SD card while tuning.

## 6. Phased implementation

Each phase is a self-contained series of commits that builds and passes unit tests. Sizes are relative (S/M/L).

### Phase 0: Baseline (S)
- `git submodule update --init --recursive`. Run `unit_tests/test.sh`. Build `compile_firmware.sh` (F4) and `compile_firmware_pro.sh` (F7).
- Record `flash0` headroom from the map files, as in `docs/firmware-flash-usage.md`. Decide F4 vs. Pro. If F4 is short, apply the documented options (e.g. `EFI_WIDEBAND_FIRMWARE_UPDATE=FALSE`) in this repo's board config.
- **Done when:** tests green, both images link, headroom recorded.

### Phase 1: Wheel speed sensors (M)
- `SensorType`s, `WheelSpeedSensor`, config fields, init/deinit wiring, Burn-time EXTI conflict check, TunerStudio dialog, live data, SD log fields, `vehicleSpeedSource`.
- **Unit tests:** frequency -> km/h with trims; multi-tooth averaging with jittered tooth spacing; decay to zero and timeout; plausibility reject; pin reconfiguration; Lua `Sensor` path feeding `WheelSpeed*`.
- **Bench:**
  - Drive the four inputs at known frequencies with a signal generator, or loop spare uaEFI low-side outputs to them and use Lua `startPwm` / `setPwmFreq`.
  - Check 0-150 km/h equivalents while self-stimulating the crank at redline.
  - Check CPU load (`threadsinfo`, perf counters).
- **Car:** log straight-line coast-downs and set per-wheel trims.

### Phase 2: Slip estimator and controller, monitor only (M)
- Module skeleton, estimator, target table, controller, inhibit state machine, `Monitor` mode (compute and log, no actuation).
- **Unit tests:** estimator (straight, cornering, spool vs. LSD modes, low-speed floor, faults); controller (step response, anti-windup, ramp-out rate, every inhibit reason, launch hand-off pre-load).
- **Closed-loop plant test.** A small deterministic longitudinal model runs in the host build, so gains are tuned and stability is checked before the car. It includes: car mass, rear wheel + driveline inertia, a mu-slip curve, engine torque times (1 - reduction), and actuator delays per path.
- **Car:** accel and skid-pad runs in Monitor. Confirm the request rises exactly where logs show wheelspin. Choose slip targets from aRef-vs-slip plots.

### Phase 3: Actuation (M)
- **Legacy bugs first**, following upstream's test-driven bugfixing rule (`CLAUDE.md`): a coverage commit that asserts current behaviour, then a fix commit that flips the expectations. Covers 3.3-1 (skip %) and 3.3-2 (speed-bin units).
- Allocation curves. Timing, cut (with deterministic pattern option) and ETB cap integration.
- **Unit tests** (`EngineTestHelper`, mocked wheel speeds):
  - Advance reduced by the expected degrees.
  - Skip ratio within 0..1, with the expected cut fraction over N events.
  - ETB setpoint never above the TC-off setpoint.
  - With TC off, every output is bit-identical to the baseline.
- **Bench:** self-stimulated engine plus PWM wheel simulation. Scope the coil outputs for cut fraction; trace the ETB target.
- **Car:** retard only at low gains on low grip, then add cut, then ETB. Compare 75 m accel times and skid-pad behaviour with TC on and off.

### Phase 4: Refinement (M/L)
- Level knob, switch, Lua hooks, dash bits, TC light.
- Flat-shift freeze, launch hand-off tuning.
- Per-gear targets and gain scheduling.
- Cornering compensation (accelerometer lat-g or steering angle).
- Wheel-speed auto-calibration (button: average ratios during a straight coast, write trims).
- `GearDetector` on driven speed.
- Restrictor-aware ETB reduction in airflow terms.

### Phase 5: Upstream (S/M)
- One PR per phase against `rusefi/rusefi`, produced with `git format-patch` (see `MR2-FIRMWARE.md`).
- Follow upstream `CLAUDE.md`: cross-platform unit tests (GCC + clang), no heap, `[tag:...]` cross-references, a `docs/AI/traction_control.md` deep-dive page, and wiki docs.

## 7. Risks and open questions

- **Flash on F4.** Upstream already disables features on uaEFI F4 to fit. Measure in Phase 0, before writing code.
- **EXTI line conflicts.** Only lines that are unique by pin number can be used. Section 5.1 lists a conflict-free set for the default crank/cam pins.
- **Signal integrity.** Wheel sensor harness routing, shielding and grounding; Hall input filter cut-off; pull-ups on button inputs.
- **Restrictor.** ETB has little effect until it closes into the choked region; retard and cut carry the fast response.
- **Engine-specific limits.** Afterfire with spark cut, and exhaust-valve temperature with sustained retard. Watch EGT (uaEFI has a MAX31855 channel).
- **Rules.**
  - The FSAE rulebook has an Electronic Throttle Control section (IC.4 in the 2019-2021 editions). It covers TPS/APPS requirements, plausibility checks, a brake system encoder / plausibility device, and ETC documentation.
  - TC must only lower the throttle target and must leave those safety paths untouched.
  - The rulebook could not be fetched from this environment. Check the current edition, and whether TC changes need to appear in the team's ETC documentation.
- **CPU and latency.** Edge load plus 200 Hz computation. Confirm the ETB thread and fast-callback timing budgets on hardware.
- **Calibration compatibility.** All new fields are zero-safe. Migrations live in `applyDefaultsOrFixAfterBurn()`.
- **Upstream drift.** Sync regularly using the procedure in `MR2-FIRMWARE.md`; upstreaming early reduces it.

## 8. Placeholder starting calibration

These are placeholders only. Replace them with values from the Phase 2 plant test and track data.

| Parameter | Start |
|---|---|
| Slip target (levels 1..8) | 4, 6, 8, 10, 12, 14, 17, 20 % (flat across speed) |
| Min slip speed | 3 km/h |
| Min arming speed | 5 km/h (0 with launch hand-off) |
| Kp / Ki | 5 %/(km/h) / 20 %/(km/h*s) |
| Max request / ramp-out | 80 % / 100 %/s |
| Spin-accel threshold | 1.2 g |
| Retard curve | 0 -> 15 deg over request 0-40 % |
| Cut curve | 0 -> 60 % over request 30-100 % |
| ETB reduction curve | 0 -> 30 % over request 20-100 % |
