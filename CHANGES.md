# Changelog — Motor position snap prevention

## Problem

After either a **reboot** (`RebootDxl` service) or a **torque disable → enable** cycle,
the Dynamixel motors would immediately snap to the controller's *previous goal position*
rather than staying at their current physical position.

### Root cause

```
ros2_control control loop (every cycle):
  1. read()              → update joint states from motor
  2. controller.update() → controller writes its buffered goal
                           directly into hdl_joint_commands_
  3. write()             → CalcJointToTransmission() reads hdl_joint_commands_
                        → WriteMultiDxlData() sends it to motor
```

`hdl_joint_commands_` is the **same memory** as the command interfaces that
controllers (MoveIt / JTC) write to every control cycle.  Any sync performed
inside `start()` or `ChangeDxlTorqueState()` is overwritten by the controller
in the very next `controller.update()`, *before* `write()` can send it to the motor.

Additionally, the old `CommReset()` called `stop()` which disabled torque on
every motor before the reboot packet was sent — an unnecessary side effect.

---

## Changes

### `dynamixel_hardware_interface.hpp`

| What | Why |
|---|---|
| `#include <atomic>` | Required for the new atomic counter |
| `std::atomic<int> post_reboot_freeze_cycles_{0}` | Tracks remaining freeze cycles after reboot / torque-enable |
| `static constexpr int REBOOT_FREEZE_CYCLES = 200` | Duration of the freeze (~200 ms at 1 kHz). Tune if needed. |

---

### `dynamixel_hardware_interface.cpp`

#### 1. `CommReset()` — no more `stop()` during reboot

```diff
- stop();   // disabled torque on every motor before sending INST_REBOOT
+ // NOTE: we intentionally do NOT call stop() here so that torque is never
+ // disabled during a comm-reset / reboot sequence.
```

Motor torque is **not** disabled before reboot.  The hardware reboot packet
itself causes the motor to lose torque temporarily; there is no need to issue
an explicit disable command.

#### 2. `CommReset()` — snapshot present position after reboot

After `InitDxlReadItems()` / `InitDxlWriteItems()` succeed, each motor's
**Present Position** is read directly (bypassing the async sync-read buffer)
and immediately written back as **Goal Position**:

```cpp
for (const auto & pr : dxl_comm_id_id_) {
    ReadItem(comm_id, id, "Present Position", present_pos);
    WriteItem(comm_id, id, "Goal Position", present_pos);
}
```

This ensures that when `start()` calls `WriteMultiDxlData()` the motors
receive goal = current physical position, not a stale value.

#### 3. `CommReset()` — arm the post-reboot freeze

```cpp
post_reboot_freeze_cycles_ = REBOOT_FREEZE_CYCLES;
dxl_status_ = DXL_OK;
```

The counter is set **after** `start()` returns and **before** the write loop
resumes, so the freeze is guaranteed to be active on the very first `write()`
cycle.

#### 4. `write()` — freeze mechanism

```cpp
if (post_reboot_freeze_cycles_ > 0) {
    SyncJointCommandWithStates();   // override controller's stale goal
    post_reboot_freeze_cycles_--;
}
CalcJointToTransmission();
WriteMultiDxlData();
```

For `REBOOT_FREEZE_CYCLES` consecutive `write()` calls after a reboot or
torque-enable, `SyncJointCommandWithStates()` is called *before* the joint →
transmission conversion.  This overwrites whatever the controller wrote to the
command interfaces with the *actual* present position read from the motor,
preventing any snap.

After the freeze period the controller regains normal authority over the
command interfaces.

#### 5. `ChangeDxlTorqueState()` — freeze on torque enable

```cpp
if (dxl_torque_status_ == REQUESTED_TO_ENABLE) {
    dxl_comm_->DynamixelEnable(...);
    SyncJointCommandWithStates();
+   post_reboot_freeze_cycles_ = REBOOT_FREEZE_CYCLES;  // ← new
}
```

The disable → enable torque path suffers the identical stale-goal problem.
Arming the freeze counter here ensures the same protection applies.

---

## Sequence diagram (after fix)

```
reboot_dxl_srv_callback()
  │
  ├─ [pre-snapshot] ReadItem(Present Position) → WriteItem(Goal Position)
  │   (optional safety net; motor RAM will be reset by reboot anyway)
  │
  └─ CommReset()
        │
        ├─ dxl_status_ = REBOOTING  (blocks write loop)
        ├─ RWDataReset()
        ├─ INST_REBOOT → motor  (motor reboots, LED blinks)
        ├─ InitDxlReadItems() / InitDxlWriteItems()
        ├─ [post-snapshot] ReadItem(Present Position) → WriteItem(Goal Position)
        ├─ start()
        │     ReadMultiDxlData() → CalcTransmissionToJoint()
        │     SyncJointCommandWithStates() → CalcJointToTransmission()
        │     WriteMultiDxlData()   ← sends present pos as goal
        │     DynamixelEnable()     ← torque ON, goal = current pos ✓
        │
        ├─ post_reboot_freeze_cycles_ = 200
        └─ dxl_status_ = DXL_OK   (write loop resumes)

write() × 200 cycles:
   controller.update() → stale goal → hdl_joint_commands_
   write():
     SyncJointCommandWithStates()   ← freeze: override stale goal ✓
     CalcJointToTransmission()      ← present pos
     WriteMultiDxlData()            ← motor stays put ✓

write() cycle 201+:
   normal operation (controller commands obeyed)
```

---

## Suggested commit messages

```
fix(hardware): prevent motor snap on reboot/torque-enable due to stale controller goal

The root cause is that ros2_control controllers write their buffered goal
directly to hdl_joint_commands_ every control cycle.  Any one-shot sync
performed inside start() or ChangeDxlTorqueState() is silently overwritten
by the controller before write() can send the corrected value to the motor.

Introduce a post_reboot_freeze_cycles_ counter (atomic<int>) that causes
write() to call SyncJointCommandWithStates() — overriding the controller's
goal with the present motor position — for REBOOT_FREEZE_CYCLES (200) cycles
after a reboot or a torque-enable request.

Also remove the stop() call from CommReset(): disabling torque before sending
INST_REBOOT is unnecessary because the reboot packet itself causes the motor
to drop torque, and the explicit disable was causing an unwanted position snap
before the reboot even started.

Fixes: motor snapping to previous goal after RebootDxl service call
Fixes: motor snapping to previous goal after disable → enable torque cycle
```

```
fix(hardware): remove unnecessary stop() in CommReset()

CommReset() was calling stop() (= DynamixelDisable for all motors) before
sending the INST_REBOOT packet.  The reboot instruction already causes the
motor to drop torque internally, so the explicit disable is redundant and
introduced an extra torque-off event that could move the arm under gravity.
```

```
fix(hardware): snapshot present position after reboot before re-enabling torque

After Dynamixel reboots its RAM registers (including Goal Position) are
reset to hardware defaults.  CommReset() now reads each motor's Present
Position directly via ReadItem() and writes it back as Goal Position
immediately after InitDxlReadItems/InitDxlWriteItems succeed, before start()
calls WriteMultiDxlData() and DynamixelEnable().  This guarantees the motor
holds its current physical pose when torque is switched back on.
```
