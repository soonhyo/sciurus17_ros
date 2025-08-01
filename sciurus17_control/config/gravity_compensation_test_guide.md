# Gravity Compensation Testing Guide

## 🛡️ **SAFETY FIRST - MANDATORY READING**

**DO NOT TEST ON REAL ROBOT WITHOUT FOLLOWING THIS PROTOCOL**

### Prerequisites
- Emergency stop button accessible
- Clear workspace around robot
- Monitor/supervisor present during testing
- Backup of working configuration

## 📋 **Testing Protocol**

### Phase 1: Simulation Testing (MANDATORY FIRST)
```bash
# If you have Gazebo simulation available
roslaunch sciurus17_gazebo sciurus17_world.launch
```

### Phase 2: Safe Real Robot Testing

#### Step 1: Launch Control Node (Gravity Compensation DISABLED by default)
```bash
# Terminal 1: Launch control node
rosrun sciurus17_control sciurus17_control

# Expected output:
# "Pinocchio model loaded successfully. Model has X DOF"
# "Gravity compensation initialized successfully"
```

#### Step 2: Verify Normal Operation (Without Gravity Compensation)
```bash
# Terminal 2: Check that gravity compensation is disabled
rosrun rqt_reconfigure rqt_reconfigure

# Navigate to: /sciurus17_control/gravity_compensation
# Verify: enable = false, gain = 0.1
```

#### Step 3: Test Normal Robot Control
```bash
# Test existing controllers to ensure normal operation
# Move robot with trajectory controllers
# Verify smooth, normal movement
```

#### Step 4: Enable Gravity Compensation (GRADUAL TESTING)
```bash
# Terminal 2: In rqt_reconfigure
# 1. Set debug_output = true (to see messages)
# 2. Keep gain = 0.1 (very low)
# 3. Set enable = true

# Watch for console output:
# "Gravity Compensation: ENABLED, Gain: 0.100"
```

#### Step 5: Monitor Robot Behavior
**Watch for these WARNING SIGNS:**
- ❌ Jerky movements → DISABLE IMMEDIATELY
- ❌ Oscillations → REDUCE GAIN or DISABLE
- ❌ Unusual servo sounds → STOP TESTING
- ❌ Unexpected motions → EMERGENCY STOP

**Good signs:**
- ✅ Smoother movements
- ✅ Less "drooping" when stationary
- ✅ Reduced servo current (if measurable)

#### Step 6: Gradual Gain Tuning (ONLY if Step 5 is successful)
```bash
# Gradually increase gain in small steps:
# 0.1 → 0.2 → 0.3 → 0.5 → 0.8 → 1.0 (maximum recommended)

# Test each gain level for 30 seconds before increasing
# If ANY issues occur, immediately reduce gain or disable
```

## 🔧 **Dynamic Reconfigure Parameters**

### Available Parameters:
- **enable** (bool): Enable/disable gravity compensation
  - Default: false (SAFE)
  - Use: Real-time on/off control

- **gain** (double): Compensation strength
  - Range: 0.0 - 2.0
  - Default: 0.1 (SAFE)
  - Recommended max: 1.0

- **debug_output** (bool): Show status messages
  - Default: false
  - Use: Monitoring during testing

### Quick Commands:
```bash
# Enable with low gain
rosrun rqt_reconfigure rqt_reconfigure

# Or via command line:
rosparam set /sciurus17_control/gravity_compensation/enable true
rosparam set /sciurus17_control/gravity_compensation/gain 0.1

# Emergency disable:
rosparam set /sciurus17_control/gravity_compensation/enable false
```

## 🚨 **Emergency Procedures**

### If Something Goes Wrong:
1. **Immediate**: Press physical emergency stop
2. **Quick disable**: `rosparam set /sciurus17_control/gravity_compensation/enable false`
3. **Restart node**: `Ctrl+C` the control node
4. **Check logs**: Look for error messages

### Recovery:
1. Restart with gravity compensation disabled
2. Check robot for any physical damage
3. Test normal operation without gravity compensation
4. Review what went wrong before retrying

## 📊 **Expected Results**

### Successful Gravity Compensation:
- Smoother position tracking
- Reduced steady-state errors
- Less servo load when holding positions
- Better performance on vertical movements

### Performance Metrics to Monitor:
- Position tracking accuracy
- Servo temperature
- Current consumption
- Control loop stability

## 🔍 **Troubleshooting**

### Common Issues:

1. **"Failed to initialize gravity compensation"**
   - Check robot_description parameter
   - Verify URDF is loaded
   - Continue testing without gravity compensation

2. **Oscillations/instability**
   - Reduce gain significantly
   - Check for mechanical backlash
   - Verify joint limits

3. **No noticeable improvement**
   - Increase gain gradually
   - Check joint directions in URDF
   - Verify Pinocchio model matches robot

## ✅ **Success Criteria**

Before considering deployment:
- [ ] All safety tests passed
- [ ] No instability at operational gains
- [ ] Measurable performance improvement
- [ ] Stable operation for extended periods
- [ ] Emergency disable procedures work

## 📝 **Test Log Template**

Date: ___________
Tester: ___________
Robot: ___________

**Phase 1 - Normal Operation (No Gravity Comp)**
- [ ] Control node starts successfully
- [ ] Robot moves normally
- [ ] No errors in logs

**Phase 2 - Gravity Compensation Testing**
- [ ] Enabled at gain 0.1 successfully
- [ ] No immediate issues observed
- [ ] Gain increased to: ______
- [ ] Final stable gain: ______
- [ ] Performance improvement observed: [ ] Yes [ ] No

**Issues Encountered:**
_________________________________

**Overall Result:** [ ] PASS [ ] FAIL [ ] NEEDS_IMPROVEMENT

---
**Remember: Safety is paramount. When in doubt, disable and investigate.**