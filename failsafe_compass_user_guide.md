# Failsafe Compass Mode - User Guide

## What is Failsafe Compass Mode?

Failsafe Compass Mode is a new emergency feature for ArduCopter that helps your drone escape from its current location when radio contact is lost. Unlike RTL (Return to Launch) which requires GPS, this mode only needs a compass and can work indoors or in areas with poor GPS reception.

When activated, your drone will:
1. Climb to a safe altitude
2. Turn to face a pre-set compass direction
3. Fly forward in that direction until you regain control

## When to Use This Mode

This mode is ideal for:
- Flying in GPS-denied environments (indoors, under bridges, in canyons)
- Areas with GPS interference
- Situations where you want the drone to fly AWAY from obstacles rather than return home
- Racing or freestyle flying where RTL might fly into obstacles

## How to Set It Up

### Step 1: Choose Your Escape Direction

Decide which compass direction you want your drone to fly when signal is lost:
- **0° = North**
- **90° = East**
- **180° = South**
- **270° = West**

Think about your flying area and choose a direction that leads to open space.

### Step 2: Configure Parameters

Using Mission Planner, QGroundControl, or another ground station:

1. **Set the escape heading:**
   - Find parameter: `FS_COMPASS_HDG`
   - Enter your chosen heading (0-359 degrees)
   - Example: Set to 180 to fly South

2. **Enable the failsafe mode:**
   - Find parameter: `FS_THR_ENABLE`
   - Set value to: `8`
   - Write parameters to save

3. **Verify RTL altitude (optional):**
   - Find parameter: `RTL_ALT`
   - This sets how high the drone climbs (in centimeters)
   - Default is usually 1500 (15 meters)
   - Adjust if needed for your flying area

## How It Works

When your transmitter signal is lost:

1. **Immediate Response**: The drone detects radio failsafe
2. **Climb Phase**: Climbs to RTL altitude to clear obstacles
3. **Turn Phase**: Rotates to face your preset compass heading
4. **Escape Phase**: Flies forward at 5 m/s (18 km/h) in that direction
5. **Recovery**: Continues until you regain radio control

## Important Safety Information

### Limitations
- **No obstacle avoidance** - The drone will fly straight into anything in its path
- **No position holding** - Wind will push the drone off course
- **No return home** - It keeps flying in one direction
- **Battery limits** - Will fly until battery dies if you don't regain control

### Pre-flight Checks
1. **Test your compass**: Ensure compass is calibrated and working properly
2. **Check heading**: Verify FS_COMPASS_HDG points to a safe direction
3. **Know your area**: Be aware of obstacles in your escape direction
4. **Battery planning**: Ensure you have enough battery for the escape flight

### Best Practices
- Choose an escape heading that leads to open areas
- Set RTL altitude high enough to clear all obstacles
- Practice regaining control in a safe area
- Monitor battery levels closely
- Have a spotter who can track the drone visually

## Regaining Control

When your radio link is restored:
1. Your transmitter will reconnect automatically
2. Move your throttle stick to regain control
3. Switch to your preferred flight mode
4. Fly back manually

## Comparison with Other Failsafe Options

| Failsafe Mode | Requires GPS | Returns Home | Good For |
|--------------|--------------|--------------|-----------|
| RTL | Yes | Yes | Outdoor flying with good GPS |
| Land | No | No (lands in place) | Safe landing zones |
| Compass | No | No (escapes) | GPS-denied areas, obstacle avoidance |

## Troubleshooting

**Drone doesn't enter Compass failsafe:**
- Verify FS_THR_ENABLE = 8
- Check radio failsafe is properly configured
- Ensure MODE_FAILSAFE_COMPASS_ENABLED in firmware

**Wrong direction:**
- Recalibrate compass
- Verify FS_COMPASS_HDG value
- Check for magnetic interference

**Not climbing:**
- Check RTL_ALT parameter
- Verify battery has enough power
- Check if already above RTL altitude

## Example Scenarios

### Indoor Flying
- Set heading toward the largest exit
- Lower RTL_ALT to avoid ceiling
- Have spotter at exit to catch drone

### Canyon/Urban Flying  
- Set heading toward open area
- Increase RTL_ALT above buildings
- Consider wind direction

### Racing Course
- Set heading away from course
- Moderate RTL_ALT to clear gates
- Brief other pilots about escape direction

## Summary

Failsafe Compass Mode provides a simple, GPS-free emergency escape option. While it has limitations, it can be invaluable in environments where traditional RTL won't work. Always plan your escape route before flying and test the system in a safe environment first.