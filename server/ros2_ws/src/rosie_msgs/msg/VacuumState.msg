# Vacuum-specific state beyond what sensor_msgs/BatteryState covers
string state            # idle, cleaning, paused, docked, returning, error
string ui_state         # raw Neato UI state string
string robot_state      # raw Neato robot state string
string error            # current error or "none"
string alert            # current alert or "none"
float32 battery_percent
float32 battery_voltage
float32 battery_temp
bool charging
bool docked
