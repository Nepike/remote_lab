#
# YARP13 Sensors
# 28.08.2023, 21.02.2024, 13.07.2024
# LP 21.04.2025
#

time tm

#
# All adc, raw values (0..1023)
#
int16[10] adc

#
int16[8] data

# encoders
int32 enc_left
int32 enc_right

#
# Compass magnetometr/gyroscope angles, degrees (yaw, pitch, roll)
#
int16 compass

# Pitch and Roll angles (degrees)
int16 cpitch
int16 croll

# Accumulator voltage, V
float32 acc_voltage

############################################################
#
# Range Finders, sm
#
############################################################
#
#
# *** FWD Range Finders
#
# Center
int16 rf_center

# Fwd Left
int16 rf_left

# Fwd Right
int16 rf_right

# *** SIDE Range Finders
int16 rf_side_left_fwd
int16 rf_side_right_fwd
int16 rf_side_left_bck
int16 rf_side_right_bck

# Back
int16 rf_bck_center

############################################################

#
# Robot`s status
#
int16 status

#
# Ultrasonic sensors, sm
#
# sic int16[8] uss

#
# Switches status
# sic int16[8] switches
int8[8] switches

#
# IR-data
int32[4] irdata

