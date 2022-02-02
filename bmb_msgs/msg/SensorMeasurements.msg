uint8 SIZE = 21
uint8 PRESSURE = 0
uint8 RAIL_ANGLE = 1
uint8 RAIL_WIDTH = 2
uint8 RAIL_x = 3
uint8 RAIL_y = 4
uint8 RAIL_FOUND = 5
uint8 PIXEL_vx = 6
uint8 PIXEL_vy = 7
uint8 IMU_q0 = 8
uint8 IMU_q1 = 9
uint8 IMU_q2 = 10
uint8 IMU_q3 = 11
uint8 IMU_wx = 12
uint8 IMU_wy = 13
uint8 IMU_wz = 14
uint8 IMU_ax = 15
uint8 IMU_ay = 16
uint8 IMU_az = 17
uint8 LAT = 18
uint8 LONG =  19
uint8 ALT = 20

sensor_msgs/FluidPressure pressure_reading
RailDetection rail_detection
OpticalFlowReading optical_flow_reading
sensor_msgs/Imu imu_reading
sensor_msgs/NavSatFix gps_reading
