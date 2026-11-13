# The filter state after one input (R1.11). One per input, in input order.
uint64 seq                    # input number, from 0
float64 t                     # filter time: last IMU timestamp, s
float64[3] position           # local NED, m
float64[3] velocity           # NED, m/s
float64[4] orientation        # quaternion w, x, y, z (body to NED)
float64[3] accel_bias         # m/s^2
float64[3] gyro_bias          # rad/s
float64 baro_bias             # m
float64[3] mag_bias           # uT
float64[19] p_diag            # covariance diagonal, in the error-state order (ARCHITECTURE.md)
float64 p_ne                  # north-east position covariance, m^2
uint8 flags                   # FLAG_* below
uint8 FLAG_ALIGNED = 1
uint8 FLAG_HEADING = 2
uint8 FLAG_DATUM = 4
uint8 FLAG_AIRBORNE = 8
