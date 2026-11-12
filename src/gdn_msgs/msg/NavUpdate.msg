# One measurement or ZUPT and what the filter did with it (R1.11). Skipped inputs are
# recorded too, with the reason. Values match gdn::Eskf::Sensor and Eskf::Reason.
uint64 seq                    # the input that caused it
float64 t                     # filter time, s
uint8 sensor
uint8 SENSOR_TRN = 1
uint8 SENSOR_AGL = 2
uint8 SENSOR_BARO = 3
uint8 SENSOR_MAG3 = 4         # 3-axis magnetometer
uint8 SENSOR_MAG_HEADING = 5  # magnetometer as heading only
uint8 SENSOR_ZUPT = 6
uint8 SENSOR_GNSS = 7
uint8 reason
uint8 APPLIED = 0
uint8 NOT_ALIGNED = 1
uint8 NO_HEADING = 2
uint8 NO_DATUM = 3
uint8 OFF_MAP = 4
uint8 FLAT = 5                # terrain flatter than slope_min (TRN)
uint8 WEAK_FIELD = 6          # horizontal field under 1 uT
uint8 INNOV_GATE = 7
uint8 NUMERIC = 8             # innovation variance not positive, or NaN
uint8 BAD_INPUT = 9
uint8 INIT = 10               # used to initialise (heading, altitude datum, baro bias)
uint8 NOT_USED = 11           # input this configuration doesn't use
float64[] innovation          # empty if none was formed
float64[] innovation_var      # diagonal of its predicted covariance S
float64 nis                   # innovation' S^-1 innovation (NaN if none)
bool gain_limited             # a clamp_* limit scaled the correction
