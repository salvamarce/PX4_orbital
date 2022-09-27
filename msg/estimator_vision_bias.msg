uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[3] bias                 # estimated barometric altitude bias (m)
float32[3] bias_var             # estimated barometric altitude bias variance (m^2)

float32[3] innov                # innovation of the last measurement fusion (m)
float32[3] innov_var            # innovation variance of the last measurement fusion (m^2)
float32[3] innov_test_ratio     # normalized innovation squared test ratio
