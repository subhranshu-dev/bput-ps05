# Ground tracking station measurement (PS05 report Sec. 8): spherical state of the UAV
# relative to the station origin. Angles in radians, range in metres.
std_msgs/Header header        # stamp = measurement (sample) time; frame_id = tracking station frame
float64 range                 # r
float64 azimuth               # theta, from station +X toward +Y
float64 elevation             # phi, above the station horizontal plane
float64 sigma_range
float64 sigma_azimuth
float64 sigma_elevation
geometry_msgs/Vector3 heading_vector   # Vh: unit vector toward the corridor centreline (0 if unknown)
