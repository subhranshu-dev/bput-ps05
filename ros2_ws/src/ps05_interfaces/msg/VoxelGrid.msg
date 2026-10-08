# Dense 3D occupancy grid, zlib-compressed. Cell values: 0 = FREE, 1 = UNKNOWN, 2 = OCCUPIED.
# Index = ix + nx * (iy + ny * iz)
std_msgs/Header header
float64 resolution
geometry_msgs/Point origin    # world coordinate of the min corner of cell (0,0,0)
uint32 nx
uint32 ny
uint32 nz
uint8[] data                  # zlib.compress(uint8 array of length nx*ny*nz)
uint32 occupied_count
uint32 free_count
