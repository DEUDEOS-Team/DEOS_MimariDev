# Sabit boyutlu LiDAR nokta bulutu mesajı (en fazla 65536 nokta x 16 byte = 1 MB, POD).
#
# Geçerli nokta sayısı `num_points` alanında tutulur (num_points <= 65536).
# Her nokta [x, y, z, intensity] şeklinde 4 adet float32 (toplam 262144 float32).
# has_fixed_size<FixedPointCloud64k>::value == true -> iceoryx loaned zero-copy destekler.

uint64 stamp_ns
uint32 seq
uint32 num_points
uint8[64] frame_id
float32[262144] points_xyzi
