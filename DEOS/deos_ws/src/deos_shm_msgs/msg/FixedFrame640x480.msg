# Sabit boyutlu 640x480 BGR8 görüntü mesajı (~921.6 KB, POD / TriviallyCopyable).
#
# Neden sensor_msgs/Image değil?
#   sensor_msgs/Image içinde hem std_msgs/Header (string frame_id) hem de uint8[] data
#   dinamik boyutludur -> rosidl_generator_traits::has_fixed_size<Image>::value == false.
#   Bu nedenle rmw_cyclonedds_cpp + iceoryx bu tipte borrow_loaned_message() ile
#   gerçek zero-copy yapamaz (yine serileştirme yapar).
#
# Bu mesajda tüm alanlar sabit boyutlu dizidir:
#   has_fixed_size<FixedFrame640x480>::value == true -> iceoryx /dev/shm zero-copy aktif!

uint64 stamp_ns
uint32 seq
uint16 width 640
uint16 height 480
uint8 channels 3
uint8 encoding_bgr8 1
uint8[64] frame_id
uint8[921600] data
