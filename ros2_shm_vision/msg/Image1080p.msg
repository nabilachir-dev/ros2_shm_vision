# Messaggio personalizzato per trasferimento immagini 1080p zero-copy
#
# Immagine 1920x1080 RGB (3 canali) in formato POD puro.
# Array a lunghezza FISSA per supporto zero-copy.
#
# Dimensione totale:
#   - timestamp_sec: 4 byte
#   - timestamp_nanosec: 4 byte  
#   - frame_id: 4 byte
#   - width: 4 byte
#   - height: 4 byte
#   - data: 1920 × 1080 × 3 = 6.220.800 byte
#   - TOTALE: ~6 MB per messaggio

# Timestamp come interi POD (nessun Header per mantenerlo POD puro)
int32 timestamp_sec
uint32 timestamp_nanosec

# Identificatore frame
uint32 frame_id

# Dimensioni immagine (fissi a 1920x1080)
uint32 width
uint32 height

# Dati pixel RGB: 1920 × 1080 × 3 = 6.220.800 byte
uint8[6220800] data
