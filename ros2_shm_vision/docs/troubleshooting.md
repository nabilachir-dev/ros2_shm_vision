# Troubleshooting - Zero Copy 1080p

## Errori Comuni

### cv_bridge not found
```bash
sudo apt install ros-humble-cv-bridge
```
Oppure rimuovi cv_bridge da package.xml se non lo usi.

### OpenCV not found
```bash
sudo apt install libopencv-dev
```

### Nessun video trovato
Metti un file .mp4 in `~/ros2_ws/src/ros2_shm_vision/video/`

### Zero-copy non attivo
Verifica variabili ambiente:
```bash
echo $RMW_FASTRTPS_USE_QOS_FROM_XML  # deve essere 1
echo $FASTRTPS_DEFAULT_PROFILES_FILE # deve puntare a fastdds_setup.xml
```

---

## Build non aggiornata

### Problema
Dopo aver modificato il codice, le modifiche non hanno effetto.

### Causa
Il comando `colcon build` deve essere eseguito **dalla cartella del workspace**, non da altre posizioni.

### Soluzione
```bash
cd ~/ros2_ws                  # VAI nel workspace!
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

### Build pulita (se necessario)
```bash
cd ~/ros2_ws
rm -rf build/ros2_shm_vision install/ros2_shm_vision
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

---

## OpenCV imshow non funziona

### Problema
Il subscriber riceve i frame (FPS: 30) ma la finestra OpenCV non appare.

### Possibili cause

1. **OpenCV compilato senza GUI (highgui)**
   ```bash
   # Verifica se highgui e disponibile
   pkg-config --libs opencv4 | grep highgui
   ```

2. **Mancano dipendenze GTK**
   ```bash
   sudo apt install libgtk2.0-dev libgtk-3-dev
   # Ricompila OpenCV se necessario
   ```

3. **Problema DISPLAY**
   ```bash
   export DISPLAY=:0
   echo $DISPLAY  # deve mostrare :0 o :1
   ```

4. **Wayland invece di X11**
   Su sistemi con Wayland, cv::imshow potrebbe non funzionare.
   Prova a passare a X11 al login.

### Workaround: Salva immagine su disco
```cpp
cv::imwrite("/home/user/test_frame.png", frame);
```
