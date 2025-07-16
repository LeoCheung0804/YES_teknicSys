# ArUco Marker Detector

## Requirements

```
pip install -r requirements.txt
```

## Run

```
python ArUcoDetector.py
```

## Notes for YES

- The camera is mounted on the end effector, there is a Raspberry Pi 4B for processing.
- To connect to the Raspberry Pi, first connect the Raspberry Pi to the same WiFi network as the laptop.
- Then, use the following command:

```
ssh kazuki@<rpi-ip-address>
```

- The password is `<go-to-ask-galad>`
- The directory on RPi is `/home/kazuki/galad_ws/aruco-board-decector-for-YES`
