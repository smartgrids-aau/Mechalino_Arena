import cv2
import cv2.aruco as aruco

# Kamera initialisieren
cap = cv2.VideoCapture(0)  # Wähle die richtige Kamera-ID, falls nötig

# ARuco-Dictionary für die Marker-Erkennung
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
parameters = aruco.DetectorParameters()

# ARuco-Detektor initialisieren
aruco_detector = aruco.ArucoDetector(dictionary, parameters)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    if ids is not None:
        print(f"Erkannte Marker-IDs: {ids}")

    # ARuco-Marker anzeigen
    aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('ARuco Detector', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
