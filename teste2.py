import dlib
import cv2

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    for face in faces:
        landmarks = predictor(gray, face)

        # Extraindo as características das diferentes regiões faciais
        face_features = {
            "sobrancelha_esquerda": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(17, 22)],
            "sobrancelha_direita": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(22, 27)],
            "olho_esquerdo": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(36, 42)],
            "olho_direito": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(42, 48)],
            "nariz": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(27, 36)],
            "boca_externa": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(48, 61)],
            "boca_interna": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(61, 68)],
            "contorno_rosto": [(landmarks.part(n).x, landmarks.part(n).y) for n in range(0, 17)]
        }

        # Desenhar as landmarks na imagem
        for n in range(0, 68):
            x = landmarks.part(n).x
            y = landmarks.part(n).y
            cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

        # Exibir as coordenadas das regiões faciais no terminal
        print("Características Faciais Extraídas:", face_features)

    cv2.imshow("Landmarks", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()