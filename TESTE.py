import dlib
import cv2
import numpy as np

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

cap = cv2.VideoCapture(0)

# Função para converter os pontos em uma estrutura utilizável
def face_landmarks_to_np(landmarks, dtype="int"):
    coords = np.zeros((68, 2), dtype=dtype)
    for i in range(0, 68):
        coords[i] = (landmarks.part(i).x, landmarks.part(i).y)
    return coords

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    for face in faces:
        landmarks = predictor(gray, face)
        landmarks_np = face_landmarks_to_np(landmarks)

        # Desenhar contornos nas características faciais
        # Definir os índices dos pontos que compõem cada característica
        indices_rosto = list(range(0, 17))
        indices_sobrancelha_esq = list(range(17, 22))
        indices_sobrancelha_dir = list(range(22, 27))
        indices_nariz = list(range(27, 36))
        indices_olho_esq = list(range(36, 42))
        indices_olho_dir = list(range(42, 48))
        indices_boca = list(range(48, 68))

        # Função para desenhar linhas entre os pontos
        def desenhar_contorno(indices, closed=False):
            pts = landmarks_np[indices]
            if closed:
                pts = np.append(pts, [pts[0]], axis=0)
            for i in range(len(pts)-1):
                pt1 = tuple(pts[i])
                pt2 = tuple(pts[i+1])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # Desenhar cada característica
        desenhar_contorno(indices_rosto)
        desenhar_contorno(indices_sobrancelha_esq)
        desenhar_contorno(indices_sobrancelha_dir)
        desenhar_contorno(indices_nariz)
        desenhar_contorno(indices_olho_esq, closed=True)
        desenhar_contorno(indices_olho_dir, closed=True)
        desenhar_contorno(indices_boca, closed=True)

    cv2.imshow("Landmarks", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()