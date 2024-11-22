import dlib
import cv2
import numpy as np

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")


def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    for face in faces:
        landmarks = predictor(gray, face)

        # Extraindo as coordenadas
        sobrancelha_esq_int = (landmarks.part(19).x, landmarks.part(19).y)
        sobrancelha_dir_int = (landmarks.part(24).x, landmarks.part(24).y)
        sobrancelha_esq_ext = (landmarks.part(17).x, landmarks.part(17).y)
        sobrancelha_dir_ext = (landmarks.part(26).x, landmarks.part(26).y)
        olho_esq_canto_int = (landmarks.part(36).x, landmarks.part(36).y)
        olho_esq_canto_ext = (landmarks.part(39).x, landmarks.part(39).y)
        canto_boca_esq = (landmarks.part(48).x, landmarks.part(48).y)
        canto_boca_dir = (landmarks.part(54).x, landmarks.part(54).y)

        # Calculando distâncias para monitorar as AUs

        # AU1 e AU2: Elevação das sobrancelhas (interna e externa)
        distancia_sobrancelha_esq = euclidean_distance(sobrancelha_esq_ext, olho_esq_canto_ext)
        distancia_sobrancelha_dir = euclidean_distance(sobrancelha_dir_ext, olho_esq_canto_int)

        if distancia_sobrancelha_esq > 20:  # Parâmetro arbitrário
            print("AU1: Elevação da sobrancelha interna detectada!")

        if distancia_sobrancelha_dir > 20:
            print("AU2: Elevação da sobrancelha externa detectada!")

        # AU12: Sorriso
        distancia_boca = euclidean_distance(canto_boca_esq, canto_boca_dir)
        if distancia_boca > 60:  # Parâmetro arbitrário
            print("AU12: Sorriso detectado!")

        # AU4: Deprimido das sobrancelhas
        if sobrancelha_esq_int[1] > olho_esq_canto_int[1]:
            print("AU4: Deprimido das sobrancelhas detectado!")

        # Desenhar as landmarks na imagem
        for n in range(0, 68):
            x = landmarks.part(n).x
            y = landmarks.part(n).y
            cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

    cv2.imshow("Landmarks and AUs", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()