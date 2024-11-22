import dlib
import cv2
import numpy as np
import time

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")


# Função de cálculo da distância euclidiana
def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


# Função de suavização de coordenadas usando médias móveis
def moving_average(points, prev_points, alpha=0.5):
    return [(int(alpha * p1[0] + (1 - alpha) * p2[0]), int(alpha * p1[1] + (1 - alpha) * p2[1]))
            for p1, p2 in zip(points, prev_points)]


# Inicializar contadores para detecções acumuladas e suavização
detec_au1 = 0
detec_au2 = 0
detec_au12 = 0
detec_au4 = 0

# Armazenar posições de landmarks para suavização
prev_landmarks = None

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    for face in faces:
        landmarks = predictor(gray, face)

        # Extraindo as coordenadas das landmarks relevantes
        landmarks_points = [(landmarks.part(n).x, landmarks.part(n).y) for n in range(68)]

        # Suavizar os pontos de landmarks usando médias móveis
        if prev_landmarks is not None:
            landmarks_points = moving_average(landmarks_points, prev_landmarks, alpha=0.7)
        prev_landmarks = landmarks_points

        # Coordenadas relevantes para AUs
        sobrancelha_esq_ext = landmarks_points[17]
        sobrancelha_dir_ext = landmarks_points[26]
        olho_esq_canto_ext = landmarks_points[39]
        olho_dir_canto_ext = landmarks_points[45]
        canto_boca_esq = landmarks_points[48]
        canto_boca_dir = landmarks_points[54]

        # AU1 e AU2: Elevação das sobrancelhas (interna e externa)
        distancia_sobrancelha_esq = euclidean_distance(sobrancelha_esq_ext, olho_esq_canto_ext)
        distancia_sobrancelha_dir = euclidean_distance(sobrancelha_dir_ext, olho_dir_canto_ext)

        if distancia_sobrancelha_esq > 20:  # Parâmetro ajustável
            detec_au1 += 1
        else:
            detec_au1 = max(0, detec_au1 - 1)  # Diminuir lentamente ao invés de resetar

        if distancia_sobrancelha_dir > 20:
            detec_au2 += 1
        else:
            detec_au2 = max(0, detec_au2 - 1)  # Diminuir lentamente ao invés de resetar

        # AU12: Sorriso
        distancia_boca = euclidean_distance(canto_boca_esq, canto_boca_dir)
        if distancia_boca > 60:  # Parâmetro ajustável
            detec_au12 += 1
        else:
            detec_au12 = max(0, detec_au12 - 1)  # Diminuir lentamente

        # AU4: Deprimido das sobrancelhas
        if sobrancelha_esq_ext[1] > olho_esq_canto_ext[1]:
            detec_au4 += 1
        else:
            detec_au4 = max(0, detec_au4 - 1)  # Diminuir lentamente

        # Mostrar detecções após um número de frames consecutivos
        if detec_au1 > 5:  # Se AU1 for detectada por 5 frames consecutivos
            print("AU1: Elevação da sobrancelha interna detectada!")
            detec_au1 = 0  # Resetar após exibir a detecção

        if detec_au2 > 5:
            print("AU2: Elevação da sobrancelha externa detectada!")
            detec_au2 = 0

        if detec_au12 > 5:
            print("AU12: Sorriso detectado!")
            detec_au12 = 0

        if detec_au4 > 5:
            print("AU4: Deprimido das sobrancelhas detectado!")
            detec_au4 = 0

        # Desenhar as landmarks suavizadas na imagem
        for (x, y) in landmarks_points:
            cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

    cv2.imshow("Landmarks and AUs", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Reduzir o atraso para 0.1 segundos para melhorar a resposta
    time.sleep(0.05)

cap.release()
cv2.destroyAllWindows()