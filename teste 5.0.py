import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk  # Para lidar com imagens no Tkinter
from threading import Thread

# Função de cálculo da distância euclidiana
def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Função de suavização de coordenadas usando médias móveis
def moving_average(points, prev_points, alpha=0.5):
    return [(int(alpha * p1[0] + (1 - alpha) * p2[0]), int(alpha * p1[1] + (1 - alpha) * p2[1]))
            for p1, p2 in zip(points, prev_points)]

# Função para atualizar a seta visual com o comando detectado
def update_arrows(up, down, left, right, command):
    if command == "Comando 1: Andar para frente":
        up.config(image=up_on_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)
    elif command == "Comando 2: Andar para trás":
        up.config(image=up_off_img)
        down.config(image=down_on_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)
    elif command == "Comando 3: Virar à esquerda":
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_on_img)
        right.config(image=right_off_img)
    elif command == "Comando 4: Virar à direita":
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_on_img)
    else:
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)

# Inicializar contadores para detecções acumuladas e suavização
detec_au1 = 0
detec_au2 = 0
detec_au12 = 0
detec_au4 = 0
prev_landmarks = None

# Inicializando Tkinter
root = tk.Tk()
root.title("Detecção de AUs e Comandos com Setas")
root.geometry("400x400")

# Carregando as imagens das setas
up_off_img = ImageTk.PhotoImage(Image.open("up_off.png"))
up_on_img = ImageTk.PhotoImage(Image.open("up_on.png"))
down_off_img = ImageTk.PhotoImage(Image.open("down_off.png"))
down_on_img = ImageTk.PhotoImage(Image.open("down_on.jpg"))
left_off_img = ImageTk.PhotoImage(Image.open("left_off.jpg"))
left_on_img = ImageTk.PhotoImage(Image.open("left_on.jpg"))
right_off_img = ImageTk.PhotoImage(Image.open("right_off.jpg"))
right_on_img = ImageTk.PhotoImage(Image.open("right_on.png"))

# Criando os widgets das setas
up_arrow = tk.Label(root, image=up_off_img)
down_arrow = tk.Label(root, image=down_off_img)
left_arrow = tk.Label(root, image=left_off_img)
right_arrow = tk.Label(root, image=right_off_img)

# Posicionando as setas na interface
up_arrow.pack(pady=10)
left_arrow.pack(side="left", padx=50)
right_arrow.pack(side="right", padx=50)
down_arrow.pack(pady=10)

# Label para exibir o comando atual
command_label = tk.Label(root, text="Aguardando detecção...")
command_label.pack(pady=20)

# Função que rodará a detecção de AUs
def detect_aus():
    global detec_au1, detec_au2, detec_au12, detec_au4, prev_landmarks

    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

    cap = cv2.VideoCapture(0)

    # Ajuste de parâmetros da câmera
    cap.set(cv2.CAP_PROP_GAIN, 0)  # Reduzir ganho para evitar ruído
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)  # Ajuste de brilho (valor ajustável)
    cap.set(cv2.CAP_PROP_CONTRAST, 50)  # Ajuste de contraste (valor ajustável)

    while True:
        ret, frame = cap.read()

        # Aplicar filtro Gaussiano para reduzir o ruído (ajustável)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

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

            if distancia_sobrancelha_esq > 22:
                detec_au1 += 1
            else:
                detec_au1 = max(0, detec_au1 - 1)

            if distancia_sobrancelha_dir > 22:
                detec_au2 += 1
            else:
                detec_au2 = max(0, detec_au2 - 1)

            # AU12: Sorriso
            distancia_boca = euclidean_distance(canto_boca_esq, canto_boca_dir)
            if distancia_boca > 65:
                detec_au12 += 1
            else:
                detec_au12 = max(0, detec_au12 - 1)

            # AU4: Deprimido das sobrancelhas
            if sobrancelha_esq_ext[1] > olho_esq_canto_ext[1]:
                detec_au4 += 1
            else:
                detec_au4 = max(0, detec_au4 - 1)

            # Lógica para junção de AUs e definição de comandos
            comando = "Aguardando detecção..."

            # Associando combinações de AUs aos comandos desejados
            if detec_au1 > 5 and detec_au12 > 5:
                comando = "Comando 1: Andar para frente"
            elif detec_au4 > 5:
                comando = "Comando 2: Andar para trás"
            elif detec_au2 > 5:
                comando = "Comando 3: Virar à esquerda"
            elif detec_au1 > 5 and detec_au4 > 5:
                comando = "Comando 4: Virar à direita"

            # Atualizar a interface gráfica com o comando detectado
            command_label.config(text=comando)
            update_arrows(up_arrow, down_arrow, left_arrow, right_arrow, comando)

        # Reduzir o atraso para 0.1 segundos para melhorar a resposta
        time.sleep(0.1)

    cap.release()

# Criar uma thread para rodar a detecção sem bloquear a interface gráfica
detect_thread = Thread(target=detect_aus)
detect_thread.daemon = True
detect_thread.start()

# Rodar o loop da interface
root.mainloop()
