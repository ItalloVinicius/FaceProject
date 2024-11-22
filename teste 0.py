import socket
import time

server_address = ('127.0.0.1', 65432)

# Cria um socket TCP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    time.sleep(1)  # Espera um segundo antes de tentar conectar
    client_socket.connect(server_address)
    print("Conectado ao servidor.")

    # Envia um comando para o servidor
    command = 'Comando 1: Andar para frente'  # Substitua pelo comando que deseja enviar
    client_socket.sendall(command.encode())
    print("Comando enviado.")

    # Espera por uma resposta do servidor
    response = client_socket.recv(1024)  # Recebe até 1024 bytes
    print(f"Resposta do servidor: {response.decode()}")

except Exception as e:
    print(f"Erro ao conectar ou enviar: {e}")
finally:
    client_socket.close()
