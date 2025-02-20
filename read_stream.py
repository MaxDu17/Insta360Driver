import socket
import numpy as np
import cv2

def set_up_receiver(server_ip, server_port):
    # Set up the socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    print(f"Listening for connections on {server_ip}:{server_port}...")
    server_socket.listen(1)

    # Accept a connection from the C++ client
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address}")
    return server_socket, client_socket, client_address


# def receive_matrix(client_socket):
#     buffer_size = int.from_bytes(client_socket.recv(4), byteorder='little')

#     # Receive the actual image data
#     buffer = b""
#     while len(buffer) < buffer_size:
#         buffer += client_socket.recv(buffer_size - len(buffer))

#     # Convert the byte buffer to a numpy array and decode it as an image
#     np_arr = np.frombuffer(buffer, dtype=np.uint8)
#     img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#     import ipdb 
#     ipdb.set_trace()

#     return img


def receive_image(client_socket):
    buffer_size = int.from_bytes(client_socket.recv(4), byteorder='little')

    # Receive the actual image data
    buffer = b""
    while len(buffer) < buffer_size:
        buffer += client_socket.recv(buffer_size - len(buffer))

    # Convert the byte buffer to a numpy array and decode it as an image
    np_arr = np.frombuffer(buffer, dtype=np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    import ipdb 
    ipdb.set_trace()

    if img is not None:
        # Show the image using OpenCV (you can also save it if needed)
        cv2.imshow("Received Image", img)
        cv2.waitKey(1)        
        # Optionally save the image
        # cv2.imwrite("received_image.png", img)
        # print("Image saved as 'received_image.png'")
    else:
        print("Failed to decode image.")



if __name__ == "__main__":
    server_socket, client_socket, client_address =  set_up_receiver('127.0.0.1', 8080)
    while True:
        receive_image(client_socket)  # Listen on localhost and port 8080

    client_socket.close()
    server_socket.close()