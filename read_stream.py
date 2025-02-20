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

def receive_image(client_socket):
    # Receive the image size (first 8 bytes)
    image_size = int.from_bytes(client_socket.recv(8), byteorder='little')

    # Receive the image data in chunks
    image_data = b''
    while len(image_data) < image_size:
        chunk = client_socket.recv(image_size - len(image_data))
        if not chunk:
            break
        image_data += chunk
    print(image_size)
    print("Image received")

    # Convert the received image data (binary) into a numpy array
    nparr = np.frombuffer(image_data, np.uint8)
    import ipdb 
    ipdb.set_trace()

    # Decode the numpy array into an image
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    # TODO: DECODE THIS 

    if img is not None:
        # Show the image using OpenCV (you can also save it if needed)
        cv2.imshow("Received Image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # Optionally save the image
        cv2.imwrite("received_image.png", img)
        print("Image saved as 'received_image.png'")
    else:
        print("Failed to decode image.")



if __name__ == "__main__":
    server_socket, client_socket, client_address =  set_up_receiver('127.0.0.1', 8080)
    while True:
        receive_image(client_socket)  # Listen on localhost and port 8080

    client_socket.close()
    server_socket.close()