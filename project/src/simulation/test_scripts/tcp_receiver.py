import socket
import struct

from message_decoders import (
    parse_depthcamera,
    parse_rgbcamera,
    parse_truestate,
    read_exact,
)


UNITY_STATE = 0
UNITY_CAMERA = 1
UNITY_IMU = 2
UNITY_DEPTH = 3
UNITY_FISHEYE = 4
UNITY_DETECTIONS = 5
MESSAGE_TYPE_COUNT = 6

HOST = '0.0.0.0'
PORT = 9998


def read_string(sock):
    characters = []
    while True:
        character = sock.recv(1)
        if character == b'\x00':
            break
        characters.append(character)
    return b''.join(characters).decode('ascii')


def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f'[INFO] Waiting for Unity connection on port {PORT}...')
    connection, address = server_socket.accept()
    print(f'[INFO] Connected by {address}')
    try:
        while True:
            magic = struct.unpack('<I', read_exact(connection, 4))[0]
            if magic != 0xDEADC0DE:
                print(f'[ERROR] Magic mismatch: {hex(magic)}')
                continue

            type_id = struct.unpack('<I', read_exact(connection, 4))[0]
            ticks = struct.unpack('<Q', read_exact(connection, 8))[0]
            name = read_string(connection)
            print(
                f'\n[INFO] Got image from {name!r} | '
                f'Type: {type_id} | Ticks: {ticks}'
            )

            if type_id == UNITY_CAMERA:
                parse_rgbcamera(
                    conn=connection,
                    name=name,
                    timestamp=ticks,
                )
            elif type_id == UNITY_DEPTH:
                parse_depthcamera(
                    conn=connection,
                    name=name,
                    timestamp=ticks,
                )
            elif type_id == UNITY_STATE:
                parse_truestate(
                    conn=connection,
                    name=name,
                    timestamp=ticks,
                    visualize=False,
                )

    except Exception as ex:
        print(f'[ERROR] {ex}')
    finally:
        connection.close()
        server_socket.close()


if __name__ == '__main__':
    main()
