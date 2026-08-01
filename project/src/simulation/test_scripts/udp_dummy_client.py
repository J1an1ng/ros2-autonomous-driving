import socket
import struct


def decode(data):
    if len(data) != 16:
        return None
    return struct.unpack('ffff', data)


port = 6657
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', port))

print(f'Listening on UDP port {port}...')

while True:
    data, address = sock.recvfrom(1024)
    values = decode(data=data)
    if values is not None:
        print(f'Address: {address}')
        print(
            f'Control values: throttle={values[0]:.3f}, '
            f'steering={values[1]:.3f}, brake={values[2]:.3f}, '
            f'dummy={values[3]:.3f}'
        )
