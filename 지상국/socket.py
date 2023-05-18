import socket

HOST = '0.0.0.0'  # 모든 인터페이스에서 접속 가능하도록 설정
PORT = 12345  # 사용할 포트 번호

# 소켓 생성 및 바인딩
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

# 클라이언트의 연결 대기
print('Waiting for a connection...')
conn, addr = s.accept()
print('Connected by', addr)

# 데이터 수신 및 출력
while True:
    data = conn.recv(1024)  # 최대 1024바이트의 데이터 수신
    if not data:
        break
    print('Received from', addr, ':', data.decode())

# 소켓 닫기
conn.close()