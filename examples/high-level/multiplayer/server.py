import socket
import threading
from queue import Queue

que = Queue()
history = []


class PlayerThread(threading.Thread):
    def __init__(self, client_address, client_socket, que):
        threading.Thread.__init__(self)
        self._socket = client_socket
        self._que = que

    def run(self):
        while True:
            data = self._socket.recv(63)
            if data:
                self._que.put(data)


class Server:
    def __init__(self):
        super().__init__()
        self.socket = socket.socket()
        self.socket.bind(("0.0.0.0", 6000))
        self.socket.listen(5)
        self.clients = []

    def start(self):
        print("listening")
        while True:
            csock, caddr = self.socket.accept()
            newthread = PlayerThread(caddr, csock, que)
            newthread.start()
            for item in history:
                csock.send(item)
            self.clients.append(csock)


class Consumer(threading.Thread):
    def __init__(self, que, clients):
        threading.Thread.__init__(self)
        self._que = que
        self._clients = clients

    def run(self):
        while True:
            data = self._que.get()
            history.append(data)
            for client in self._clients:
                client.send(data)


server = Server()
consumer = Consumer(que, server.clients)
consumer.start()
server.start()
