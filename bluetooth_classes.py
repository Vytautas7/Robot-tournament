import threading

class ServerSendThread(threading.Thread):

    def __init__(self, name, port, f):
        threading.Thread.__init__(self)
        self.name = name
        self.port = port
        self.f = f

    def run(self):
        print("Starting sending as server: " + self.name)
        self.f(self.name, self.port)
        print("Stopping sending as server: " + self.name)

class ServerReceiveThread(threading.Thread):

    def __init__(self, name, port, f):
        threading.Thread.__init__(self)
        self.name = name
        self.port = port
        self.f = f

    def run(self):
        print("Starting receiving as server: " + self.name)
        self.f(self.name, self.port)
        print("Stopping receiving as server: " + self.name)


class ClientSendThread(threading.Thread):

    def __init__(self, name, port, address, f, lock):
        threading.Thread.__init__(self)
        self.name = name
        self.port = port
        self.address = address
        self.lock = threading.Lock()
        self.f = f

    def run(self):
        print("Starting sending as client" + self.name)
        self.f(self.name, self.port, self.address, self.lock)
        print("Stopping sending as client" + self.name)

class ClientReceiveThread(threading.Thread):

    def __init__(self, name, port, address, f):
        threading.Thread.__init__(self)
        self.name = name
        self.port = port
        self.address = address
        self.f = f

    def run(self):
        print("Starting receiving as client" + self.name)
        self.f(self.name, self.port, self.address)
        print("Stopping receiving as client" + self.name)