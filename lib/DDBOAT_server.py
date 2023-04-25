import socket
import threading

class Server(threading.Thread):
    def __init__(self, agent_number = 1, host="172.20.26.193", port=33000):
        # agent_number is the number of agents that will connect to the server
        # host is the ip address of the server (my pc)
        # port is the port number of the server

        threading.Thread.__init__(self)
        self.host = host
        self.port = port

        self.agent_number = agent_number
        self.data_memory = [[0.,0.,0.,0.,0.] for i in range(agent_number)] # id px py cx cy

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.host, self.port))

    def send(self, msg):
        msg = msg.encode()
        bytes_sent = self.conn.sendall(msg)
        return bytes_sent

    def wait(self):
        # wait for the agents to connect
        self.s.listen(self.agent_number)
        self.conn, self.addr = self.s.accept()
        self.connected = True
        print("Connected by", self.addr)

    def recv(self, size):
        msg = self.conn.recv(size)
        msg = msg.decode()
        return msg

    def mem_data(self,msg):
        # msg is a string of the form "DATA,id,px,py,cx,cy"
        msg = msg.split(",")
        msg = [float(msg[x]) for x in range(1,len(msg))]
        for i in range(self.agent_number):
            if self.data_memory[i][0] == 0 or self.data_memory[i][0] == msg[0]: # new agent or known one
                self.data_memory[i] = msg
                break

    def run(self):
        self.wait()
        while self.connected:
            self.send("UP") # ask the agent to give their data

            msg = self.recv(1024)

            if msg:
                print("Received:", msg) # id, px, py, cx, cy
                self.mem_data(msg) # store the data in the memory

                # send data to agents
                self.send(msg)
        self.conn.close()

if __name__ == "__main__":
    server = Server()
    server.start()
    server.join()