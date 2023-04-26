import socket
import threading
from _thread import start_new_thread

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

    def send(self, msg,c):
        msg = msg.encode()
        bytes_sent = c.sendall(msg)
        return bytes_sent

    def wait(self):
        # wait for the agents to connect
        self.s.listen(self.agent_number)

    def mem_data(self,msg):
        # msg is a string of the form "DATA,id,px,py,cx,cy"
        for i in range(self.agent_number):
            if self.data_memory[i][0] == 0 or self.data_memory[i][0] == msg[0]: # new agent or known one
                self.data_memory[i] = msg
                break

    def other_data(self,id):
        # id of the robot int
        msg = "DATAO"
        for i in range(self.agent_number):
            if self.data_memory[i][0] != id:
                msg += "," + str(int(self.data_memory[i][0])) + "," + str(self.data_memory[i][1]) +\
                       "," + str(self.data_memory[i][2]) + "," + str(self.data_memory[i][3]) +\
                       "," + str(self.data_memory[i][4])
        return msg
    def threaded(self, c):
        while True:
            self.send("UP",c)  # ask the agent to give their data

            msg = c.recv(1024)
            msg = msg.decode()

            if msg:
                print("Received:", msg)  #DATA,id, px, py, cx, cy
                msg = msg.split(",")
                msg = [float(msg[x]) for x in range(1, len(msg))]
                self.mem_data(msg)  # store the data in the memory

                # send other data to the agent
                msg_out = self.other_data(int(msg[0]))
                print("msg_out is",msg_out)
                self.send(msg_out,c)
        c.close()


    def run(self):
        self.wait()
        while True:
            # establish connection with client
            c, addr = self.s.accept()

            # lock acquired by client
            print('Connected to :', addr[0], ':', addr[1])

            # Start a new thread and return its identifier
            start_new_thread(self.threaded, (c,))
        self.s.close()

if __name__ == "__main__":
    server = Server(agent_number=3)
    server.start()