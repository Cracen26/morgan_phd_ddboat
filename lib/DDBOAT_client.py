import socket
import threading
import time

class Client(threading.Thread):
    def __init__(self,my_data, host="172.20.26.193", port=33000):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False
        self.my_data = my_data # [id,px,py,cx,cy]
        self.other_data = []

    def send(self,msg):
        msg = msg.encode()
        bytes_sent = self.s.sendall(msg)
        return bytes_sent

    def recv(self,buff):
        msg = self.s.recv(buff)
        return msg.decode()
    def connect(self):
        # connect to the server
        while not self.connected:
            try:
                self.s.connect((self.host, self.port))
                self.connected = True
            except:
                print("Connection failed, retrying...")
                time.sleep(1)
    def run(self):
        self.connect()
        while self.connected:
            answer = self.recv(1024)
            if answer:
                if answer == "UP": # send my data
                    msg = "DATA,"+ str(int(self.my_data[0])) + "," + str(self.my_data[1]) +\
                          "," + str(self.my_data[2]) + "," + str(self.my_data[3]) +\
                          "," + str(self.my_data[4])
                    self.send(msg)
                else: # store the data of the other agents
                    answer = answer.split(",")
                    if int(answer[1]) != 0: # if it is the data of another agent
                        answer = [float(answer[x]) for x in range(1,len(answer))]
                        self.other_data = [] # reset
                        for i in range(len(answer)//5):
                            if int(answer[i*5]) != 0:
                                self.other_data.append(answer[i*5:i*5+5])
                    time.sleep(1)


if __name__ == "__main__":
    my_data = [16,1.,2.,3.,4.]
    client = Client(my_data)
    client.start()
    client.join()
