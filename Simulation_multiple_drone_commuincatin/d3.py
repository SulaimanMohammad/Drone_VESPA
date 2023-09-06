import zmq
import threading
import signal


signal.signal(signal.SIGINT, signal.SIG_DFL)

def pub(id, total_drones):
    context = zmq.Context()

    # Socket for replaying messages
    socket_rep = context.socket(zmq.PUB)
    self_address = "tcp://*:55" + id #str(id)
    socket_rep.bind(self_address)
    print("port to replay to ", self_address)

    # Socket for sending messages
    # Publish a message to the other node
    while True:
        reply = f"{id}"
        socket_rep.send_string(reply)
    socket_rep.close()
    context.term()


def sub(id, total_drones):
    context = zmq.Context()
    req=[]
    for target_port in range (0,total_drones):
        if target_port != int(id[0:]):
            socket_req = context.socket(zmq.SUB)
            target_address = "tcp://localhost:550"+ str(target_port)
            print("port to request from",target_address )
            socket_req.connect(target_address)
            socket_req.setsockopt(zmq.SUBSCRIBE, b"")
            req.append(socket_req) 

    poller = zmq.Poller()
    for soc in req:
        poller.register(soc, zmq.POLLIN)
    
    rec_msg=[]    
    x=0 
    while True: 
        socks = dict(poller.poll(1000))  # Timeout of 100 milliseconds
        for soc in req: 
            # Check for incoming messages
            if soc in socks and socks[soc] == zmq.POLLIN:     
                # Receive message from the other node
                message = int(soc.recv_string())
                print("Received message:", message)
                if message not in rec_msg:
                    x+=1
                    rec_msg.append(message)
                print(rec_msg )
    # for soc in req: 
    # # Close the sockets
    # soc.close()

# Create threads for each node

num=3
id="02"
threads = []
t1 = threading.Thread(target=pub, args=(id,num ))
threads.append(t1)
t2 = threading.Thread(target=sub, args=(id,num))
threads.append(t2)

# Start all threads
for thread in threads:
    thread.start()
# Wait for all threads to complete
for thread in threads:
    thread.join()


