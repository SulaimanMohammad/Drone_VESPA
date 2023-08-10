import zmq
import threading
import signal


signal.signal(signal.SIGINT, signal.SIG_DFL)
def node(id, total_drones):
    context = zmq.Context()

        # Socket for replaying messages
    socket_rep = context.socket(zmq.REP)
    self_address = "tcp://*:55" + id #str(id)
    socket_rep.bind(self_address)
    print("port to replay to ", self_address)

    # Socket for sending messages
    req=[]
    for target_port in range (0,total_drones):
        if target_port != int(id[0:]):
            socket_req = context.socket(zmq.REQ)
            target_address = "tcp://localhost:550"+ str(target_port)
            print("port to request from",target_address )
            #socket_req.subscribe(b"")
            socket_req.connect(target_address)
            req.append(socket_req) 
        #socket_rep.setsockopt(zmq.RCVTIMEO, 1000)  # Set a timeout of 1 second


    poller = zmq.Poller()
    for soc in req:
        poller.register(soc, zmq.POLLIN)

    for soc in req: 
        print(f"Node {id} sending meg req")
        request_message = f"{id}"
        soc.send_string(request_message)

    response_message=" "
    while response_message==" ":
        try:
            # replay will recive the message from client 
            # Receive the request from the  node
            response_message = socket_rep.recv_string()
            socket_rep.send_string(f"{id}") #send 
        except zmq.Again:
                    print("No request received") 

    x=0
    rec_msg=[]
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
       


    for soc in req: 
        # Close the sockets
        soc.close()

    socket_rep.close()
    context.term()

# Create threads for each node
num=3
node("00",num)


