import zmq
import threading

def node(id, total_drones):
    context = zmq.Context()
    req=[]
    for target_port in range (0,total_drones):
        socket_req = context.socket(zmq.REQ)
        if target_port != int(id[0:]):
            target_address = "tcp://localhost:550"+ str(target_port)
            print("port to request from",target_address )
            socket_req.connect(target_address)
            req.append(socket_req) 
        #socket_rep.setsockopt(zmq.RCVTIMEO, 1000)  # Set a timeout of 1 second

    # Socket for replaying messages
    socket_rep = context.socket(zmq.REP)
    self_address = "tcp://*:55" + id #str(id)
    socket_rep.bind(self_address)
    print("port to replay to ", self_address)

    
    # Wait for all nodes to bind their sockets
    #    print("pass barrier")
    # Send a request to the target node and wait for read the result 
    for sock in req: 
        print(f"Node {id} sending meg req")
        request_message = f"Node {id} requesting data of location"
        sock.send_string(request_message)
        
    response_message=" "
    while response_message==" ":
        try:
            # replay will recive the message from client 
            # Receive the response from the target node
            response_message = socket_rep.recv_string()
            print(f"Node {id} received response:", response_message)
            socket_rep.send_string(f"my location is known to you{id}")
        except zmq.Again:
                    print("No request received") 

    for sock in req: 
        print(f"Node {id} recive as result of request  meg req")
        response_message = sock.recv_string()
        print(f"Node {id} received response:", response_message)    
    


    


    # Close the sockets
    socket_req.close()
    socket_rep.close()
    context.term()

# Create threads for each node
num=3
node("01",num)


