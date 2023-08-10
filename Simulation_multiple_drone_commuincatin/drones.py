import zmq
import threading

def node(id, total_drones, barrier):
    context = zmq.Context()

    # Socket for sending messages
    socket_req = context.socket(zmq.REQ)
    self_address = "tcp://localhost:55" + id #str(id)
    socket_req.connect(self_address)
    print("connect to self_address ", self_address)

    # Socket for replaying messages
    socket_rep = context.socket(zmq.REP)
    for target_port in range (0,total_drones):
        if target_port != int(id[0:]):
            target_address = "tcp://*:550"+ str(target_port)
            print("port to bind to target_port",target_address )
            socket_rep.bind(target_address)
            #socket_rep.setsockopt(zmq.RCVTIMEO, 1000)  # Set a timeout of 1 second
    
    # Wait for all nodes to bind their sockets
    barrier.wait()
    print("pass barrier")
    # Send a request to the target node and wait for read the result 
    print(f"Node {id} sending meg req")
    request_message = f"Node {id} requesting data of location"
    socket_req.send_string(request_message)
    
    try:
    # replay will recive the message from client 
    # Receive the response from the target node
        response_message = socket_rep.recv_string(flags=zmq.NOBLOCK)
        print(f"Node {id} received response:", response_message)
        socket_rep.send_string("my location is known to you")
    except zmq.Again:
                print("No request received")

    print(f"Node {id} recive as result of request  meg req")
    response_message = socket_req.recv_string(flags=zmq.NOBLOCK)
    print(f"Node {id} received response:", response_message)

    # Close the sockets
    socket_req.close()
    socket_rep.close()
    context.term()

# Create threads for each node
threads = []
num=3
barrier = threading.Barrier(num + 1)  # The "+1" accounts for the main thread

for i in range(0, num):
    t = threading.Thread(target=node, args=("0"+str(i),num, barrier))
    threads.append(t)


# Start all threads
for thread in threads:
    thread.start()
# Wait for all threads to complete the bind operation
barrier.wait()
# Wait for all threads to complete
for thread in threads:
    thread.join()


