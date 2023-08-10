import zmq

context = zmq.Context()
socket_send = context.socket(zmq.PUB)
port = 5555  # Unique port for each node
self_address = "tcp://*:" + str(port) 
# Bind the receiving socket
socket_send.bind(self_address)


socket_receive = context.socket(zmq.SUB)
# Connect the sending socket to other nodes' addresses
socket_receive.connect("tcp://localhost:5556")
#subscribe to all messages by using an empty string as the subscription filte
socket_receive.subscribe(b"")

poller = zmq.Poller()
poller.register(socket_receive, zmq.POLLIN)
while True:
    # Check for incoming messages
    socks = dict(poller.poll(100))  # Timeout of 100 milliseconds
    if socket_receive in socks and socks[socket_receive] == zmq.POLLIN:
        # Receive message from the other node
        message = socket_receive.recv_string()
        print("Received message:", message)

    # Publish a message to the other node
    reply = input("Enter a reply: ")
    socket_send.send_string(reply)