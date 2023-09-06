import zmq

def node_1():
    context = zmq.Context()
    socket_pub = context.socket(zmq.PUB)
    socket_pub.bind("tcp://*:5555")

    socket_sub = context.socket(zmq.SUB)
    socket_sub.connect("tcp://localhost:5556")
    socket_sub.setsockopt_string(zmq.SUBSCRIBE, '')

    while True:
        message = input("Node 1 - Enter a message to publish (or 'exit' to quit): ")
        if message == "exit":
            break
        socket_pub.send_string(message)

        for _ in range(3):
            received_message = socket_sub.recv_string()
            print("Node 1 - Received message:", received_message)

    socket_pub.close()
    socket_sub.close()
    context.term()

node_1()
