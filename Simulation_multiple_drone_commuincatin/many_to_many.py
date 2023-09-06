import zmq
import threading

def node(node_id):
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind(f"tcp://*:555{node_id}")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    while True:
        events = dict(poller.poll())
        if socket in events and events[socket] == zmq.POLLIN:
            message = socket.recv_string()
            print(f"Node {node_id} received message: {message}")

            # Process the message

            if message == 'exit':
                break

            # Reply to the message
            reply_message = f"Reply from Node {node_id}"
            socket.send_string(reply_message)

def main():
    context = zmq.Context()
    nodes = []
    for i in range(4):
        t = threading.Thread(target=node, args=(i,))
        t.start()
        nodes.append(t)


    sender_id = int(input("Enter sender node ID (0-3): "))
    receiver_id = int(input("Enter receiver node ID (0-3): "))
    message = input("Enter message (or 'exit' to quit): ")

    sender_socket = context.socket(zmq.PAIR)
    sender_socket.connect(f"tcp://localhost:555{sender_id}")
    sender_socket.send_string(message)

    receiver_socket = context.socket(zmq.PAIR)
    receiver_socket.connect(f"tcp://localhost:555{receiver_id}")
    reply = receiver_socket.recv_string()
    print(f"Node {receiver_id} received reply: {reply}")

    for t in nodes:
        t.join()

if __name__ == "__main__":
    main()