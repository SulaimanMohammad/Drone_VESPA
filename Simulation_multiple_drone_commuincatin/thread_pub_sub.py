import zmq
import threading

def publisher_thread():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")

    while True:
        message = input("Enter a message to publish (or 'exit' to quit): ")
        if message == 'exit':
            break
        socket.send_string(message)

def replier_thread():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5556")
    socket.subscribe(b"")
    print("start the listener")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    while True:
        events = dict(poller.poll(timeout=100))
        if socket in events and events[socket] == zmq.POLLIN:
            message = socket.recv_string()
            reply_message = f"Received: {message}"
            socket.send_string(reply_message)

if __name__ == "__main__":
    publisher = threading.Thread(target=publisher_thread)
    replier = threading.Thread(target=replier_thread)

    publisher.start()
    replier.start()

    publisher.join()
    replier.join()
