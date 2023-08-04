import threading

def action():
    while not stop_event.is_set():
        print("hello")
        stop_event.wait(1)
        
def interrupt():
    key=input("type Q to stop...\n")
    if (key =="Q"):
        stop_event.set()
    else:
        interrupt()
        
stop_event = threading.Event()

output_thread = threading.Thread(target=action)
input_thread = threading.Thread(target=interrupt)

input_thread.start()
output_thread.start()

input_thread.join()
output_thread.join()

print("End")