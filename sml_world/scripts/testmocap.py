import mocap
import sys      #for exit
import signal

if __name__ == '__main__':

    stop = False
    def signal_handler(signal, frame):
        stop = True
        print "kjhskdfghskjvhgsdkj"
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)
    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")
    body = mocap.Body(Qs,id_body)
    while not stop:
        print body.getPose()
