import sys
import time

import zmq
import numpy


def main():
    connect_to = "tcp://127.0.0.1:4242"
    topics = ["sports.football"]

    ctx = zmq.Context()
    s = ctx.socket(zmq.SUB)
    s.connect(connect_to)

    # manage subscriptions
    if not topics:
        print("Receiving messages on ALL topics...")
        s.setsockopt(zmq.SUBSCRIBE, '')
    else:
        print("Receiving messages on topics: %s ..." % topics)
        for t in topics:
            s.setsockopt(zmq.SUBSCRIBE, t)
    print
    try:
        while True:
            topic, msg = s.recv_multipart()
            print('   Topic: %s, msg:%s' % (topic, msg))
    except KeyboardInterrupt:
        pass
    print("Done.")


if __name__ == "__main__":
    main()



    {
        "persons" : [
            {
                "mask" : "with_mask",   
                "bbox" : [10, 10, 100, 100]
            },
            {
                "mask" : "no_mask",
                "bbox" : [120, 10, 220, 100]
            },
            ...
        ]   
    }