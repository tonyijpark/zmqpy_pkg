#!/usr/bin/python

import itertools
import sys
import time

import zmq

def main() :
    bind_to = "tcp://127.0.0.1:4242"
    all_topics = [
        'sports.general',
        'sports.football',
        'sports.basketball',
        'stocks.general',
        'stocks.GOOG',
        'stocks.AAPL',
        'weather',
    ]
    ctx = zmq.Context()
    s = ctx.socket(zmq.PUB)
    s.bind(bind_to)
    print("Starting broadcast on topics:")
    print("   %s" % all_topics)
    print("Hit Ctrl-C to stop broadcasting.")
    print("Waiting so subscriber sockets can connect...")
    print("")
    time.sleep(1.0)

    msg_counter = itertools.count()
    try:
        for topic in itertools.cycle(all_topics):
            msg_body = str(msg_counter.next())
            print('   Topic: %s, msg:%s' % (topic, msg_body))
            s.send_multipart([topic, msg_body])
            # short wait so we don't hog the cpu
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print("Waiting for message queues to flush...")
    time.sleep(0.5)
    print("Done.")

if __name__ == "__main__":
    main()