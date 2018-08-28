#!/usr/bin/env python3

from collections import namedtuple
import psycopg2
import socketserver
import struct

Reading = namedtuple('Reading', ['lux', 'pressure', 'temperature', 'humidity', 'tvoc', 'eco2', 'full', 'ir', 'gain', 'timing', 'mic_min', 'mic_max', 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'])

DB = None

def init_db():
    global DB
    if not DB or DB.closed != 0:
        DB = psycopg2.connect("dbname='roommetrics' user='kevin' host='localhost' password='Lb289zWj'")
        DB.set_session(autocommit=True)

init_db()

class HandlePacket(socketserver.BaseRequestHandler):

    def handle(self):
        data = self.request[0].strip()
        data = Reading(*struct.unpack("<4f14H", data))
        DB.cursor().execute('INSERT INTO data VALUES (NOW(), ' + ','.join(str(x) for x in data) + ');')

if __name__ == "__main__":
    with socketserver.UDPServer(("0.0.0.0", 9942), HandlePacket) as server:
        server.serve_forever()
