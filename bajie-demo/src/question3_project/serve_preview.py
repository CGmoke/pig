#!/usr/bin/env python3
"""Serve only the Q3 viewer and its current run state on loopback."""
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path
import sys


def main():
    run = Path(sys.argv[1]).resolve(strict=True)
    page = Path(__file__).with_name('preview.html').read_bytes()

    def read(name, default):
        try:
            return json.loads((run / name).read_text(encoding='utf-8'))
        except FileNotFoundError:
            return default

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                data, content_type = page, 'text/html; charset=utf-8'
            elif self.path == '/state':
                data = json.dumps({'frame': read('frame.json', None),
                                   'exit': read('exit.json', None)}).encode()
                content_type = 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_args):
            pass

    with HTTPServer(('127.0.0.1', 0), Handler) as server:
        print(f'http://127.0.0.1:{server.server_port}/', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
