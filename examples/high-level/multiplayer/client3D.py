import logging
import math
import socket
import uuid

from payton.math.vector import distance
from payton.scene import Scene
from payton.scene.geometry import Cube

logger = logging.getLogger(__name__)


host = 'localhost'


class App(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_click_plane([0, 0, 0], [0, 0, 1], self.add_cube)
        self.on_select = self.pick_cube
        self._add_on_existing = False
        self._cube_count = 0
        self._socket = socket.socket()
        self._socket.connect((host, 6000))
        self._id = str(uuid.uuid4())
        self.create_clock("server", 0.001, self.listen)
        self.clocks["server"].pause()
        logger.debug(f"connecting to {host}:6000 with client_id {self._id}")

    def listen(self, period, total):
        print("waiting")
        data = self._socket.recv(63)
        data_str = data.decode('utf-8')
        parts = data_str.split('|')
        if parts[0] == self._id:
            return
        client_id = parts[0]
        x = float(parts[1])
        y = float(parts[2])
        z = float(parts[3])
        logger.debug(f"client {client_id} put box to {x}x{y}x{z}")
        self.create_cube([x, y, z])

    def create_cube(self, position):
        c = Cube()
        c.position = position
        self.add_object(f"cube_{self._cube_count}", c)
        self._cube_count += 1

    def add_cube(self, hit):
        if not self._add_on_existing:
            hit[0] = math.floor(hit[0]) + 0.5
            hit[1] = math.floor(hit[1]) + 0.5
            hit[2] = math.floor(hit[2]) + 0.5
            self.create_cube(hit)
            self.publish_action(hit)
        self._add_on_existing = False

    def publish_action(self, location):
        converted = ["%08.2f" % location[0], "%08.2f" % location[1], "%08.2f" % location[2]]
        data = f"{self._id}|{converted[0]}|{converted[1]}|{converted[2]}"
        print(len(data.encode('utf-8')))
        result = self._socket.send(data.encode('utf-8'))
        if result > 0:
            logger.debug(f"Result came back: {result}")

    def pick_cube(self, selection):
        self._add_on_existing = True
        dist = distance(selection[0].position, self.active_camera.position)
        nearest_pick = selection[0]
        for obj in selection:
            if distance(obj.position, self.active_camera.position) < dist:
                nearest_pick = obj
        pos = [
            nearest_pick.position[0],
            nearest_pick.position[1],
            nearest_pick.position[2] + 1,
        ]
        self.create_cube(pos)
        self.publish_action(pos)


s = App()
s.run()
