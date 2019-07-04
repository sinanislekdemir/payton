# NOTE! This example requires "requests" to be installed
# You can it install via `pip install requests`
import os
import requests
from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import Hud, Text


y = 0
z = 0

scene = Scene()
font_file = os.path.join(os.path.dirname(__file__), "../static/arial_narrow_7.ttf")


def fetch_instagram(name, scene, period, total):
    """This method runs once and continues to build the scene"""
    global y, z
    scene.clocks["fetch_instagram"].pause()
    url = "https://www.instagram.com/nasa/?__a=1"
    data = requests.get(url)
    images = data.json()["graphql"]["user"]["edge_owner_to_timeline_media"][
        "edges"
    ]
    for image in images:
        node = image["node"]
        for thum in node["thumbnail_resources"]:
            if thum["config_width"] == 640:
                src = thum["src"]
                print(f"Downloading {src}")
                f = requests.get(src, allow_redirects=True)
                with open(f"/tmp/{node['id']}.jpg", "wb") as fi:
                    fi.write(f.content)

                cube = Cube()
                cube.position = [0, y - 1, z]
                y += 1
                if y == 3:
                    y = 0
                    z += 1
                cube.material.texture = f"/tmp/{node['id']}.jpg"
                cube.info = node["edge_media_to_caption"]["edges"][0]["node"][
                    "text"
                ]
                scene.add_object(node["id"], cube)


def select(list):
    global scene
    print(list[0].info)
    scene.huds["hud"].children["info"].label = list[0].info
    scene.active_observer.target_object = list[0]


scene.on_select = select
scene.create_clock("fetch_instagram", 0.1, fetch_instagram)
hud = Hud()
hud.set_font(font_file)

text = Text(
    label="Instagram Photos", color=(1, 1, 1), position=(0, 0), size=(800, 600)
)
hud.add_child("info", text)

scene.add_object("hud", hud)

scene.run()
