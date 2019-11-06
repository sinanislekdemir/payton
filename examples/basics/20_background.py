from payton.scene import Scene

target = [124 / 255, 185 / 255, 232 / 255]
initial = [0, 0.1, 0.2, 1.0]
rise = True


def change_background(period, total):
    global scene, rise
    c = scene.background.top_color
    if rise:
        if c[0] <= target[0]:
            c[0] += 0.01
        if c[1] <= target[1]:
            c[1] += 0.01
        if c[2] <= target[2]:
            c[2] += 0.01
        if sum(c) >= sum(target):
            rise = False
    else:
        if c[0] >= initial[0]:
            c[0] -= 0.01
        if c[1] >= initial[1]:
            c[1] -= 0.01
        if c[2] >= initial[2]:
            c[2] -= 0.01

        if sum(c) <= sum(initial) + 0.01:
            rise = True
    scene.background.top_color = c


scene = Scene()
scene.create_clock("sun", 0.1, change_background)
scene.huds["_help"].show()

scene.run()
