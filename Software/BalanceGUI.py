import pyglet
from pyglet import shapes




window = pyglet.window.Window()
pyglet.gl.glClearColor(0.7,0.7,0.7,1)
bg = pyglet.graphics.Batch()
feet = pyglet.graphics.Batch()


body = shapes.Rectangle(window.width//2, window.height//2, 90, 180, color=(255, 255, 20), batch=bg)
body.opacity = 255
body.anchor_position = (45,90)
center = shapes.Circle(window.width//2, window.height//2,10, color=(0, 0, 255), batch=bg)
center.opacity = 200
center.anchor_position = (0,0)


lbFoot = shapes.Circle(x=window.width//2-(90/2), y=window.height//2-(180/2), radius=10, color=(0, 0, 0),batch=feet)
rbFoot = shapes.Circle(x=window.width//2+(90/2), y=window.height//2-(180/2), radius=10, color=(0, 0, 0),batch=feet)
lfFoot = shapes.Circle(x=window.width//2-(90/2), y=window.height//2+(180/2), radius=10, color=(0, 0, 0),batch=feet)
rfFoot = shapes.Circle(x=window.width//2+(90/2), y=window.height//2+(180/2), radius=10, color=(0, 0, 0),batch=feet)

lbFoot.opacity = 100
rbFoot.opacity = 100
lfFoot.opacity = 100
rfFoot.opacity = 100
##body.anchor_position = (5,5)

@window.event
def on_draw():
  window.clear()
  bg.draw()
  feet.draw()



def update(dt):
    


pyglet.clock.schedule_interval(update,1./240.)
pyglet.app.run()