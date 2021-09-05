import std/strutils

import nico
import nico/vec
import nico/matrix

import model

var obj: Model3D

var cameraX,cameraY,cameraZ: float32

var pitch,yaw,roll: float32

var cameraControl = true

var frame = 0

var fov: float32 = 27.0

var lightPos = vec3f(5, 5, 5)

var time = 0.0'f

var drawWire = false

proc gameInit() =
  nico.createWindow("test3d", 240, 136, 2, false)
  setPalette(loadPalettePico8Extra())

  obj = loadObj("car.obj")
  cameraZ = 5.0'f
  cameraY = 10.0'f
  setPerspective(50.0'f, 1.0'f, 100.0'f)

proc gameUpdate(dt: float32) =
  time += dt * 0.5'f

  if btn(pcA):
    cameraZ += 0.2'f
  if btn(pcB):
    cameraZ -= 0.2'f

  if btn(pcLeft):
    cameraX -= 0.2'f
  if btn(pcRight):
    cameraX += 0.2'f
  if btn(pcUp):
    cameraY -= 0.2'f
  if btn(pcDown):
    cameraY += 0.2'f

  if btnp(pcX):
    drawWire = not drawWire

  lightPos.x = cos(time * 0.1) * 5
  lightPos.y = sin(time * 0.1) * 5

let enginePos = vec3f(0, 6.5, 1.7)

proc gameDraw() =
  frame+=1
  cls()
  stencilClear(255)
  setStencilWrite(false)

  let cameraPos = vec3f(cameraX, cameraY, cameraZ)

  setCamera3D(cameraPos, vec3f(0,0,0), vec3f(0,0,1))
  setLightDir(-lightPos.normalized)

  modelReset()


  setColor(1)
  for x in -10..10:
    let x = x.float32
    line3d(vec3f(x,-10,0), vec3f(x,10,0))
  for y in -10..10:
    let y = y.float32
    line3d(vec3f(-10,y,0), vec3f(10,y,0))

  setColor(5)
  circflat3d(worldOrigin, 5, worldUp, 64)
  circflat3d(worldOrigin, 10, worldUp, 64)
  lineDashed3d(vec3f(0,0,0), vec3f(lightPos.x, lightPos.y, 0))
  lineDashed3d(vec3f(0,0,0), vec3f(cameraPos.x, cameraPos.y, 0).clamp(10))
  setColor(6)
  lineDashed3d(lightPos, vec3f(lightPos.x, lightPos.y, 0))
  setColor(7)
  circfill3d(lightPos, 1)
  circflat3d(vec3f(lightPos.x, lightPos.y, 0), 1, worldUp, 64)

  modelRotate(time * 0.7, vec3f(0,0,1))
  obj.drawLit([1.ColorId,2,2,2, 8,8,8,7])

  setColor(12)
  circfill3d(enginePos, rnd(1.1, 2.5).float32)
  setColor(7)
  circfill3d(enginePos, rnd(0.5, 1).float32)

  stencilMode(stencilLEqual)
  setStencilRef(0)

  if drawWire:
    setColor(0)
    obj.drawWire(false)

  modelReset()
  #drawAxes(vec3f(0,0,0), 10)

  stencilMode(stencilAlways)

  setCamera()
  print("x: " & $cameraX, 1, 1)
  print("y: " & $cameraY, 1, 10)
  print("z: " & $cameraZ, 1, 20)

nico.init("game","game3d")
fixedSize(false)
integerScale(true)

nico.run(gameInit,gameUpdate,gameDraw)

