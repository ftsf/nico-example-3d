import nico/vec
import nico/quat
import nico/matrix
import nico

import strscans
import strutils
import math

{.experimental:"codeReordering".}

var depthBuffer: seq[uint8]
var near,far: float32
var perspectiveWidth,perspectiveHeight: float32

var modelMatrix: Mat4x4f = mat4x4f()
var viewMatrix: Mat4x4f = mat4x4f()
var projectionMatrix: Mat4x4f = mat4x4f()
var mvpDirty = false
var mvp: Mat4x4f = mat4x4f()

var cameraPos: Vec3f
var cameraDir: Vec3f
var cameraMat: Mat4x4f = mat4x4f()
var lightDir: Vec3f = vec3f(0,1,0)
var lightDithering = true

var depthTest: bool = false
var depthWrite: bool = false

const worldOrigin* = vec3f(0,0,0)
const worldUp* = vec3f(0,0,1)
const worldRight* = vec3f(1,0,0)
const worldForward* = vec3f(0,1,0)
const worldX* = vec3f(1,0,0)
const worldY* = vec3f(0,1,0)
const worldZ* = vec3f(0,0,1)

type Plane = tuple
  distance: float32
  normal: Vec3f

var nearPlane: Plane

template trifill(a,b,c: Vec2f): untyped =
  trifill(a.x, a.y, b.x, b.y, c.x, c.y)

proc perspectiveDivide*(a: var Vec4f)

proc setDepthBuffer*(newDepthBuffer: seq[uint8]) =
  depthBuffer = newDepthBuffer

proc setDither*(on: bool) =
  lightDithering = on

proc newDepthBuffer*() =
  depthBuffer = newSeq[uint8](screenWidth * screenHeight)

proc depthSet*(x,y: int, z: uint8) =
  depthBuffer[y * screenWidth + x] = z

proc depthGet*(x,y: int): uint8 =
  if x < 0 or y < 0 or x >= screenWidth or y >= screenHeight:
    return 255
  return depthBuffer[y * screenWidth + x]

proc onResize*(w,h: int) =
  newDepthBuffer()

addResizeFunc(onResize)

proc setPerspective*(fovDegrees: float32, znear, zfar: float32) =
  let aspect = screenWidth.float32 / screenHeight.float32
  perspectiveWidth = screenWidth.float32
  perspectiveHeight = screenHeight.float32
  projectionMatrix = perspectiveRH(fovDegrees.deg2rad, aspect, znear, zfar)
  mvpDirty = true
  near = znear
  far = zfar

  nearPlane.normal = vec3f(0,0,1)
  nearPlane.distance = 0

proc setUseDepth*(on: bool) =
  depthTest = on
  depthWrite = on

proc setCamera3D*(pos: Vec3f, at: Vec3f, up: Vec3f = worldUp) =
  cameraPos = pos
  cameraDir = (at - pos).normalized
  viewMatrix = lookAt(pos, at, up)
  mvpDirty = true

proc modelReset*() =
  modelMatrix = mat4x4f()
  mvpDirty = true

proc modelTranslate*(x,y,z: float32) =
  modelMatrix.translate(x,y,z)
  mvpDirty = true

proc modelTranslate*(v: Vec3f) =
  modelMatrix.translate(v)
  mvpDirty = true

proc modelRotate*(angle: float32, axis: Vec3f) =
  modelMatrix.rotate(angle, axis)
  mvpDirty = true


proc modelScale*(s: float32) =
  modelMatrix.scale(s)
  mvpDirty = true

proc modelScale*(v: Vec3f) =
  modelMatrix.scale(v)
  mvpDirty = true

proc modelForward*(): Vec3f =
  return modelMatrix.forward()

proc modelRight*(): Vec3f =
  return modelMatrix.right()

proc modelUp*(): Vec3f =
  return modelMatrix.up()

proc modelX*(): Vec3f =
  return modelMatrix.xaxis()

proc modelY*(): Vec3f =
  return modelMatrix.yaxis()

proc modelZ*(): Vec3f =
  return modelMatrix.zaxis()

proc refreshMVP() =
  if mvpDirty:
    mvp = projectionMatrix * viewMatrix * modelMatrix
    mvpDirty = false

proc setLightDir*(dir: Vec3f) =
  lightDir = dir

type
  Model2D* = object
    polys*: seq[seq[Vec2f]]
    colors*: seq[int]
  Face* = array[3,int]
  Face3D* = tuple
    verts: array[4,int]
    normals: array[4,int]
    uvs: array[4,int]
    faceNormal: Vec3f
  FaceSet* = tuple
    material: int
    faces: seq[Face3D]
  Model3D* = object
    vertices*: seq[Vec3f]
    normals*: seq[Vec3f]
    uvs*: seq[Vec2f]
    faceSets*: seq[FaceSet]

proc draw*(model: Model2D, pos: Vec2f, angle: float, colors: openArray[ColorId], scale = 1.0) =
  for i,poly in model.polys:
    let color = if i < model.colors.len:
      colors[model.colors[i]]
      else: colors[0]
    setColor(color)
    if poly.len == 3:
      let a = pos + rotate(poly[0], angle) * scale
      let b = pos + rotate(poly[1], angle) * scale
      let c = pos + rotate(poly[2], angle) * scale
      trifill(a,b,c)
      line(a,b)
      line(b,c)
      line(c,a)
    elif poly.len == 2:
      let a = pos + rotate(poly[0], angle) * scale
      let b = pos + rotate(poly[1], angle) * scale
      line(a,b)
    elif poly.len == 1:
      let a = pos + rotate(poly[0], angle) * scale
      pset(a.x,a.y)
    else:
      discard

proc draw3d*(model: Model2D, pos: Vec3f, angle: float, colors: openArray[ColorId], scale: float32) =
  for i,poly in model.polys:
    let color = if i < model.colors.len:
      colors[model.colors[i]]
      else: colors[0]
    setColor(color)
    if poly.len == 3:
      let a = pos + rotate(poly[0], angle).vec3f() * scale
      let b = pos + rotate(poly[1], angle).vec3f() * scale
      let c = pos + rotate(poly[2], angle).vec3f() * scale
      trifill3d(a,b,c)
      # lines to fatten it up
      line3d(a,b)
      line3d(b,c)
      line3d(c,a)
    elif poly.len == 2:
      let a = pos + rotate(poly[0], angle).vec3f() * scale
      let b = pos + rotate(poly[1], angle).vec3f() * scale
      line3d(a,b)
    elif poly.len == 1:
      let a = pos + rotate(poly[0], angle).vec3f() * scale
      pset(a.x,a.y)
    else:
      discard

proc perspectiveDivide*(a: var Vec4f) =
  a.x = -a.x / a.w
  a.y = -a.y / a.w
  a.z = a.z / a.w

  #normalizedSpaceToScreenSpace*(a: var Vec4f) =
  let hsw = perspectiveWidth * 0.5'f
  let hsh = perspectiveHeight * 0.5'f
  a.x = floor(hsw * (a.x + 1'f))
  a.y = floor(hsh * (a.y + 1'f))

proc barycentric(a,b,c: Vec2f, p: Vec2f): Vec3f =
  let v0 = b - a
  let v1 = c - a
  let v2 = p - a
  let d00 = dot(v0,v0)
  let d01 = dot(v0,v1)
  let d11 = dot(v1,v1)
  let d20 = dot(v2,v0)
  let d21 = dot(v2,v1)
  let denom = d00 * d11 - d01 * d01
  if abs(denom) < 0.001:
    return
  result.x = (d11 * d20 - d01 * d21) / denom
  result.y = (d00 * d21 - d01 * d20) / denom
  result.z = 1.0'f - result.x - result.y

proc hlineRaw(x0,x1,y: PInt) =
  for x in x0..x1:
    pset(x,y)

proc edgeFunc(ax,ay,bx,by,cx,cy: Pint): int =
  return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)

proc edgeFunc(ax,ay,bx,by,cx,cy: float32): float32 =
  return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)

proc edgeFunc(a,b,c: Vec2i): int =
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)

proc edgeFunc(a,b,c: Vec3f): float32 =
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)

proc edgeFunc(a,b,c: Vec2f): float32 =
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)

proc isTopLeft(a,b: Vec2i): bool =
  if a.y == b.y and b.x < a.x:
    # top edge
    return true

  if b.y > a.y:
    # left edge
    return true
  return false

type Edge = tuple
  oneStepX,oneStepY: Vec4i

proc initEdge(v0,v1,origin: Vec2i): (Edge, Vec4i) =
  let A = v0.y - v1.y
  let B = v1.x - v0.x
  let C = v0.x*v1.y - v0.y*v1.x

  result[0].oneStepX = vec4i(A * 1)
  result[0].oneStepY = vec4i(B * 1)

  let x = vec4i(origin.x)
  let y = vec4i(origin.y)

  result[1] = vec4i(A) * x + vec4i(B)*y + vec4i(C)

proc getDepthValue*(z: float32): uint8 =
  result = clamp((z * z) * 255, 0, 255).uint8

#proc trifillDepth*(a,b,c: Vec3f) =
#  let camera = vec3f(cameraX.float32, cameraY.float32,0)
#
#  let az = a.z
#  let bz = b.z
#  let cz = c.z
#
#  let a = (a - camera).xyi
#  let b = (b - camera).xyi
#  let c = (c - camera).xyi
#
#  let topLeft = vec2i(max(min(min(a.x, b.x), c.x), clipMinX), max(min(min(a.y, b.y), c.y), clipMinY))
#  let bottomRight = vec2i(min(max(max(a.x, b.x), c.x), clipMaxX), min(max(max(a.y, b.y), c.y), clipMaxY))
#
#  let
#    A01 = a.y - b.y
#    B01 = b.x - a.x
#    A12 = b.y - c.y
#    B12 = c.x - b.x
#    A20 = c.y - a.y
#    B20 = a.x - c.x
#
#  var bias0 = if isTopLeft(b, c): 0 else: -1
#  var bias1 = if isTopLeft(c, a): 0 else: -1
#  var bias2 = if isTopLeft(a, b): 0 else: -1
#
#  var w0_row = edgeFunc(b.xyi,c.xyi,topLeft)
#  var w1_row = edgeFunc(c.xyi,a.xyi,topLeft)
#  var w2_row = edgeFunc(a.xyi,b.xyi,topLeft)
#
#  w0_row += bias0
#  w1_row += bias1
#  w2_row += bias2
#
#  var py = topLeft.y
#  var px = topLeft.x
#
#  for py in topLeft.y.int..bottomRight.y.int:
#    var w0 = w0_row
#    var w1 = w1_row
#    var w2 = w2_row
#
#    for px in countup(topLeft.x.int,bottomRight.x.int):
#      if (w0 or w1 or w2) >= 0:
#        #let uvw = barycentric(a.xyf,b.xyf,c.xyf, vec2f(px.float32,py.float32))
#        #let depth = (uvw.x * az + uvw.y * bz + uvw.z * cz)
#        #let depth = az * w0 + bz * w1 * cz * w2
#        #let z = getDepthValue(depth)
#        #debug az, bz, cz, z
#        debug w0.float32 / (bottomRight.x - topLeft.x).float32, w1.float32 / A20.float32, w2.float32 / A01.float32
#        psetRaw(px,py)
#        #depthSet(px,py,z)
#
#      w0 += A12
#      w1 += A20
#      w2 += A01
#
#    w0_row += B12
#    w1_row += B20
#    w2_row += B01

proc trifillDepth*(a,b,c: Vec4f) =
  let camera = vec2i(cameraX, cameraY)

  let az = a.z
  let bz = b.z
  let cz = c.z

  let area = edgeFunc(a.xy,b.xy,c.xy)
  if area < 0.01:
    return

  let a = (a.xyi - camera)
  let b = (b.xyi - camera)
  let c = (c.xyi - camera)

  let topLeft = vec2i(max(min(min(a.x, b.x), c.x), clipMinX), max(min(min(a.y, b.y), c.y), clipMinY))
  let bottomRight = vec2i(min(max(max(a.x, b.x), c.x), clipMaxX), min(max(max(a.y, b.y), c.y), clipMaxY))

  let
    A01 = a.y - b.y
    B01 = b.x - a.x
    A12 = b.y - c.y
    B12 = c.x - b.x
    A20 = c.y - a.y
    B20 = a.x - c.x

  var bias0 = if isTopLeft(b, c): 0 else: -1
  var bias1 = if isTopLeft(c, a): 0 else: -1
  var bias2 = if isTopLeft(a, b): 0 else: -1

  var w0_row = edgeFunc(b.xyi,c.xyi,topLeft)
  var w1_row = edgeFunc(c.xyi,a.xyi,topLeft)
  var w2_row = edgeFunc(a.xyi,b.xyi,topLeft)

  #w0_row += bias0
  #w1_row += bias1
  #w2_row += bias2

  var py = topLeft.y
  var px = topLeft.x

  if depthTest:
    stencilMode(stencilLEqual)
  else:
    stencilMode(stencilAlways)
  setStencilWrite(depthWrite)

  for py in topLeft.y..bottomRight.y:
    var w0 = w0_row
    var w1 = w1_row
    var w2 = w2_row

    for px in topLeft.x..bottomRight.x:
      if (w0 or w1 or w2) >= 0:
        let w0p = w0.float32 / area
        let w1p = w1.float32 / area
        let w2p = w2.float32 / area
        let depth = az*w0p + bz*w1p + cz*w2p
        let z = getDepthValue(depth)
        setStencilRef(z)
        psetRaw(px,py)
        #if z < depthGet(px,py):
        #  psetRaw(px,py)
        #  depthSet(px,py,z)

      w0 += A12
      w1 += A20
      w2 += A01

    w0_row += B12
    w1_row += B20
    w2_row += B01

  stencilMode(stencilAlways)
  setStencilWrite(false)

proc clipPlaneTest*(a: Vec4f): bool =
  if a.w <= 0:
    return false
  if a.w > 0 and abs(a.z) < a.w:
    return true
  return false

proc trifill3d*(a,b,c: Vec3f) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  var c = mvp * vec4f(c,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  c.perspectiveDivide()
  if not c.clipPlaneTest(): return
  trifillDepth(a,b,c)

template trifill3dr*(a,b,c: Vec3f) = trifill3d(c,b,a)

proc quadfill3d*(a,b,c,d: Vec3f) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  var c = mvp * vec4f(c,1)
  var d = mvp * vec4f(d,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  c.perspectiveDivide()
  if not c.clipPlaneTest(): return
  d.perspectiveDivide()
  if not d.clipPlaneTest(): return
  trifillDepth(a,b,c)
  trifillDepth(a,c,d)

template quadfill3dr*(a,b,c,d: Vec3f) = quadfill3d(d,c,b,a)

proc line3d*(a,b: Vec3f) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  let da = dot(a.xyz, nearPlane.normal) - nearPlane.distance
  let db = dot(b.xyz, nearPlane.normal) - nearPlane.distance
  if da < 0 and db < 0:
    # if both are zero it's behind the clipping plane
    return
  elif (da < 0 and db > 0) or (da > 0 and db < 0):
    # need to clip it
    let s = da/(da-db)
    let ip = a.xyz + (b.xyz-a.xyz) * s
    if da < 0:
      a = vec4f(ip,1)
    elif db < 0:
      b = vec4f(ip,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  line(a.x,a.y,b.x,b.y)

proc lineDashed3d*(a,b: Vec3f, pattern: uint8 = 0b1100_1100) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  lineDashed(a.x,a.y,b.x,b.y, pattern)

proc quad3d*(a,b,c,d: Vec3f) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  var c = mvp * vec4f(c,1)
  var d = mvp * vec4f(d,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  c.perspectiveDivide()
  if not c.clipPlaneTest(): return
  d.perspectiveDivide()
  if not d.clipPlaneTest(): return
  line(a.x,a.y,b.x,b.y)
  line(b.x,b.y,c.x,c.y)
  line(c.x,c.y,d.x,d.y)
  line(d.x,d.y,a.x,a.y)

proc tri3d*(a,b,c: Vec3f) =
  refreshMVP()
  var a = mvp * vec4f(a,1)
  var b = mvp * vec4f(b,1)
  var c = mvp * vec4f(c,1)
  a.perspectiveDivide()
  if not a.clipPlaneTest(): return
  b.perspectiveDivide()
  if not b.clipPlaneTest(): return
  c.perspectiveDivide()
  if not c.clipPlaneTest(): return
  line(a.x,a.y,b.x,b.y)
  line(b.x,b.y,c.x,c.y)
  line(c.x,c.y,a.x,a.y)

proc circfill3d*(c: Vec3f, r: float32) =
  refreshMVP()
  var c = mvp * vec4f(c,1)
  c.perspectiveDivide()
  if not c.clipPlaneTest(): return
  let z = getDepthValue(c.z)

  if depthTest:
    stencilMode(stencilLEqual)
  else:
    stencilMode(stencilAlways)

  setStencilRef(z - 2)
  setStencilWrite(depthWrite)
  circfill(c.x,c.y,((r*screenHeight.float32)/c.w))

proc circflat3d*(c: Vec3f, r: float32, up: Vec3f = vec3f(0,0,1), steps = 32) =
  var prev: Vec3f
  let angleInc = TAU / steps.float32
  var angle = 0.0'f
  for i in 0..steps:
    var current = vec3f(sin(angle), cos(angle), 0) * r
    if i >= 1:
      line3d(c + prev, c + current)
    prev = current
    angle += angleInc

proc drawFlat*(model: Model3D, colors: openarray[ColorId]) =
  for i in 0..<model.faceSets.len:
    var color = colors[model.faceSets[i].material]
    setColor(color)
    for face in model.faceSets[i].faces:
      var a = model.vertices[face.verts[0]]
      var b = model.vertices[face.verts[1]]
      var c = model.vertices[face.verts[2]]
      if face.verts[3] >= 0:
        var d = model.vertices[face.verts[3]]
        quadfill3d(a,b,c,d)
      else:
        trifill3d(a,b,c)

proc drawLit*(model: Model3D, colors: array[8, ColorId]) =
  let lightDir = lightDir.normalized
  let normalMatrix = transpose(inverse(modelMatrix))
  for i in 0..<model.faceSets.len:
    for face in model.faceSets[i].faces:
      var a = model.vertices[face.verts[0]]
      var b = model.vertices[face.verts[1]]
      var c = model.vertices[face.verts[2]]
      var normal = face.faceNormal

      refreshMVP()

      normal = (modelMatrix * vec4f(normal,0)).xyz

      #let eyeNormal = (normalMatrix * vec4f(normal,0)).normalized.xyz

      # cull backfaces
      #if normal.dot(-cameraDir) < 0:
      #  continue

      #normal = (normalMatrix * vec4f(normal,0)).xyz.normalized
      #normal = (modelMatrix * vec4f(normal,0)).xyz.normalized

      # TODO improve depth testing

      let dot = normal.normalized().dot(-lightDir.normalized())

      if lightDithering:
        let lightLevel = clamp(dot * 32, 0, 31).int
        let highColor = colors[lightLevel div 4] # 31 = 7
        let baseColor = colors[if lightLevel <= 4: 0 else: lightLevel div 4 - 1] # 31 = 6
        if lightLevel mod 4 == 3:
          ditherPattern()
        elif lightLevel mod 4 == 2:
          ditherPattern(0b1111_1011_1111_1110)
        elif lightLevel mod 4 == 1:
          ditherPattern(0b1111_1010_1111_1010)
        elif lightLevel mod 4 == 0:
          ditherPattern(0b0000_0100_0000_0001)
        setDitherColor(baseColor)
        setColor(highColor)
      else:
        let lightLevel = clamp(dot * 8, 0, 7).int
        let baseColor = colors[lightLevel]
        setColor(baseColor)

      setUseDepth(true)
      if face.verts[3] >= 0:
        let d = model.vertices[face.verts[3]]
        quadfill3d(a,b,c,d)
      else:
        trifill3d(a,b,c)

      setDitherColor()
      ditherPattern()

      # draw normals
      #var mp = (a+b+c) / 3.0'f
      #var np = mp + normal * 1.0'f
      #setColor(12)
      #setUseDepth(false)
      #line3d(mp, np)

proc drawWire*(model: Model3D, seeThrough: bool = true) =
  for i in 0..<model.faceSets.len:
    for face in model.faceSets[i].faces:
      let a = model.vertices[face.verts[0]]
      let b = model.vertices[face.verts[1]]
      let c = model.vertices[face.verts[2]]

      refreshMVP()

      #if not seeThrough:
      #  var normal = (model.normals[ni[0]] + model.normals[ni[1]] + model.normals[ni[2]]) / 3.0'f
      #  normal = (modelMatrix * vec4f(normal,0)).xyz
      #  if normal.dot(-cameraDir) < 0:
      #    continue

      if face.verts[3] >= 0:
        let d = model.vertices[face.verts[3]]
        quad3d(a,b,c,d)
      else:
        tri3d(a,b,c)

proc drawAxes*(pos: Vec3f = vec3f(0,0,0), scale = 1.0'f) =
  setColor(7)
  circfill3d(pos,2)
  setColor(8)
  line3d(pos, pos + vec3f(1,0,0) * scale)
  circfill3d(pos + vec3f(1,0,0) * scale, 2)
  setColor(11)
  line3d(pos, pos + vec3f(0,1,0) * scale)
  circfill3d(pos + vec3f(0,1,0) * scale, 2)
  setColor(12)
  line3d(pos, pos + vec3f(0,0,1) * scale)
  circfill3d(pos + vec3f(0,0,1) * scale, 2)

var depthPal*: array[256, tuple[r,g,b: uint8]]
for i in 0..<256:
  let i = i.uint8
  depthPal[i] = (r:i,g:i,b:i)

proc drawDepthBuffer*() =
  for y in 0..<screenHeight:
    for x in 0..<screenWidth:
      let v = stencilGet(x,y)
      psetRaw(x,y,v.ColorId)

proc loadObj*(filename: string): Model3D =
  result.vertices = @[]
  result.normals = @[]
  result.uvs = @[]
  result.faceSets = @[(material: 0, faces: newSeq[Face3D]())]

  var usingMaterialGroups = false
  var faceSet = 0
  var material = 0

  var fp = open(joinPath(assetPath,filename), fmRead)

  var line: string
  while fp.readLine(line):
    if line.startsWith("g "):
      var groupName: string
      var materialId = 0
      proc matchMaterialId(input: string, output: var string, start: int): int =
        result = 0
        while start+result < input.len and input[start+result] in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._":
          output.add(input[start+result])
          inc result
      if scanf(line, "g ${matchMaterialId}", groupName):
        var start = groupName.rfind('_')
        if start != -1:
          try:
            materialId = parseInt(groupName[start+1..groupName.high]) - 1
          except:
            materialId = 0
      else:
        discard
      if not usingMaterialGroups:
        usingMaterialGroups = true
      else:
        faceSet += 1
        result.faceSets.add((material: materialId, faces: @[]))
    elif line.startsWith("v "):
      # vertex
      var x,y,z: float
      if scanf(line, "v $f $f $f", x,y,z):
        result.vertices.add(vec3f(x,y,z))
      else:
        raise newException(IOError, "invalid vertex: $1".format(line))
    elif line.startsWith("vt "):
      # texcoord
      var u,v: float
      if scanf(line, "vt $f $f", u,v):
        result.uvs.add(vec2f(u,1.0-v))
      else:
        echo "invalid uv: ", line
    elif line.startsWith("vn "):
      # normal
      var x,y,z: float
      if scanf(line, "vn $f $f $f", x,y,z):
        result.normals.add(vec3f(x,y,z).normalized)
      else:
        echo "invalid normal: ", line
    elif line.startsWith("f "):
      # face
      var v1,v2,v3,v4: int
      var uv1,uv2,uv3,uv4: int
      var n1,n2,n3,n4: int
      if scanf(line, "f $i/$i/$i/$i $i/$i/$i/$i $i/$i/$i/$i", v1,uv1,n1, v2,uv2,n2, v3,uv3,n3, v4,uv4,n4):
        var face: Face3D
        face.verts = [v1-1,v2-1,v3-1,v4-1]
        face.uvs = [uv1-1,uv2-1,uv3-1,uv4-1]
        face.normals = [n1-1,n2-1,n3-1,n4-1]
        face.faceNormal = (result.normals[n1-1] + result.normals[n2-1] + result.normals[n3-1] + result.normals[n4-1]) / 4.0'f
        result.faceSets[faceSet].faces.add(face)
      elif scanf(line, "f $i/$i/$i $i/$i/$i $i/$i/$i", v1,uv1,n1, v2,uv2,n2, v3,uv3,n3):
        var face: Face3D
        face.verts = [v1-1,v2-1,v3-1,-1]
        face.uvs = [uv1-1,uv2-1,uv3-1,-1]
        face.normals = [n1-1,n2-1,n3-1,-1]
        face.faceNormal = (result.normals[n1-1] + result.normals[n2-1] + result.normals[n3-1]) / 3.0'f
        result.faceSets[faceSet].faces.add(face)
      elif scanf(line, "f $i//$i $i//$i $i//$i $i//$i", v1,n1, v2,n2, v3,n3, v4,n4):
        var face: Face3D
        face.verts = [v1-1,v2-1,v3-1,v4-1]
        face.normals = [n1-1,n2-1,n3-1,n4-1]
        face.faceNormal = (result.normals[n1-1] + result.normals[n2-1] + result.normals[n3-1] + result.normals[n4-1]) / 4.0'f
        result.faceSets[faceSet].faces.add(face)
      elif scanf(line, "f $i//$i $i//$i $i//$i", v1,n1, v2,n2, v3,n3):
        var face: Face3D
        face.verts = [v1-1,v2-1,v3-1,-1]
        face.normals = [n1-1,n2-1,n3-1,-1]
        face.faceNormal = (result.normals[n1-1] + result.normals[n2-1] + result.normals[n3-1]) / 3.0'f
        result.faceSets[faceSet].faces.add(face)
      else:
        echo "invalid face: ", line

  fp.close()
