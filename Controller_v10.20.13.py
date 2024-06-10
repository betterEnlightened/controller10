import FreeCAD, Part, math

#(l)ength       : (w)est (X-)   (c)entre (X0)   (e)ast (X+)
#(b)readth      : (s)outh (Y-)  (m)iddle (Y0)   (n)orth (Y+)
#(h)eight       : (d)own (Z-)   (g)round (Z0)   (u)p (Z+)

#Origins (o)    : CMD of mainboard
#Vectors (v)    : (x), (y), (z)
#Joints (j)
#Radii (r)
#Apothemns (a)
#Proportion (p) : dictionary for grouping radius and intensity into a single object
#Intensity (i)  : length/breath/width of object independant of orientation
#Thicknesses (t): minimum, nominal, large, protected
#Fitgaps (f)    : tight0, tight1, close0, close1, loose0, loose1

#Iterators      : (k), (q)

#changelog
#frame.d z -2mm to provide more internal space for driver components
#frame.u y +2mm to provide more internal space for wires
#panel wash sink radius fitgap changed from close1 to tight1
#bDialShaftFlat fitgap from tight1 to tight0

FreeCAD.newDocument('Controller_v10.20.13')
FreeCAD.Gui.runCommand('Std_DrawStyle',6) #shaded wireframe
intensityInd = True
directionInd = True
resetButton = True
directionButton = True
rotaryEnc = False

o = FreeCAD.Vector(0, 0, 0)

#t wall thickness calibrated for 0.40mm extrusion width 0.160mm layer height
t = {"min": FreeCAD.Vector(0.8, 0.8, 0.32)\
    ,"nom": FreeCAD.Vector(1.6, 1.6, 0.64)\
    ,"lrg": FreeCAD.Vector(2.4, 2.4, 0.96)\
    ,"pro": FreeCAD.Vector(3.2, 3.2, 1.28)}

#f fitgap calibrated for 0.40mm extrusion width & 0.160mm layer height
f = {"tight0": FreeCAD.Vector(0.16, 0.16, 0.16)\
    ,"tight1": FreeCAD.Vector(0.22, 0.22, 0.16)\
    ,"close0": FreeCAD.Vector(0.28, 0.28, 0.16)\
    ,"close1": FreeCAD.Vector(0.40, 0.40, 0.16)\
    ,"loose0": FreeCAD.Vector(0.80, 0.80, 0.32)\
    ,"loose1": FreeCAD.Vector(1.20, 1.20, 0.48)}

class BeFlat:
    def __init__(self, vertices, joints = [], normal = FreeCAD.Vector(0, 0, 0)):
        self.v = vertices
        self.j = joints
        self.normal = normal
        
        bc = 0.55228474983079 #bezier curve coefficient (https://spencermortensen.com/articles/bezier-circle/)
        edges = []
        if len(self.j) < len(self.v):
            for k in range(len(self.j), len(self.v)):
                self.j.append(FreeCAD.Vector(0, 0, 0))
        if self.v[0] == self.v[-1]:
            vector = self.v[-1] - self.v[-2]
            vertexC = self.v[-1] +(self.j[-1].dot(vector)/vector.Length *vector.normalize())
            vertexD = vertexC +(bc *(self.v[-1] -vertexC))
        for k in range(1, len(self.v)):
            vector = self.v[k] -self.v[k-1]
            vertexB = self.v[k-1] +(self.j[k-1].dot(vector)/vector.Length *vector.normalize())
            vertexA = vertexB +(bc *(self.v[k-1] -vertexB))
            if not self.j[k-1].isEqual(FreeCAD.Vector(0, 0, 0), 0):
                curve = Part.BezierCurve()
                curve.setPoles([vertexC, vertexD, vertexA, vertexB])
                edges.append(curve.toShape())
            vertexC = self.v[k] +(self.j[k].dot(vector)/vector.Length *vector.normalize())
            if not vertexB.isEqual(vertexC, 0):
                edges.append(Part.LineSegment(vertexB, vertexC).toShape())
            vertexD = vertexC +(bc *(self.v[k] -vertexC))
        self.wire = Part.Wire(edges)
        self.face = Part.Face(self.wire)
        if self.normal.isEqual(FreeCAD.Vector(0, 0, 0), 0):
            self.normal = self.face.normalAt(0, 0)

class BeBox:
    def __init__(self, vWSD, vENU, joints = []):
        self.vWSD = vWSD
        self.vENU = vENU
        self.j = joints
        
        if len(self.j) == 1:
            self.j.append(FreeCAD.Vector(-self.j[0].x, self.j[0].y, 0))
            self.j.append(FreeCAD.Vector(-self.j[0].x, -self.j[0].y, 0))
            self.j.append(FreeCAD.Vector(self.j[0].x, -self.j[0].y, 0))
        if len(self.j) == 4:
            self.j.append(self.j[0])
        self.vESD = FreeCAD.Vector(self.vENU.x, self.vWSD.y, self.vWSD.z)
        self.vEND = FreeCAD.Vector(self.vENU.x, self.vENU.y, self.vWSD.z)
        self.vWND = FreeCAD.Vector(self.vWSD.x, self.vENU.y, self.vWSD.z)
        self.vWNU = FreeCAD.Vector(self.vWSD.x, self.vENU.y, self.vENU.z)
        self.vWSU = FreeCAD.Vector(self.vWSD.x, self.vWSD.y, self.vENU.z)
        self.vESU = FreeCAD.Vector(self.vENU.x, self.vWSD.y, self.vENU.z)
        self.w = self.vWSD.x
        self.e = self.vENU.x
        self.s = self.vWSD.y
        self.n = self.vENU.y
        self.d = self.vWSD.z
        self.u = self.vENU.z
        self.c = (self.vWSD.x + self.vENU.x) /2
        self.m = (self.vWSD.y + self.vENU.y) /2
        self.g = (self.vWSD.z + self.vENU.z) /2
        self.l = self.vENU.x - self.vWSD.x
        self.b = self.vENU.y - self.vWSD.y
        self.h = self.vENU.z - self.vWSD.z
        self.vCMG = FreeCAD.Vector(self.c, self.m, self.g)
        flat = BeFlat([self.vWSD, self.vESD, self.vEND, self.vWND, self.vWSD], self.j)
        self.solid = flat.face.extrude(FreeCAD.Vector(0, 0, self.vENU.z -self.vWSD.z))

class BeAgon:
    def __init__(self, centre, radius, normal, sides = 1, twist = 0):
        self.centre = centre
        self.radius = radius
        self.normal = normal
        self.sides = sides
        self.twist = twist
        
        circle = Part.Circle(self.centre, self.normal, self.radius)
        circle.rotate(FreeCAD.Placement(self.centre, self.normal, self.twist))
        self.v = [self.centre +(circle.XAxis *self.radius)]
        self.j = []
        self.wire = Part.Wire(circle.toShape())
        self.apothem = self.radius *math.cos(math.pi/self.sides)
        if self.sides > 1:
            self.v = circle.discretize(Number = self.sides +1)
            self.wire = BeFlat(self.v).wire
        for k in self.v:
            self.j.append(FreeCAD.Vector(0, 0, 0))
        self.face = Part.Face(self.wire)

class BeDron:
    def __init__(self, flatagons):
        self.face = []
        self.wire = []
        self.normal = []
        self.i = []
        
        for k in flatagons:
            self.face.append(k.face)
            self.wire.append(k.wire)
            self.normal.append(k.normal)
        self.solid = Part.Solid()
        if len(flatagons) == 1:
            self.solid = self.face[0].extrude(self.normal[0])
            self.i.append(self.normal[0].Length)
        else:
            self.i.append((self.face[1].valueAt(0, 0) -self.face[0].valueAt(0, 0)).Length)
            for k in range(1, len(flatagons)):
                self.solid = self.solid.fuse(Part.makeLoft([self.wire[k-1], self.wire[k]], True, True))
                self.i.append((self.face[k].valueAt(0, 0) -self.face[k-1].valueAt(0, 0)).Length)

class BeSpin:
    def __init__(self, flatagon, angle, axis, pivot = FreeCAD.Vector(0, 0, 0)):
        self.flatagon = flatagon
        self.angle = angle
        self.axis = axis
        self.pivot = pivot
        
        self.solid = flatagon.face.revolve(self.pivot, self.axis, self.angle)

def fuseRotate(solid, angle, axis, pivot = FreeCAD.Vector(0, 0, 0)):
    solidCopy = solid.copy()
    return solid.fuse(solidCopy.rotate(pivot, axis, angle))

def fuseTranslate(solid, translation):
    solidCopy = solid.copy()
    return solid.fuse(solidCopy.translate(translation))

def fuseMirror(solid, axis, pivot = FreeCAD.Vector(0, 0, 0)):
    return solid.fuse(solid.mirror(pivot, axis))

pWashSink = {"radius": 3.80, "intensity": 1.00} #changed 4.00 to 3.80
pHexNut = {"radius": 3.50, "intensity": 2.30, "apothem": 3.50 *math.cos(math.pi/6)}
pFastSink0104 = {"radius": 2.64, "intensity": 2.40}
pFastShaft0104 = {"radius": 1.36, "intensity": 5.80}
pFastSink0308 = {"radius": 2.64, "intensity": 2.40}
pFastShaft0308 = {"radius": 1.36, "intensity": 9.24}
pLed0030Body = {"radius": 1.50, "intensity": 4.50}
pLed0030Base = {"radius": 1.90, "intensity": 1.00}
pBoardBoss = {"radius": 1.90, "intensity": 0.64}
pPowerCable = {"radius": 2.1}

#frame 124x136x22mm, offset y+13mm, offset z-4
bFrame = BeBox(FreeCAD.Vector(-62, -55, -15), FreeCAD.Vector(62, 81, 7), [FreeCAD.Vector(12, 12, 0)])
frameMass = bFrame.solid

#board
bBoard = BeBox(FreeCAD.Vector(-50, -50, 0), FreeCAD.Vector(50, 50, 1.6))
bBoardVoid = BeBox(bBoard.vWSD -f["close1"], bBoard.vENU +f["close1"])
bBoardClear = BeBox(bBoardVoid.vWSD, FreeCAD.Vector(bBoardVoid.e, bBoardVoid.n, bFrame.u))
frameMass = frameMass.cut(bBoardVoid.solid).cut(bBoardClear.solid)

#cavity
bCavity = BeBox(FreeCAD.Vector(bBoardVoid.w, bBoardVoid.s, bFrame.d) +t["nom"]\
    , FreeCAD.Vector(bBoardVoid.e -t["nom"].x, bFrame.n -(pWashSink["intensity"] +pHexNut["intensity"] +t["pro"].y), bFrame.u)\
    , [FreeCAD.Vector(4, 4, 0)])
frameMass = frameMass.cut(bCavity.solid)

#panel
bPanelW = BeBox(FreeCAD.Vector(bCavity.w +20, bCavity.n, bFrame.d), FreeCAD.Vector(bFrame.c, bFrame.n, bFrame.u))
bPanelE = BeBox(FreeCAD.Vector(bFrame.c, bCavity.n, bFrame.d), FreeCAD.Vector(bCavity.e -20, bFrame.n, bFrame.u))
roPanelWSD = FreeCAD.Placement(o, FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), Degree = -45), bPanelW.vWSD)
roPanelESD = FreeCAD.Placement(o, FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), Degree = 45), bPanelE.vESD)
dPanelVoid = BeDron([BeFlat([bPanelW.vWSD\
    , roPanelWSD.multVec(bPanelW.vESD)\
    , roPanelESD.multVec(bPanelE.vWSD)\
    , bPanelE.vESD\
    , bPanelE.vEND\
    , bPanelW.vWND\
    , bPanelW.vWSD]\
    , normal = FreeCAD.Vector(0, 0, bFrame.u -bFrame.d))])
lineSegmentA = Part.LineSegment(roPanelESD.multVec(bPanelE.vWND), roPanelESD.multVec(bPanelE.vEND))
lineSegmentB = Part.LineSegment(roPanelWSD.multVec(bPanelW.vWND), roPanelWSD.multVec(bPanelW.vEND))
lineSegmentC = Part.LineSegment(roPanelESD.multVec(bPanelE.vWSD) +FreeCAD.Vector(0, bPanelW.b, 0)\
    , roPanelWSD.multVec(bPanelW.vESD) +FreeCAD.Vector(0, bPanelW.b, 0))
intersectAC = lineSegmentA.intersect(lineSegmentC)[0]
intersectBC = lineSegmentB.intersect(lineSegmentC)[0]
dPanel = BeDron([BeFlat([bPanelW.vWSD\
    , roPanelWSD.multVec(bPanelW.vESD)\
    , roPanelESD.multVec(bPanelE.vWSD)\
    , bPanelE.vESD\
    , roPanelESD.multVec(bPanelE.vEND)\
    , FreeCAD.Vector(intersectAC.X, intersectAC.Y, intersectAC.Z)\
    , FreeCAD.Vector(intersectBC.X, intersectBC.Y, intersectBC.Z)\
    , roPanelWSD.multVec(bPanelW.vWND), bPanelW.vWSD]\
    , normal = FreeCAD.Vector(0, 0, bFrame.u -bFrame.d))])
sPanelElbowW = BeSpin(BeFlat([bPanelW.vWSD\
    , bPanelW.vWND\
    , bPanelW.vWNU\
    , bPanelW.vWSU\
    , bPanelW.vWSD])\
    , math.degrees(roPanelWSD.Rotation.Angle)\
    , FreeCAD.Vector(0, 0, 1), bPanelW.vWSD)
sPanelElbowE = BeSpin(BeFlat([bPanelE.vESD\
    , bPanelE.vEND\
    , bPanelE.vENU\
    , bPanelE.vESU\
    , bPanelE.vESD])\
    , math.degrees(roPanelESD.Rotation.Angle)\
    , FreeCAD.Vector(0, 0, 1), bPanelE.vESD)
aTerminalShaftWc = BeAgon(FreeCAD.Vector(bPanelW.w +(pWashSink["radius"] +t["pro"].x), bPanelW.n, bCavity.d +(pWashSink["radius"] +t["pro"].x))\
    , pFastShaft0308["radius"] +f["close1"].x\
    , FreeCAD.Vector(0, -bPanelW.b, 0))
dTerminalShaftWc = BeDron([aTerminalShaftWc])
dTerminalSinkWc = BeDron([BeAgon(aTerminalShaftWc.centre\
    , pWashSink["radius"] +f["tight1"].x\
    , FreeCAD.Vector(0, -pWashSink["intensity"], 0))])
dTerminalHexWc = BeDron([BeAgon(aTerminalShaftWc.centre +FreeCAD.Vector(0, -bPanelW.b, 0)\
    , pHexNut["radius"] +(f["tight1"].x /math.cos(math.pi/6))\
    , FreeCAD.Vector(0, pHexNut["intensity"], 0)\
    , 6)])
terminalMassWc = dTerminalShaftWc.solid.fuse(dTerminalSinkWc.solid).fuse(dTerminalHexWc.solid)
terminalMassW = fuseTranslate(terminalMassWc, FreeCAD.Vector((pWashSink["radius"] *2) +t["pro"].x, 0, 0))
terminalMassE = terminalMassW.mirror(o, FreeCAD.Vector(1, 0, 0))
terminalMassW.Placement = roPanelWSD
terminalMassE.Placement = roPanelESD
vShortIndW = FreeCAD.Vector(bPanelW.w -(pLed0030Base["radius"] +t["pro"].x), bCavity.n, aTerminalShaftWc.centre.z)
dShortIndBodyW = BeDron([BeAgon(vShortIndW, pLed0030Body["radius"] +f["tight0"].x, FreeCAD.Vector(0, bPanelW.b, 0))])
dShortIndBaseW = BeDron([BeAgon(vShortIndW, pLed0030Base["radius"] +f["tight0"].x, FreeCAD.Vector(0, pLed0030Base["intensity"], 0))])
shortIndMassW = dShortIndBodyW.solid.fuse(dShortIndBaseW.solid)
shortIndMass = fuseMirror(shortIndMassW, FreeCAD.Vector(1, 0, 0))
frameMass = frameMass.cut(dPanelVoid.solid).fuse(dPanel.solid).fuse(sPanelElbowW.solid).fuse(sPanelElbowE.solid).cut(terminalMassW).cut(terminalMassE).cut(shortIndMass)

#lid
bLidVoid = BeBox(FreeCAD.Vector(bFrame.w +t["pro"].x, bFrame.s +t["pro"].y, bBoard.u)\
    , FreeCAD.Vector(bFrame.e -t["pro"].x, bFrame.n -t["pro"].y, bFrame.u)\
    , [bFrame.j[0] -FreeCAD.Vector(t["pro"].x, t["pro"].y, 0)])
fLidVoidTrimW = BeFlat([FreeCAD.Vector(bLidVoid.w, -30, bLidVoid.d)\
    , FreeCAD.Vector(bBoard.w -t["pro"].x, -30 +(bBoard.w -bLidVoid.w -t["pro"].x /math.tan(math.pi/4)), bLidVoid.d)\
    , FreeCAD.Vector(bBoard.w -t["pro"].x, 34 -(bBoard.w -bLidVoid.w -t["pro"].x /math.tan(math.pi/4)), bLidVoid.d)\
    , FreeCAD.Vector(bLidVoid.w, 34, bLidVoid.d)\
    , FreeCAD.Vector(bLidVoid.w, -30, bLidVoid.d)]\
    , normal = FreeCAD.Vector(0, 0, bLidVoid.h))
dLidVoidTrimW = BeDron([fLidVoidTrimW])
lidVoidTrimMass = fuseMirror(dLidVoidTrimW.solid, FreeCAD.Vector(1, 0, 0))
bLid = BeBox(bLidVoid.vWSD +f["close1"]\
    , bLidVoid.vENU +FreeCAD.Vector(-f["close1"].x, -f["close1"].y, 0)\
    , [bLidVoid.j[0] -FreeCAD.Vector(f["close1"].x, f["close1"].y, 0)])
fLidTrimW = BeFlat([fLidVoidTrimW.v[0] +FreeCAD.Vector(f["close1"].x, -f["close1"].x *math.tan(math.pi/8), 0)\
    , fLidVoidTrimW.v[1] +FreeCAD.Vector(f["close1"].x, -f["close1"].x *math.tan(math.pi/8), 0)\
    , fLidVoidTrimW.v[2] +FreeCAD.Vector(f["close1"].x, f["close1"].x *math.tan(math.pi/8), 0)\
    , fLidVoidTrimW.v[3] +FreeCAD.Vector(f["close1"].x, f["close1"].x *math.tan(math.pi/8), 0)\
    , fLidVoidTrimW.v[4] +FreeCAD.Vector(f["close1"].x, -f["close1"].x *math.tan(math.pi/8), 0)]\
    , normal = FreeCAD.Vector(0, 0, bLid.u -bLidVoid.d))
dLidTrimW = BeDron([fLidTrimW])
lidTrimMass = fuseMirror(dLidTrimW.solid, FreeCAD.Vector(1, 0, 0))
bLidCavity = BeBox(FreeCAD.Vector(bBoardVoid.w +t["nom"].x, bBoardVoid.s +t["nom"].y, bLid.d)\
    , FreeCAD.Vector(bBoardVoid.e -t["nom"].x, bBoardVoid.n -t["nom"].y, bLid.u -t["nom"].z)\
    , bCavity.j)
dPowerCable = BeDron([BeAgon(FreeCAD.Vector(0, bLidCavity.n, bBoard.u), pPowerCable["radius"], FreeCAD.Vector(0, bLid.n -bLidCavity.n, 0))])
lidVoidMass = bLidVoid.solid.cut(lidVoidTrimMass)
lidMass = bLid.solid.cut(bLidCavity.solid).cut(lidTrimMass).cut(dPowerCable.solid)
frameMass = frameMass.cut(lidVoidMass).cut(dPowerCable.solid)
if intensityInd:
    intensityIndS = -26
    intensityIndN = 30
    bIntensityInd = BeBox(FreeCAD.Vector(bLidCavity.w, intensityIndS -(pLed0030Body["radius"] +t["nom"].y), bLidCavity.u -t["pro"].z)\
        , FreeCAD.Vector(-40 +(pLed0030Body["radius"] +t["nom"].x), intensityIndN +(pLed0030Body["radius"] +t["nom"].y), bLidCavity.u)\
        , [FreeCAD.Vector(pLed0030Body["radius"], -pLed0030Body["radius"], 0)\
        , FreeCAD.Vector(-pLed0030Body["radius"], pLed0030Body["radius"], 0)\
        , FreeCAD.Vector(-pLed0030Body["radius"], -pLed0030Body["radius"], 0)\
        , FreeCAD.Vector(pLed0030Body["radius"], pLed0030Body["radius"], 0)])
    lidMass = lidMass.fuse(bIntensityInd.solid)
    for y in range(intensityIndS, intensityIndN +4, 4):
        aIntensityInd = BeAgon(FreeCAD.Vector(-40, y, bBoard.u)\
            , pLed0030Body["radius"] +f["tight1"].z\
            , FreeCAD.Vector(0, 0, pLed0030Base["intensity"] +pLed0030Body["intensity"] +f["close1"].z))
        dIntensityInd = BeDron([BeAgon(aIntensityInd.centre, aIntensityInd.radius, FreeCAD.Vector(0, 0, 1))\
            , BeAgon(aIntensityInd.centre +FreeCAD.Vector(0, 0, aIntensityInd.normal.z *0.8), aIntensityInd.radius, FreeCAD.Vector(0, 0, 1))\
            , BeAgon(aIntensityInd.centre +FreeCAD.Vector(0, 0, max(aIntensityInd.normal.z, bLid.u -bBoard.u)), aIntensityInd.radius *0.8, FreeCAD.Vector(0, 0, 1))])
        lidMass = lidMass.cut(dIntensityInd.solid)
if directionInd:
    directionIndS = -25
    directionIndN = 25
    for y in [directionIndS, directionIndN]:
        bDirectionInd = BeBox(FreeCAD.Vector(40 -(pLed0030Body["radius"] +t["nom"].x), y -(pLed0030Body["radius"] +t["nom"].y), bLidCavity.u -t["pro"].z)\
            , FreeCAD.Vector(bLidCavity.e, y +(pLed0030Body["radius"] +t["nom"].y), bLidCavity.u)\
            , [FreeCAD.Vector(pLed0030Body["radius"], pLed0030Body["radius"], 0)\
            , FreeCAD.Vector(-pLed0030Body["radius"], -pLed0030Body["radius"], 0)\
            , FreeCAD.Vector(-pLed0030Body["radius"], pLed0030Body["radius"], 0)\
            , FreeCAD.Vector(pLed0030Body["radius"], -pLed0030Body["radius"], 0)])
        lidMass = lidMass.fuse(bDirectionInd.solid)
        aDirectionInd = BeAgon(FreeCAD.Vector(40, y, bBoard.u)\
            , pLed0030Body["radius"] +f["tight1"].z\
            , FreeCAD.Vector(0, 0, pLed0030Base["intensity"] +pLed0030Body["intensity"] +f["close1"].z))
        dDirectionInd = BeDron([BeAgon(aDirectionInd.centre, aDirectionInd.radius, FreeCAD.Vector(0, 0, 1))\
            , BeAgon(aDirectionInd.centre +FreeCAD.Vector(0, 0, aDirectionInd.normal.z *0.8), aDirectionInd.radius, FreeCAD.Vector(0, 0, 1))\
            , BeAgon(aDirectionInd.centre +FreeCAD.Vector(0, 0, max(aDirectionInd.normal.z, bLid.u -bBoard.u)), aDirectionInd.radius *0.8, FreeCAD.Vector(0, 0, 1))])
        lidMass = lidMass.cut(dDirectionInd.solid)

#button
if resetButton or directionButton:
    bButtonBaseCavity = BeBox(FreeCAD.Vector(bBoard.c -(3.00 +f["close0"].x), bBoard.m -(3.00 +f["close0"].y), bBoard.u +f["loose0"].z)\
        , FreeCAD.Vector(bBoard.c +(3.00 +f["close0"].x), bBoard.m +(3.00 +f["close0"].y), bBoard.u +(5.7 +f["loose0"].z)))
    bButtonShaftCavity = BeBox(FreeCAD.Vector(bBoard.c -(1.40 +f["tight0"].x), bBoard.m -(1.40 +f["tight0"].y), bBoard.u +5.7)\
        , FreeCAD.Vector(bBoard.c +(1.40 +f["tight0"].x), bBoard.m +(1.40 +f["tight0"].y), bBoard.u +(7.5 +f["tight0"].z)))
    bButtonBase = BeBox(FreeCAD.Vector(bButtonBaseCavity.c -6.4, bButtonBaseCavity.m -6.4, bButtonBaseCavity.d)\
        , FreeCAD.Vector(bButtonBaseCavity.c +6.4, bButtonBaseCavity.m +6.4, bButtonShaftCavity.u +t["pro"].z)\
        , [FreeCAD.Vector(2, 2, 0)])
    bButtonHead = BeBox(FreeCAD.Vector(bButtonBase.c -20, bButtonBase.m -15, bFrame.u +t["lrg"].z)\
        , FreeCAD.Vector(bButtonBase.c +20, bButtonBase.m +15, bButtonBase.u)\
        , [FreeCAD.Vector(8, 8, 0)])
    dButtonBaseHexS = BeDron([BeAgon(FreeCAD.Vector(bButtonBase.c, -8, bButtonBase.d)\
        , pHexNut["radius"] +((f["tight1"].x +t["min"].x +f["loose0"].x) /math.cos(math.pi/6))\
        , FreeCAD.Vector(0, 0, bButtonBase.h)\
        , 6)])
    dButtonBaseHexN = BeDron([BeAgon(FreeCAD.Vector(bButtonBase.c, +8, bButtonBase.d)\
        , pHexNut["radius"] +((f["tight1"].x +t["min"].x +f["loose0"].x) /math.cos(math.pi/6))\
        , FreeCAD.Vector(0, 0, bButtonBase.h)\
        , 6)])
    bButtonBaseHexTrim = BeBox(FreeCAD.Vector(bButtonBase.w, bButtonBaseCavity.s -t["min"].y, bButtonBase.d)\
        , FreeCAD.Vector(bButtonBase.e, bButtonBaseCavity.n +t["min"].y, bButtonBase.u))
    buttonBaseHexMass = dButtonBaseHexS.solid.fuse(dButtonBaseHexN.solid).cut(bButtonBaseHexTrim.solid)
    buttonMass = bButtonBase.solid.cut(buttonBaseHexMass).fuse(bButtonHead.solid).cut(bButtonBaseCavity.solid).cut(bButtonShaftCavity.solid)
    for y in [bButtonBaseCavity.s +1.2, bButtonBaseCavity.n -1.2]:
        for x in [bButtonBaseCavity.w, bButtonBaseCavity.e]:
            dButtonPinCavity = BeDron([BeAgon(FreeCAD.Vector(x, y, bButtonBaseCavity.d), 1.2, FreeCAD.Vector(0, 0, 2.4))])
            buttonMass = buttonMass.cut(dButtonPinCavity.solid)

#rotaryDial
if rotaryEnc:
    shaftSections = []
    shaftSections.extend([BeAgon(FreeCAD.Vector(0, 0, bFrame.u), 13.0 +f["loose0"].x, FreeCAD.Vector(0, 0, 1))\
        , BeAgon(FreeCAD.Vector(0, 0, bBoard.u +10.0 +f["loose0"].z), 13.0 +f["loose0"].x, FreeCAD.Vector(0, 0, 1))\
        , BeAgon(FreeCAD.Vector(0, 0, bBoard.u +10.0 +f["loose0"].z), 4.5 +f["loose0"].x, FreeCAD.Vector(0, 0, 1))\
        , BeAgon(FreeCAD.Vector(0, 0, bBoard.u +8.0 +7.0 +f["loose0"].z), 4.5 +f["loose0"].x, FreeCAD.Vector(0, 0, 1))
        , BeAgon(FreeCAD.Vector(0, 0, bBoard.u +8.0 +7.0 +f["loose0"].z), 3.0 +f["tight1"].x, FreeCAD.Vector(0, 0, 1))\
        , BeAgon(FreeCAD.Vector(0, 0, bBoard.u +8.0 +20.0 +f["tight1"].z), 3.0 +f["tight1"].x, FreeCAD.Vector(0, 0, 1))])
    dDialShaft = BeDron(shaftSections)
    bDialShaftFlat = BeBox(FreeCAD.Vector(-shaftSections[-1].radius, 1.5 +f["tight0"].y, bBoard.u +8.0 +7.0 +1.0 +f["loose0"].z)\
        , FreeCAD.Vector(shaftSections[-1].radius, shaftSections[-1].radius, shaftSections[-1].centre.z))
    dialShaftMass = dDialShaft.solid.cut(bDialShaftFlat.solid)
    dDial = BeDron([BeAgon(FreeCAD.Vector(0, 0, bFrame.u +t["lrg"].z), 30, FreeCAD.Vector(0, 0, 1), 12)\
        , BeAgon(FreeCAD.Vector(0, 0, shaftSections[-1].centre.z +t["pro"].z -4), 30, FreeCAD.Vector(0, 0, 1), 12)\
        , BeAgon(FreeCAD.Vector(0, 0, shaftSections[-1].centre.z +t["pro"].z), 26, FreeCAD.Vector(0, 0, 1), 24)])
    dialMass = dDial.solid.cut(dialShaftMass)
    
#post
bPostLidSupport = BeBox(FreeCAD.Vector(roPanelWSD.multVec(bPanelW.vESD).x, bCavity.s, bCavity.d)\
        , FreeCAD.Vector(roPanelESD.multVec(bPanelE.vWSD).x, roPanelESD.multVec(bPanelE.vWSD).y, bCavity.d +t["pro"].z)\
        , [FreeCAD.Vector(-bCavity.j[0].x, bCavity.j[0].y, 0)\
        , FreeCAD.Vector(bCavity.j[0].x, bCavity.j[0].y, 0)\
        , FreeCAD.Vector(0, 0, 0)\
        , FreeCAD.Vector(0, 0, 0)])
frameMass = frameMass.fuse(bPostLidSupport.solid)
for y in [-33, -17, 17, 33]:
    dPostLid = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bLidCavity.u), pHexNut["radius"] +f["tight1"].x +t["nom"].x, FreeCAD.Vector(0, 0, -1))\
        , BeAgon(FreeCAD.Vector(bBoard.c, y, bBoard.u +f["close0"].z), pHexNut["radius"] +f["tight1"].x +t["min"].x, FreeCAD.Vector(0, 0, -1))])
    dPostFrame = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bCavity.d), pFastSink0308["radius"] +f["close1"].x +t["nom"].x, FreeCAD.Vector(0, 0, 1))\
        , BeAgon(FreeCAD.Vector(bBoard.c, y, bBoardVoid.d), pFastSink0308["radius"] +f["close1"].x +t["min"].x, FreeCAD.Vector(0, 0, 1))])
    lidMass = lidMass.fuse(dPostLid.solid)
    frameMass = frameMass.fuse(dPostFrame.solid)
for y in [-33, 33]:
    dPostLidBoss = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bBoard.u +f["close0"].z)\
        , pBoardBoss["radius"] -f["tight0"].x\
        , FreeCAD.Vector(0, 0, -pBoardBoss["intensity"]))])
    dPostFrameBoss = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bBoard.d -f["close0"].z)\
        , pBoardBoss["radius"] -f["tight0"].x\
        , FreeCAD.Vector(0, 0, pBoardBoss["intensity"]))])
    lidMass = lidMass.fuse(dPostLidBoss.solid)
    frameMass = frameMass.fuse(dPostFrameBoss.solid)

#lidButton
buttons = []
if resetButton:
    buttons.append(0)
if directionButton:
    buttons.extend([-25, 25])
if len(buttons) > 0:
    bLidButton = BeBox(FreeCAD.Vector(bButtonBase.w -f["loose0"].x -t["nom"].x, bLidCavity.s, bLidCavity.u -t["pro"].z)\
        , FreeCAD.Vector(bButtonBase.e +f["loose0"].x +t["nom"].x, bLidCavity.n, bLidCavity.u)\
        , [FreeCAD.Vector(-bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(-bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)])
    lidMass = lidMass.fuse(bLidButton.solid)
    for y in buttons:
        bLidButtonVoid = BeBox(FreeCAD.Vector(bButtonBase.w -f["loose0"].x, y -((bButtonBase.b /2) +f["loose0"].y), bLid.d)\
            , FreeCAD.Vector(bButtonBase.e +f["loose0"].x, y +((bButtonBase.b /2) +f["loose0"].y), bLid.u)\
            , bButtonBase.j)
        dLidButtonVoidHexS = BeDron([BeAgon(FreeCAD.Vector(bButtonBase.c, y -8, bLid.d)\
            , pHexNut["radius"] +((f["tight1"].x +t["min"].x) /math.cos(math.pi/6))\
            , FreeCAD.Vector(0, 0, bLid.h)\
            , 6)])
        dLidButtonVoidHexN = BeDron([BeAgon(FreeCAD.Vector(bButtonBase.c, y +8, bLid.d)\
            , pHexNut["radius"] +((f["tight1"].x +t["min"].x) /math.cos(math.pi/6))\
            , FreeCAD.Vector(0, 0, bLid.h)\
            , 6)])
        bLidButtonVoidHexTrim = BeBox(FreeCAD.Vector(bButtonBase.w, y -8 +pHexNut["apothem"] +f["tight1"].y, bLid.d)\
            , FreeCAD.Vector(bButtonBase.e, y +8 -(pHexNut["apothem"] +f["tight1"].y), bLid.u))
        lidButtonVoidHexMass = dLidButtonVoidHexS.solid.fuse(dLidButtonVoidHexN.solid).cut(bLidButtonVoidHexTrim.solid)
        lidButtonVoidMass = bLidButtonVoid.solid.cut(lidButtonVoidHexMass)
        lidMass = lidMass.cut(lidButtonVoidMass)

#lidRotary
if rotaryEnc:
    bLidButtonSupport = BeBox(FreeCAD.Vector(-8.0 -f["loose0"].x -t["nom"].x, bLidCavity.s, bLidCavity.u -t["pro"].z)\
        , FreeCAD.Vector(8.0 +f["loose0"].x +t["nom"].x, bLidCavity.n, bLidCavity.u)\
        , [FreeCAD.Vector(-bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(-bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)])
    bLidRotaryVoid = BeBox(FreeCAD.Vector(-8.0 -f["loose0"].x, -8.3 -f["loose0"].y, bBoard.u)\
        , FreeCAD.Vector(8.0 +f["loose0"].x, 9.3 +f["loose0"].y, bLid.u)\
        , [FreeCAD.Vector(1, 1, 0)\
        , FreeCAD.Vector(-1, 1, 0)\
        , FreeCAD.Vector(-2, -2, 0)\
        , FreeCAD.Vector(2, -2, 0)])
    bLidRotaryTipVoid = BeBox(FreeCAD.Vector(-2.0 -f["loose0"].x, bLidRotaryVoid.n, bLidRotaryVoid.d)\
        , FreeCAD.Vector(2.0 +f["loose0"].x, 12.5 +f["loose0"].y, bLidRotaryVoid.u)\
        , [FreeCAD.Vector(-1, 1, 0)\
        , FreeCAD.Vector(1, 1, 0)\
        , FreeCAD.Vector(0, 0, 0)\
        , FreeCAD.Vector(0, 0, 0)])
    bLidButtonSupport = BeBox(FreeCAD.Vector(bLidRotaryVoid.w -t["nom"].x, bLidCavity.s, bLidCavity.u -t["pro"].z)\
        , FreeCAD.Vector(bLidRotaryVoid.e +t["nom"].x, bLidCavity.n, bLidCavity.u)\
        , [FreeCAD.Vector(-bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)\
        , FreeCAD.Vector(-bLidCavity.j[0].x, -bLidCavity.j[0].y, 0)])
    lidMass = lidMass.fuse(bLidButtonSupport.solid).cut(bLidRotaryVoid.solid).cut(bLidRotaryTipVoid.solid)

#postVoid
for y in [-17, 17]:
    dPostLidHex = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bLid.u)\
        , pHexNut["radius"] +(f["tight1"].x /math.cos(math.pi/6))\
        , FreeCAD.Vector(0, 0, -(pHexNut["intensity"] +f["loose0"].z))\
        , 6)])
    dPostShaft = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bLid.u -(pHexNut["intensity"] +f["loose0"].z +t["nom"].z))\
        , pFastShaft0308["radius"] +f["close1"].x\
        , FreeCAD.Vector(0, 0, -(pFastShaft0308["intensity"] -(pHexNut["intensity"] +f["loose0"].z +(bLidVoid.h -bLid.h) +(t["nom"].z *2)))))])
    dPostSink = BeDron([BeAgon(FreeCAD.Vector(bBoard.c, y, bLid.u -pFastShaft0308["intensity"])\
        , pFastSink0308["radius"] +f["close1"].x\
        , FreeCAD.Vector(0, 0, -(bLid.u -pFastShaft0308["intensity"] -bFrame.d)))])
    lidMass = lidMass.cut(dPostLidHex.solid).cut(dPostShaft.solid)
    frameMass = frameMass.cut(dPostShaft.solid).cut(dPostSink.solid)

#secure
for y in [bLid.s +bLid.j[0].y +(pHexNut["radius"] +f["close0"].y), bLid.n +bLid.j[3].y -(pHexNut["radius"] +f["close0"].y)]:
    aSecureLidHex = BeAgon(FreeCAD.Vector(bLid.w +t["min"].x +pHexNut["radius"] +(f["close0"].x /math.cos(math.pi/6)), y, bLid.d +t["pro"].z)\
    , pHexNut["radius"] +(f["close0"].x /math.cos(math.pi/6))\
    , FreeCAD.Vector(0, 0, pHexNut["intensity"] +f["tight1"].z)\
    , 6)
    dSecureLidHex = BeDron([aSecureLidHex])
    bSecureLidHex = BeBox(FreeCAD.Vector(bLid.w, y -aSecureLidHex.apothem, aSecureLidHex.centre.z)\
        ,FreeCAD.Vector(aSecureLidHex.centre.x, y +aSecureLidHex.apothem, aSecureLidHex.centre.z +aSecureLidHex.normal.z))
    dSecureShaft = BeDron([BeAgon(FreeCAD.Vector(aSecureLidHex.centre.x, y, aSecureLidHex.centre.z -t["nom"].z)\
        , pFastShaft0308["radius"] +f["close1"].x\
        , FreeCAD.Vector(0, 0, -(pFastShaft0308["intensity"] +f["loose0"].z -(pHexNut["intensity"] +f["tight1"].z +(bLidVoid.h -bLid.h) +(t["nom"].z *2)))))])
    dSecureSink = BeDron([BeAgon(FreeCAD.Vector(aSecureLidHex.centre.x, y, bSecureLidHex.u -(pFastShaft0308["intensity"] +f["loose0"].z))\
        , pFastSink0308["radius"] +f["close0"].x\
        , FreeCAD.Vector(0, 0, -(bSecureLidHex.u -(pFastShaft0308["intensity"] +f["loose0"].z) -bFrame.d)))])
    secureMassW = (dSecureLidHex.solid).fuse(bSecureLidHex.solid).fuse(dSecureShaft.solid).fuse(dSecureSink.solid)
    secureMass = fuseMirror(secureMassW, FreeCAD.Vector(1, 0, 0))
    lidMass = lidMass.cut(secureMass)
    frameMass = frameMass.cut(secureMass)

#airVents
for y in [42, 36, 30, 24, 18, 12]:
    bAirVentE = BeBox(FreeCAD.Vector(bCavity.e -(bBoardVoid.d -bCavity.d -(t["pro"].z *2)), y -0.8, bCavity.d +t["pro"].z)\
        , FreeCAD.Vector(bCavity.e, y +0.8, bCavity.d +t["pro"].z +(bFrame.e -bCavity.e))\
        , [FreeCAD.Vector(0.8, 0.8, 0), FreeCAD.Vector(-0.8, 0.8, 0), FreeCAD.Vector(-0.8, -0.8, 0), FreeCAD.Vector(0.8, -0.8, 0)])
    bAirVentE.solid.rotate(FreeCAD.Vector(bCavity.e, y, bCavity.d +t["pro"].z), FreeCAD.Vector(0, 1, 0), 90)
    airVentMass = fuseMirror(bAirVentE.solid, FreeCAD.Vector(1, 0, 0))
    frameMass = frameMass.cut(airVentMass)
for x in [-40, -34, -28, -22, -16, -10, 10, 16, 22, 28, 34, 40]:
    bAirVentS = BeBox(FreeCAD.Vector(x -0.8, bFrame.s, bCavity.d +t["pro"].z)\
        , FreeCAD.Vector(x +0.8, bFrame.s +(bBoardVoid.d -bCavity.d -(t["pro"].z *2)), bCavity.d +t["pro"].z +(bFrame.s -bCavity.s))\
        , [FreeCAD.Vector(0.8, 0.8, 0), FreeCAD.Vector(-0.8, 0.8, 0), FreeCAD.Vector(-0.8, -0.8, 0), FreeCAD.Vector(0.8, -0.8, 0)])
    bAirVentS.solid.rotate(FreeCAD.Vector(x, bFrame.s, bCavity.d +t["pro"].z), FreeCAD.Vector(1, 0, 0), 90)
    frameMass = frameMass.cut(bAirVentS.solid)

#frameDesign
fFrameTrimW = BeFlat([FreeCAD.Vector(fLidVoidTrimW.v[0].x -t["pro"].x, fLidVoidTrimW.v[0].y +t["pro"].x *math.tan(math.pi/8), bFrame.d)\
    , FreeCAD.Vector(fLidVoidTrimW.v[1].x -t["pro"].x, fLidVoidTrimW.v[1].y +t["pro"].x *math.tan(math.pi/8), bFrame.d)\
    , FreeCAD.Vector(fLidVoidTrimW.v[2].x -t["pro"].x, fLidVoidTrimW.v[2].y -t["pro"].x *math.tan(math.pi/8), bFrame.d)\
    , FreeCAD.Vector(fLidVoidTrimW.v[3].x -t["pro"].x, fLidVoidTrimW.v[3].y -t["pro"].x *math.tan(math.pi/8), bFrame.d)\
    , FreeCAD.Vector(fLidVoidTrimW.v[4].x -t["pro"].x, fLidVoidTrimW.v[4].y +t["pro"].x *math.tan(math.pi/8), bFrame.d)]\
    , normal = FreeCAD.Vector(0, 0, bFrame.h))
dFrameTrimW = BeDron([fFrameTrimW])
frameTrimMass = fuseMirror(dFrameTrimW.solid, FreeCAD.Vector(1, 0, 0))
frameMass = frameMass.cut(frameTrimMass)

frameFeature = Part.show(frameMass, 'frame')
frameFeature.ViewObject.Transparency = 50
frameFeature.ViewObject.ShapeColor = (0.20, 0.60, 0.80)

lidFeature = Part.show(lidMass, 'lid')
lidFeature.ViewObject.Transparency = 50
lidFeature.ViewObject.ShapeColor = (0.80, 0.20, 0.60)

if resetButton or directionButton:
    buttonFeature = Part.show(buttonMass, 'button')
    buttonFeature.ViewObject.Transparency = 50
    buttonFeature.ViewObject.ShapeColor = (0.60, 0.80, 0.20)

if rotaryEnc:
    dialFeature = Part.show(dialMass, 'dial')
    dialFeature.ViewObject.Transparency = 50
    dialFeature.ViewObject.ShapeColor = (0.60, 0.80, 0.20)

FreeCAD.Gui.activeDocument().activeView().viewIsometric()
FreeCAD.Gui.SendMsgToActiveView("ViewFit")

#changelog
# -added fuseRotate, fuseTranslate, and fuseMirror functions
# -adjusted relevant code to call new functions
# -reordered BeFuse attributes to better align with new fuse fuctions
# -renamed certain variable names to properly place suffixes
# -renamed certain variable names to reflect object type (e.g. dTerminalMass -> terminalMass since it is not a dron object)
# -added line breaks where appropriate for readability