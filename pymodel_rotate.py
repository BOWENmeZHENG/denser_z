from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

# Constants
r_out = 0.3
r_in = 0.2
width = 0.1
spoke_width = 0.04
num_spokes = 4
meshsize = 0.02
r_depth = 0.02
r_pressure = 0.1
E = 1e8
mu = 0.3
load = 10000
init_angle = 0
results_location = 'C:/Users/bowen/Desktop/'

# Names
part_name = 'wheel'
material_name = 'wheel_material'
section_name = 'wheel_section'
assembly_name = 'wheel-assembly'
step_name = 'static_load'
load_name = 'compression'
bc_name = 'fixed'
job_name = 'wheel_compression'

# Derived values
search_point_whole = (0.0, r_out, width / 2)
search_point_lateral = (0.0, r_out, width / 2)
search_point_extrusion = (0.0, (r_in + r_out) / 2, width)
search_point_outer_edge = (0.0, r_out, width)

spoke_start = (r_out + r_in) / 2
search_points_spoke = [(-spoke_start + 0.01, spoke_width / 2),
                       (-spoke_start + 0.01, -spoke_width / 2),
                       (-spoke_start, 0),
                       (spoke_start, 0)]

rotate_angle = 180 / num_spokes

# Define wheel geometry
mymodel = mdb.models['Model-1']
mymodel.ConstrainedSketch(name='__profile__', sheetSize=r_out * 2)
mymodel.sketches['__profile__'].CircleByCenterPerimeter(center=(0.0, 0.0), point1=(r_out, 0.0))
mymodel.sketches['__profile__'].CircleByCenterPerimeter(center=(0.0, 0.0), point1=(r_in, 0.0))
mymodel.Part(dimensionality=THREE_D, name=part_name, type=DEFORMABLE_BODY)

mypart = mymodel.parts[part_name]
mypart.BaseSolidExtrude(depth=width, sketch=mymodel.sketches['__profile__'])
del mymodel.sketches['__profile__']

for i in range(num_spokes):
    face_base = mypart.faces.findAt((search_point_extrusion,), )[0]
    edge_extrusion = mypart.edges.findAt((search_point_outer_edge,), )[0]
    mymodel.ConstrainedSketch(gridSpacing=0.04, name='__profile__', sheetSize=1.7,
                              transform=mypart.MakeSketchTransform(
                                  sketchPlane=face_base, sketchPlaneSide=SIDE1, sketchUpEdge=edge_extrusion,
                                  sketchOrientation=RIGHT, origin=(0.0, 0.0, width)))
    mysketch = mymodel.sketches['__profile__']
    mypart.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=mysketch)
    mysketch.rectangle(point1=(-spoke_start, -spoke_width / 2), point2=(spoke_start, spoke_width / 2))
    mysketch.rotate(angle=init_angle+rotate_angle*(i), centerPoint=(0.0, 0.0),
                    objectList=(
                        mysketch.geometry.findAt(search_points_spoke[0], ),
                        mysketch.geometry.findAt(search_points_spoke[1], ),
                        mysketch.geometry.findAt(search_points_spoke[2], ),
                        mysketch.geometry.findAt(search_points_spoke[3], )))
    mypart.SolidExtrude(depth=width, flipExtrudeDirection=ON, sketch=mysketch, sketchOrientation=RIGHT,
                        sketchPlane=face_base, sketchPlaneSide=SIDE1, sketchUpEdge=edge_extrusion)
    del mysketch

mypart.Set(faces=mypart.faces.getByBoundingSphere(center=(0, 0, 0), radius=10.0),
           name='all_faces')  # set for exterior nodes

# Material & Section
mymodel.Material(name=material_name)
mymodel.materials[material_name].Elastic(table=((E, mu), ))
mymodel.HomogeneousSolidSection(material=material_name, name=section_name, thickness=None)
mypart.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE,
                         region=Region(cells=mypart.cells.findAt((search_point_whole,), )),
                         sectionName=section_name, thicknessAssignment=FROM_SECTION)

# Assembly
mymodel.rootAssembly.DatumCsysByDefault(CARTESIAN)
mymodel.rootAssembly.Instance(dependent=ON, name=assembly_name, part=mypart)
myassembly = mymodel.rootAssembly.instances[assembly_name]

# Step
mymodel.StaticStep(name=step_name, previous='Initial')

# Mesh
mypart.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=meshsize)
mypart.setMeshControls(elemShape=TET, regions=mypart.cells.findAt((search_point_whole,), ), technique=FREE)
mypart.setElementType(elemTypes=(ElemType(elemCode=C3D8R, elemLibrary=STANDARD),
                                 ElemType(elemCode=C3D6, elemLibrary=STANDARD),
                                 ElemType(elemCode=C3D4, elemLibrary=STANDARD,
                                          secondOrderAccuracy=OFF, distortionControl=DEFAULT)),
                      regions=(mypart.cells.findAt(((0.0, r_out, width / 2),), ),))
mypart.generateMesh()

# get nodes for loading and BC
mypart.Set(faces=mypart.faces.findAt((search_point_lateral,), ), name='face_big')
face_big = mypart.sets['face_big'].faces[0]
mypart.Set(nodes=face_big.getNodes(), name='face_nodes')
face_big_nodes = mypart.sets['face_nodes'].nodes
mypart.Set(nodes=face_big_nodes.getByBoundingCylinder(center1=(0.0, r_out - r_depth, width / 2),
                                                      center2=(0.0, r_out + r_depth, width / 2),
                                                      radius=r_pressure), name='nodes_load')
nodes_load = mypart.sets['nodes_load'].nodes
mypart.Set(nodes=face_big_nodes.getByBoundingCylinder(center1=(0.0, -(r_out - r_depth), width / 2),
                                                      center2=(0.0, -(r_out + r_depth), width / 2),
                                                      radius=r_pressure), name='nodes_bc')
nodes_bc = mypart.sets['nodes_bc'].nodes

# Load & BC
num_nodes_load = len(nodes_load)
mymodel.ConcentratedForce(cf2=-load/num_nodes_load, createStepName=step_name,
                          distributionType=UNIFORM, field='', localCsys=None, name=load_name,
                          region=myassembly.sets['nodes_load'])
mymodel.EncastreBC(createStepName=step_name, localCsys=None, name=bc_name, region=myassembly.sets['nodes_bc'])

# Job
mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, explicitPrecision=SINGLE,
        getMemoryFromAnalysis=True, historyPrint=OFF, memory=90, memoryUnits=PERCENTAGE,
        model='Model-1', modelPrint=OFF, multiprocessingMode=DEFAULT, name=job_name,
        nodalOutputPrecision=SINGLE, numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='',
        type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
mdb.jobs[job_name].submit(consistencyChecking=OFF)

# Access results
odb_name = job_name + '.odb'
odb = openOdb(path=odb_name, readOnly=True)
odb_assembly = odb.rootAssembly
odb_instance = odb_assembly.instances.keys()[0]
odb_step1 = odb.steps.values()[0]
frame = odb.steps[odb_step1.name].frames[-1]
elemStress = frame.fieldOutputs['S']
elemDisp = frame.fieldOutputs['U']  # new
odb_set_whole = odb_assembly.elementSets[' ALL ELEMENTS']
field = elemStress.getSubset(region=odb_set_whole, position=ELEMENT_NODAL)
field_disp = elemDisp.getSubset(region=odb_set_whole, position=ELEMENT_NODAL)  # new
nodalS11 = {}
for value in field.values:
    if value.nodeLabel in nodalS11:
        nodalS11[value.nodeLabel].append(value.data[0])
    else:
        nodalS11.update({value.nodeLabel: [value.data[0]]})
for key in nodalS11:
    nodalS11.update({key: sum(nodalS11[key]) / len(nodalS11[key])})
max_value = max(nodalS11.values())
print(nodalS11.values()[0])

nodal_disp = {}
for value in elemDisp.values:
    nodal_disp.update({value.nodeLabel: value.data[0]})
print(nodal_disp.values()[0], len(nodal_disp.values()))