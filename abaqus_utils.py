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


def derived_values(r_in, r_out, width, spoke_width):
    s_pt_whole = (0.0, r_out, width / 2)
    # s_pt_lateral = (0.0, r_out, width / 2)
    s_pt_load = (0.0, r_out, width / 2)
    s_pt_bc = (0.0, -r_out, width / 2)
    s_pt_extr = (0.0, (r_in + r_out) / 2, width)
    s_pt_out_edge = (0.0, r_out, width)
    s_pt_mesh_edge = (r_out, 0.0, width / 2)
    spoke_start = (r_out + r_in) / 2
    s_pts_spoke = [(-spoke_start + 0.01, spoke_width / 2),
                   (-spoke_start + 0.01, -spoke_width / 2),
                   (-spoke_start, 0),
                   (spoke_start, 0)]
    return s_pt_whole, s_pt_load, s_pt_bc, s_pt_extr, s_pt_out_edge, spoke_start, s_pts_spoke, s_pt_mesh_edge


def init_part(mymodel, r_out, r_in, width, part_name):
    mymodel.ConstrainedSketch(name='__profile__', sheetSize=r_out * 2)
    mymodel.sketches['__profile__'].CircleByCenterPerimeter(center=(0.0, 0.0), point1=(r_out, 0.0))
    mymodel.sketches['__profile__'].CircleByCenterPerimeter(center=(0.0, 0.0), point1=(r_in, 0.0))
    mymodel.Part(dimensionality=THREE_D, name=part_name, type=DEFORMABLE_BODY)
    mypart = mymodel.parts[part_name]
    mypart.BaseSolidExtrude(depth=width, sketch=mymodel.sketches['__profile__'])
    del mymodel.sketches['__profile__']
    return mypart


def spoke(mymodel, mypart, width, num_spokes, spoke_width, init_angle,
          spoke_start, s_pts_spoke, s_pt_extr, s_pt_out_edge):
    for i in range(num_spokes):
        face_base = mypart.faces.findAt((s_pt_extr,), )[0]
        edge_extrusion = mypart.edges.findAt((s_pt_out_edge,), )[0]
        mymodel.ConstrainedSketch(gridSpacing=0.04, name='__profile__', sheetSize=1.7,
                                  transform=mypart.MakeSketchTransform(
                                      sketchPlane=face_base, sketchPlaneSide=SIDE1, sketchUpEdge=edge_extrusion,
                                      sketchOrientation=RIGHT, origin=(0.0, 0.0, width)))
        mysketch = mymodel.sketches['__profile__']
        mypart.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=mysketch)
        mysketch.rectangle(point1=(-spoke_start, -spoke_width / 2), point2=(spoke_start, spoke_width / 2))
        mysketch.rotate(angle=180 / num_spokes * i + init_angle, centerPoint=(0.0, 0.0),
                        objectList=(
                            mysketch.geometry.findAt(s_pts_spoke[0], ),
                            mysketch.geometry.findAt(s_pts_spoke[1], ),
                            mysketch.geometry.findAt(s_pts_spoke[2], ),
                            mysketch.geometry.findAt(s_pts_spoke[3], )))
        mypart.SolidExtrude(depth=width, flipExtrudeDirection=ON, sketch=mysketch, sketchOrientation=RIGHT,
                            sketchPlane=face_base, sketchPlaneSide=SIDE1, sketchUpEdge=edge_extrusion)
        del mysketch


def mat_sect(mymodel, mypart, material_name, E, mu, section_name, s_pt_whole):
    mymodel.Material(name=material_name)
    mymodel.materials[material_name].Elastic(table=((E, mu),))
    mymodel.HomogeneousSolidSection(material=material_name, name=section_name, thickness=None)
    mypart.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE,
                             region=Region(cells=mypart.cells.findAt((s_pt_whole,), )),
                             sectionName=section_name, thicknessAssignment=FROM_SECTION)


def make_assembly(mymodel, mypart, assembly_name):
    mymodel.rootAssembly.DatumCsysByDefault(CARTESIAN)
    mymodel.rootAssembly.Instance(dependent=ON, name=assembly_name, part=mypart)
    myassembly = mymodel.rootAssembly.instances[assembly_name]
    return myassembly


def make_mesh(mypart, meshsize, z_density, s_pt_whole, s_pt_mesh_edge, r_out, width):
    # mypart.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=meshsize)
    # mypart.setMeshControls(elemShape=TET, regions=mypart.cells.findAt((s_pt_whole,), ), technique=FREE)
    # mypart.setElementType(elemTypes=(ElemType(elemCode=C3D8R, elemLibrary=STANDARD),
    #                                  ElemType(elemCode=C3D6, elemLibrary=STANDARD),
    #                                  ElemType(elemCode=C3D4, elemLibrary=STANDARD,
    #                                           secondOrderAccuracy=OFF, distortionControl=DEFAULT)),
    #                       regions=(mypart.cells.findAt(((0.0, r_out, width / 2),), ),))
    # mypart.generateMesh()
    mypart.setMeshControls(elemShape=TET, regions=mypart.cells.findAt((s_pt_whole,), ), technique=FREE)
    mypart.setElementType(elemTypes=(ElemType(elemCode=C3D8R, elemLibrary=STANDARD),
                                     ElemType(elemCode=C3D6, elemLibrary=STANDARD),
                                     ElemType(elemCode=C3D4, elemLibrary=STANDARD,
                                              secondOrderAccuracy=OFF, distortionControl=DEFAULT)),
                          regions=(mypart.cells.findAt(((0.0, r_out, width / 2),), ),))
    mypart.PartitionCellByPlaneThreePoints(cells=mypart.cells.findAt((s_pt_whole,), ),
                                           point1=(0, 0, 0), point2=(1, 0, 0), point3=(1, 0, 1))
    mypart.seedEdgeBySize(constraint=FINER, deviationFactor=0.1,
                          edges=mypart.edges.findAt((s_pt_mesh_edge,), ), size=meshsize/z_density)
    mypart.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=meshsize)
    mypart.generateMesh()


def load_bc(mymodel, mypart, myassembly, step_name, load_name, bc_name,
            r_out, width, r_depth, r_pressure, load, s_pt_load, s_pt_bc):
    mypart.Set(faces=mypart.faces.findAt((s_pt_load,), ), name='face_load')
    face_load = mypart.sets['face_load'].faces[0]
    mypart.Set(nodes=face_load.getNodes(), name='face_load_nodes')
    face_load_nodes = mypart.sets['face_load_nodes'].nodes
    mypart.Set(nodes=face_load_nodes.getByBoundingCylinder(center1=(0.0, r_out - r_depth, width / 2),
                                                           center2=(0.0, r_out + r_depth, width / 2),
                                                           radius=r_pressure), name='nodes_load')
    nodes_load = mypart.sets['nodes_load'].nodes

    mypart.Set(faces=mypart.faces.findAt((s_pt_bc,), ), name='face_bc')
    face_bc = mypart.sets['face_bc'].faces[0]
    mypart.Set(nodes=face_bc.getNodes(), name='face_bc_nodes')
    face_bc_nodes = mypart.sets['face_bc_nodes'].nodes
    mypart.Set(nodes=face_bc_nodes.getByBoundingCylinder(center1=(0.0, -(r_out - r_depth), width / 2),
                                                         center2=(0.0, -(r_out + r_depth), width / 2),
                                                         radius=r_pressure), name='nodes_bc')
    nodes_bc = mypart.sets['nodes_bc'].nodes
    num_nodes_load = len(mypart.sets['nodes_load'].nodes)
    mymodel.ConcentratedForce(cf2=-load / num_nodes_load, createStepName=step_name,
                              distributionType=UNIFORM, field='', localCsys=None, name=load_name,
                              region=myassembly.sets['nodes_load'])
    mymodel.EncastreBC(createStepName=step_name, localCsys=None, name=bc_name, region=myassembly.sets['nodes_bc'])


def job(job_name):
    mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, explicitPrecision=SINGLE,
            getMemoryFromAnalysis=True, historyPrint=OFF, memory=90, memoryUnits=PERCENTAGE,
            model='Model-1', modelPrint=OFF, multiprocessingMode=DEFAULT, name=job_name,
            nodalOutputPrecision=SINGLE, numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='',
            type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
    mdb.jobs[job_name].submit(consistencyChecking=OFF)


def get_nodal_S(index, field):
    nodalS = {}
    for value in field.values:
        if value.nodeLabel in nodalS:
            nodalS[value.nodeLabel].append(value.data[index])
        else:
            nodalS.update({value.nodeLabel: [value.data[index]]})
    for key in nodalS:
        nodalS.update({key: sum(nodalS[key]) / len(nodalS[key])})
    return nodalS


def get_nodal_U(index, elemDisp):
    nodalU = {}
    for value in elemDisp.values:
        nodalU.update({value.nodeLabel: value.data[index] * 1000})
    return nodalU


def post_process(job_name):
    odb_name = job_name + '.odb'
    odb = openOdb(path=odb_name, readOnly=True)
    odb_assembly = odb.rootAssembly
    odb_step1 = odb.steps.values()[0]
    frame = odb.steps[odb_step1.name].frames[-1]
    elemStress = frame.fieldOutputs['S']
    elemDisp = frame.fieldOutputs['U']
    odb_set_whole = odb_assembly.elementSets[' ALL ELEMENTS']
    field = elemStress.getSubset(region=odb_set_whole, position=ELEMENT_NODAL)

    nodalS11 = get_nodal_S(0, field)
    nodalS22 = get_nodal_S(1, field)
    nodalS33 = get_nodal_S(2, field)
    nodalS12 = get_nodal_S(3, field)
    nodalS13 = get_nodal_S(4, field)
    nodalS23 = get_nodal_S(5, field)
    nodalU1 = get_nodal_U(0, elemDisp)
    nodalU2 = get_nodal_U(1, elemDisp)
    nodalU3 = get_nodal_U(2, elemDisp)

    # nodal_mises = {}
    # for value in field.values:
    #     nodal_mises.update({value.nodeLabel: value.mises})
    nodal_all = [nodalU1, nodalU2, nodalU3, nodalS11, nodalS22, nodalS33, nodalS12, nodalS13, nodalS23]
    nodalUS = nodalS11.copy()
    for key in nodalUS:
        nodalUS[key] = []
    for nodal_set in nodal_all:
        for key, value in nodal_set.items():
            nodalUS[key].append(value)
    return nodalUS


def output_csv(mypart, results_location, nodalUS, filename):
    # Exterior nodes
    node_object_external = mypart.sets['all_faces'].nodes
    node_labels_external = [node.label for node in node_object_external]
    node_object_load = mypart.sets['nodes_load'].nodes
    # node_labels_load = [node.label for node in node_object_load]
    node_object_bc = mypart.sets['nodes_bc'].nodes
    # node_labels_bc = [node.label for node in node_object_bc]

    # Print_result
    with open(results_location + filename + '_nodes.csv', 'w') as f:
        f.write('nodeid,nodetype,x,y,z,U1,U2,U3,S11,S22,S33,S12,S13,S12\n')
        for nodeid, component in nodalUS.items():
            meshnode_object = mypart.nodes[nodeid - 1]
            x, y, z = meshnode_object.coordinates[0] * 1000, meshnode_object.coordinates[1] * 1000, meshnode_object.coordinates[2] * 1000
            if nodeid in node_labels_external:
                nodetype = 1
            else:
                nodetype = 0
            f.write('%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n' % (nodeid, nodetype, x, y, z, component[0], component[1], component[2], component[3], component[4], component[5], component[6], component[7], component[8]))


    with open(results_location + filename + '_elements.csv', 'w') as f:
        f.write('elementid,node1,node2,node3,node4\n')
        for element in mypart.elements:
            f.write('%d,%d,%d,%d,%d\n' % (element.label, element.connectivity[0], element.connectivity[1],
                                          element.connectivity[2], element.connectivity[3]))
