#!/usr/bin/env python

"""URDF files to Webots PROTO converter."""


import os
import errno
import optparse
import re
import sys
import urdf2webots.parserURDF
import urdf2webots.writeProto
from xml.dom import minidom

try:
    import rospkg
except ImportError:
    pass


def convertLUtoUN(s):
    """Capitalize a string."""
    r = ''
    i = 0
    while i < len(s):
        if i == 0:
            r += s[i].upper()
            i += 1
        elif s[i] == '_' and i < (len(s) - 1):
            r += s[i + 1].upper()
            i += 2
        else:
            r += s[i]
            i += 1
    return r


def mkdirSafe(directory):
    """Create a dir safely."""
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print('Directory "' + directory + '" already exists!')


def convert2urdf(inFile, outFile=None, normal=False, boxCollision=False, disableMeshOptimization=False):
    if not os.path.exists(inFile):
        sys.exit('Input file "%s" does not exists.' % inFile)

    urdf2webots.parserURDF.disableMeshOptimization = disableMeshOptimization

    with open(inFile, 'r') as file:
        inPath = os.path.dirname(os.path.abspath(inFile))
        content = file.read()
        packages = re.findall('"package://(.*)"', content)
        if packages:
            packageName = packages[0].split('/')[0]
            directory = os.path.dirname(inFile)
            while packageName != os.path.split(directory)[1] and os.path.split(directory)[1]:
                directory = os.path.dirname(directory)
            if not os.path.split(directory)[1]:
                try:
                    rospack = rospkg.RosPack()
                    directory = rospack.get_path(packageName)
                except rospkg.common.ResourceNotFound:
                    sys.stderr.write('Package "%s" not found.\n' % packageName)
                except NameError:
                    sys.stderr.write('Impossible to find location of "%s" package, installing "rospkg" might help.\n'
                                     % packageName)
            if os.path.split(directory)[1]:
                packagePath = os.path.split(directory)[0]
                content = content.replace('package:/', packagePath)
            else:
                sys.stderr.write('Can\'t determine package root path.\n')

        domFile = minidom.parseString(content)

        for child in domFile.childNodes:
            if child.localName == 'robot':
                robotName = convertLUtoUN(urdf2webots.parserURDF.getRobotName(child))  # capitalize
                outputFile = outFile if outFile else robotName + '.proto'

                urdf2webots.parserURDF.robotName = robotName  # pass robotName
                mkdirSafe(outputFile.replace('.proto', '') + '_textures')  # make a dir called 'x_textures'

                robot = child
                protoFile = open(outputFile, 'w')
                urdf2webots.writeProto.header(protoFile, inFile, robotName)
                linkElementList = []
                jointElementList = []
                for child in robot.childNodes:
                    if child.localName == 'link':
                        linkElementList.append(child)
                    elif child.localName == 'joint':
                        jointElementList.append(child)
                    elif child.localName == 'material':
                        if not child.hasAttribute('name') \
                           or child.getAttribute('name') not in urdf2webots.parserURDF.Material.namedMaterial:
                            material = urdf2webots.parserURDF.Material()
                            material.parseFromMaterialNode(child)

                linkList = []
                jointList = []
                parentList = []
                childList = []
                rootLink = urdf2webots.parserURDF.Link()

                for joint in jointElementList:
                    jointList.append(urdf2webots.parserURDF.getJoint(joint))
                    parentList.append(jointList[-1].parent)
                    childList.append(jointList[-1].child)
                parentList.sort()
                childList.sort()
                for link in linkElementList:
                    linkList.append(urdf2webots.parserURDF.getLink(link, inPath))
                for link in linkList:
                    if urdf2webots.parserURDF.isRootLink(link.name, childList):
                        rootLink = link
                        # if root link has only one joint which type is fixed,
                        # it should not be part of the model (link between robot and static environment)
                        while True:
                            directJoint = []
                            found = False  # To avoid endless loop
                            for joint in jointList:
                                if joint.parent == rootLink.name:
                                    directJoint.append(joint)
                            if len(directJoint) == 1 and directJoint[0].type == 'fixed':
                                for childLink in linkList:
                                    if childLink.name == directJoint[0].child:
                                        rootLink = childLink
                                        found = True
                                        break
                            else:
                                break
                            if not found:
                                break
                        print('Root link: ' + rootLink.name)
                        break

                for child in robot.childNodes:
                    if child.localName == 'gazebo':
                        urdf2webots.parserURDF.parseGazeboElement(child, rootLink.name, linkList)

                sensorList = (urdf2webots.parserURDF.IMU.list +
                              urdf2webots.parserURDF.Camera.list +
                              urdf2webots.parserURDF.Lidar.list)
                print('There are %d links, %d joints and %d sensors' % (len(linkList), len(jointList), len(sensorList)))

                urdf2webots.writeProto.declaration(protoFile, robotName)
                urdf2webots.writeProto.URDFLink(protoFile, rootLink, 1, parentList, childList, linkList, jointList,
                                                sensorList, boxCollision=boxCollision, normal=normal, robot=True)
                protoFile.write('}\n')
                protoFile.close()
                return
    print('Could not read file')


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog --input=my_robot.urdf [options]')
    optParser.add_option('--input', dest='inFile', default='', help='Specifies the urdf file to convert.')
    optParser.add_option('--output', dest='outFile', default='', help='Specifies the name of the resulting PROTO file.')
    optParser.add_option('--normal', dest='normal', action='store_true', default=False,
                         help='If set, the normals are exported if present in the URDF definition.')
    optParser.add_option('--box-collision', dest='boxCollision', action='store_true', default=False,
                         help='If set, the bounding objects are approximated using boxes.')
    optParser.add_option('--disable-mesh-optimization', dest='disableMeshOptimization', action='store_true', default=False,
                         help='If set, the duplicated vertices are not removed from the meshes (this can speed up a lot the '
                         'conversion).')
    options, args = optParser.parse_args()

    convert2urdf(options.inFile, options.outFile, options.normal, options.boxCollision, options.disableMeshOptimization)
