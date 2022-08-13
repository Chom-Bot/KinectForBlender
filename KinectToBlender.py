bl_info = {
    "name": "Motion from Azure Kinect",
    "blender": (2, 80, 0),
    "category": "Object",
}
from os import close
import bpy
import bpy_extras.io_utils
import json
import mathutils
import numpy
from bpy.types import Armature, EditBone, Pose, PoseBone

class ReadLocationsFromFile(bpy.types.Operator,bpy_extras.io_utils.ImportHelper):
    """Capture Motion From Mocap Text File."""      # Use this as a tooltip for menu items and buttons.
    bl_idname = "armature.position_bones"        # Unique identifier for buttons and menu items to reference.
    bl_label = "Reposition Armature Bones"         # Display name in the interface.
    bl_options = {'REGISTER', 'UNDO'}  # Enable undo for the operator.

    #json format for bone info: {"Timestamp":double,"Name":string,"X":double,"Y":double,"Z":double,"W":double,"XLoc":double,"YLoc":double,"ZLoc":double}


    def execute(self, context:bpy.context):        # execute() is called when running the operator.
        mode = 'SimpleArmature'
        bonedict = {}
        if mode == 'SimpleArmature':
            bonedict = {
            "Pelvis": "spine",
            "SpineNavel": "spine.002",
            "SpineChest": "spine.003",
            "Neck": "spine.004",
            "ClavicleLeft":"shoulder.L",
            "ClavicleRight":"shoulder.R",
            "ShoulderLeft":"upper_arm.L",
            "ShoulderRight":"upper_arm.R",
            "ElbowLeft":"forearm.L",
            "ElbowRight":"forearm.R",
            "WristLeft":"hand.L",
            "WristRight":"hand.R",
            "HandLeft":"hand.L",
            "HandRight":"hand.R",
            "HandTipLeft":"hand.L",
            "HandTipRight":"hand.R",
            "ThumbLeft":"thumb.L",
            "ThumbRight":"thumb.R",
            "HipLeft":"thigh.L",
            "HipRight":"thigh.R",
            "KneeLeft":"shin.L",
            "KneeRight":"shin.R",
            "AnkleLeft":"foot.L",
            "AnkleRight":"foot.R",
            "FootLeft":"toe.L",
            "FootRight":"toe.R",
            "Head":"spine.006",
            "Nose":"spine.006",
            "EyeLeft":"toe.L",
            "EyeRight":"toe.L",
            "EarLeft":"toe.L",
            "EarRight":"toe.L"
            }
        else:
            bonedict = {}
        inputfilepath :str = self.filepath
        print(inputfilepath+" opening")
        scene = context.scene
        armature = scene.objects['armature']
        armature.select_set(True)
        
        bpy.ops.object.mode_set(mode = 'POSE')
        scale = 1.0
        file = open(inputfilepath)
        i = 0
        bonelist:list[list[str,float,float,float,float,float,float,str]] = [] #'list[[str,float,float,float,float,float,float]]' name, headx, heady, headz, tailx, taily,tailz
        parent = 'POSE'
        while i < 32:
                
                bonejsonline:str = file.readline()
                bone = json.loads(bonejsonline)
                pbone = armature.pose.bones[bonedict[bone['Name']]]
                if bone['Name'] != 'Pelvis':
                    parent = pbone.parent.name
                x_pos = float(bone['XLoc'])*scale
                y_pos = float(bone['YLoc'])*scale
                z_pos = float(bone['ZLoc'])*scale
                head =  [x_pos,y_pos,z_pos]
                if not (bone['Name'] == 'Nose') and not(bone['Name'] == 'EyeLeft') and not( bone['Name'] == 'EyeRight') and not( bone['Name'] == 'EarLeft') and not( bone['Name'] == 'EarRight'):
                   bone_entry = [pbone.name, head[0],head[1],head[2],0,0,0, parent]
                   bonelist.append(bone_entry)
                i+=1
        for bone_entry in bonelist:     # ensure tail positions are correct
            if bone_entry[0] != 'spine':
                parent_locations = get_bone(bone_entry[7], bonelist)
                bone_entry[4] = parent_locations[1]
                bone_entry[5] = parent_locations[2]
                bone_entry[6] = parent_locations[3]
                print(bone_entry)
        for bone_location in bonelist:
            if bone_location[0] != 'spine':
                pbone = armature.pose.bones[bone_location[7]]
                vector_base = [0.0,0.0,1.0]
                tail_vector = bone_location[1:4]
                print(tail_vector)
                head_vector = bone_location[4:7]
                print(head_vector)
                vector_bone = bone_vector(head_vector, tail_vector)
                pbone.rotation_mode = 'AXIS_ANGLE'
                pbone.rotation_axis_angle = axis_angle_from_vector(vector_bone)
        file.close()
        return {'FINISHED'}            # Lets Blender know the operator finished successfully.

def menu_func(self, context:bpy.context):
    self.layout.operator(ReadLocationsFromFile.bl_idname)

def read_line(line :str):
    values = line.split(' ')
    

def register():
    bpy.utils.register_class(ReadLocationsFromFile)
    bpy.types.VIEW3D_MT_object.append(menu_func)  # Adds the new operator to an existing menu.

def unregister():
    bpy.utils.unregister_class(ReadLocationsFromFile)

def get_bone(name:str, bonelist:'list[list[str,float,float,float,float,float,float,str]]'):
    for bone in bonelist:
        if bone[0] == name:
            return bone
    return None


def axis_angle_from_vector(vector:'list[float]'): #vector from bone_vector, tail - head
    vector_base = [0,0,1]
    axis = numpy.cross(vector_base,vector)
    angle = numpy.arccos(numpy.dot(vector_base,vector)/(vector_length(vector)*vector_length(vector_base)))
    return [angle, axis[0],axis[1],axis[2]]   #object space rotation -> bone space rotation in blender [w,x,y,z]->[w,-x,z,y]

def vector_length(vector:'list[float]'):
    horizontal = numpy.sqrt(numpy.power(vector[0],2)+vector[1])
    length = numpy.sqrt(vector[2]*vector[2] + horizontal*horizontal)
    return length

def bone_vector(head:'list[float]', tail:'list[float]'):
    output = [tail[0]-head[0],tail[1]-head[1],tail[2]-head[2]]
    return output

def old_execute(self, context:bpy.context):        # converts bone LOCATIONS in edit mode from input file. Exploratory use only, cannot be used for keyframe animation due to limitations on keyframing.
        mode = 'SimpleArmature'
        bonedict = {}
        if mode == 'SimpleArmature':
            bonedict = {
            "Pelvis": "spine",
            "SpineNavel": "spine.002",
            "SpineChest": "spine.003",
            "Neck": "spine.004",
            "ClavicleLeft":"shoulder.L",
            "ClavicleRight":"shoulder.R",
            "ShoulderLeft":"upper_arm.L",
            "ShoulderRight":"upper_arm.R",
            "ElbowLeft":"forearm.L",
            "ElbowRight":"forearm.R",
            "WristLeft":"hand.L",
            "WristRight":"hand.R",
            "HandLeft":"hand.L",
            "HandRight":"hand.R",
            "HandTipLeft":"hand.L",
            "HandTipRight":"hand.R",
            "ThumbLeft":"hand.L",
            "ThumbRight":"hand.R",
            "HipLeft":"thigh.L",
            "HipRight":"thigh.R",
            "KneeLeft":"shin.L",
            "KneeRight":"shin.R",
            "AnkleLeft":"foot.L",
            "AnkleRight":"foot.R",
            "FootLeft":"toe.L",
            "FootRight":"toe.R",
            "Head":"spine.006",
            "Nose":"spine.006",
            "EyeLeft":"toe.L",
            "EyeRight":"toe.L",
            "EarLeft":"toe.L",
            "EarRight":"toe.L"
            }
        else:
            bonedict = {}
        inputfilepath :str = self.filepath
        print(inputfilepath+" opening")
        scene = context.scene
        armature = scene.objects['armature']
        armature.select_set(True)
        #armature = bpy.context.object

        bpy.ops.object.mode_set(mode = 'EDIT')
        scale = 1.0
        file = open(inputfilepath)
        previous_quaternion = [1,0,0,0]
        i = 0
        while i < 32:
                bonejsonline:str = file.readline()
                bone = json.loads(bonejsonline)
                hbone = armature.data.edit_bones[bonedict[bone['Name']]]
                if not (bone['Name'] == 'Nose') and not(bone['Name'] == 'EyeLeft') and not( bone['Name'] == 'EyeRight') and not( bone['Name'] == 'EarLeft') and not( bone['Name'] == 'EarRight'):
                    hbone.head  = [float(bone['XLoc'])*scale ,float(bone['YLoc'])*scale,float(bone['ZLoc'])*scale]
                    print(bonedict[bone['Name']])
                    if (bone['Name'] == 'FootLeft' or bone['Name'] =='FootRight' or bone['Name'] =='HandTipLeft' or bone['Name'] =='HandTipRight'):
                        hbone.tail  = [float(bone['XLoc'])*scale ,float(bone['YLoc'])*scale,float(bone['ZLoc'])*scale]
                if bone['Name'] == 'Nose':
                    hbone.tail  = [float(bone['XLoc'])*scale ,float(bone['YLoc'])*scale,float(bone['ZLoc'])*scale]


                i+=1
           #     continue
        file.close()
        return {'FINISHED'}            # Lets Blender know the operator finished successfully.

if __name__ == "__main__":
    register()
    #test: invert y and w on right leg
    #todo: adjust mocap data to scale of armature
    #todo: adjust armature to scale of mocap data
    #todo: adjust mocap data to account for floor/gravity orientation, likely in C# code using gyroscope/IMU





                    #    else :
                #        hbone.head  = [float(bone['XLoc'])*scale ,float(bone['YLoc'])*scale,float(bone['ZLoc'])*scale]  




                #hbone.rotation_mode = 'AXIS_ANGLE'
                #hbone.rotation_axis_angle = [float(bone['W']),float(bone['X']),float(bone['Y']),float(bone['Z'])]
                #hbone.rotation_mode = 'QUATERNION'
                #hbone.rotation_quaternion = [float(bone['W']),float(bone['X']),float(bone['Y']),float(bone['Z'])] #rotation is global while blender rotation is local. Requires transformation.
                #if bone['Name'] == 'HipRight': # or 'KneeRight' or 'AnkleRight' or 'FootRight'): #right leg coordinate space has inverted x and y 
                 #   euler_rot :mathutils.Euler = mathutils.Quaternion(hbone.rotation_quaternion).to_euler('XYZ')
                 #   euler_rot.z = euler_rot.z - 180
                 #   euler_rot.y = euler_rot.y - 180
                 #   hbone.rotation_quaternion = euler_rot.to_quaternion()
                 #   hbone.rotation_quaternion = [-float(bone['W']),float(bone['X']),float(bone['Y']),-float(bone['Z'])]
                    #hbone.rotation_quaternion = mathutils.Quaternion(hbone.rotation_quaternion).inverted() #rotation is global while blender rotation is local. Requires transformation.
                #parent_quaternion = [float(bone['PW']),float(bone['PX']),float(bone['PY']),float(bone['PZ'])]

                #if bone['Name'] != 'Pelvis':
                #    inverted = mathutils.Quaternion(parent_quaternion).inverted()
                #    hbone.rotation_quaternion = hbone.rotation_quaternion @ inverted #transform from global to local space by multiplying by inverse of parent transform
                   
                   # hbone.rotation_axis_angle = 
                    
                #    print(bone['Name'] + hbone.rotation_quaternion.__str__() +"hip inverted" + mathutils.Quaternion(hbone.rotation_quaternion).inverted().__str__() + " "+ inverted.__str__())
                        #rot_invert:mathutils.Quaternion = hbone.rotation_quaternion@ mathutils.Quaternion(hbone.rotation_quaternion).inverted()
                     #   hbone.rotation_quaternion = [float(bone['W']),float(bone['X']),float(bone['Y']),float(bone['Z'])]
                        #hbone.rotation_quaternion = hbone.rotation_quaternion @ inverted
                #debone = Matrix([[1,0,0,0],[0,0,-1,0], [0,1,0,0],[0,0,0,1]])
                #previous_quaternion = hbone.rotation_quaternion
                #hbone.rotation_quaternion = hbone.rotation_quaternion @ previous_quaternion
       #     except any as err:
       #         print(err)
           # finally: