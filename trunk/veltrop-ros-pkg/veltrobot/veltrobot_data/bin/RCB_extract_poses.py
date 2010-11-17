#!/usr/bin/python
# Note: This assumes that all RCB files will present their parameters in the same order.
#       If the order needs to be dynamic, then the main loop needs to be modified.
# TODO?: command line option to set joint names per servo id? (currently config file)

# config file for RCB Servo ID# -> ROS joint name: 
#  pkgs/roboard/roboard_pose/conf/RCB_extract_servo_joint_map.yaml

import sys
import os.path
import yaml
from optparse import OptionParser
import roslib.packages

# Servo value in RCB file is a positive/negative offset from an externally defined home position.
# The value stored in the RCB file is the value seen in the Heart2Heart GUI + 16384

# Max/Mins:

# PWM  min | center | max               =  700     | 1500 | 2300
# RCB  min | center | max for above PWM = -260     | 0    | 260
# 788  min | center | max in degrees    = -90      | 0    | 90
# 788  min | center | max in radians    = -PI/2    | 0    | PI/2
# 4024 min | center | max in degrees    = -130     | 0    | 130
# 4024 min | center | max in radians    = -13PI/18 | 0    | 13PI/18

# Scaling: 

# Scale RCB value of 788  to Degrees offset =  (90)          / 260 = 0.346153846154
# Scale RCB value of 788  to Radians offset =  (90PI / 180)  / 260 = 0.006041524334
# Scale RCB value of 788  to PWM     offset =  (800)         / 260 = 3.07692307692
# Scale RCB value of 4024 to Degrees offset =  (130)         / 260 = 0.5
# Scale RCB value of 4024 to Radians offset =  (130PI / 180) / 260 = 0.00872664626
# Scale RCB value of 4024 to PWM     offset =  (800)         / 260 = 3.07692307692

# PWM scale defailt:
#default="3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692,3.07692",
# Radian scale default:
#default="0.00604,0.00873,0.00604,0.00604,0.00604,0.00873,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604",

# set up command line options, defaults
usage = "usage: %prog [options] INPUTFILE.RCB"
version = "%prog 1.0"
parser = OptionParser(usage)
parser.add_option("-c", "--center", dest="center", default=16384, type="int",
                  help="subtract CENTER from input servo values"
                  " [default:%default]")  
parser.add_option("-s", "--scale", dest="scale", type="string",
default="0.00604,0.00873,0.00604,0.00604,0.00604,0.00873,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604,0.00604",
                  help="multiply input servo values by SCALE after subtracting center"
                  " [default:%default]")                  
parser.add_option("-u", "--units", dest="units", default="radians_offset", type="choice",
                  choices=["pwm_offset", "pwm", "degrees_offset", "radians_offset"],
                  help="units parameter for output file "
                  "(pwm_offset, pwm, degrees_offset, radians_offset) [default:%default]")  
parser.add_option("-t", "--timescale", dest="timescale", default=15.0, type="float",
                  help="multiply input time by TIMESCALE [default:%default]")                  
parser.add_option("-n", "--namebase", dest="namebase", type="string",
                  help="set pose name to [NAMEBASE + name of pose from RCB]"
                  " [default:input file name + name of pose]")                                   
parser.add_option("-o", "--outputbase", dest="outputbase", type="string",
                  help="write output file to alternate [OUTPUTBASE + name of pose from RCB.xml]"
                  " [default:name base + name of pose.xml]")
(options, args) = parser.parse_args()

if len(args) != 1:
    parser.error("incorrect number of arguments")

scales = options.scale.split(",")

# conf file for mapping
#servo_joint_map_filename = os.environ.get("ROS_ROOT") + "/../pkgs/veltrop-ros-pkg/veltrobot/veltrobot_data/conf/RCB_extract_servo_joint_map.yaml"
servo_joint_map_filename = roslib.packages.get_pkg_dir('veltrobot_data') + "/conf/RCB_extract_servo_joint_map.yaml"
servo_joint_map = yaml.load(file(servo_joint_map_filename, 'r'))

# prepare input file
this_file = os.path.basename(sys.argv[0])
if not os.path.exists(args[0]) or not os.path.isfile(args[0]):
    print this_file + ": Input File Not Found, Aborting."
    sys.exit()

input_file = open(args[0], "r")
if not input_file:
    print this_file + ": Error Opening Input File, Aborting."
    sys.exit() 
input_file_basename = os.path.splitext(os.path.basename(args[0]))[0]

line = input_file.readline().strip("\n")
if not line:
    print this_file + ": Input File Empty, Aborting."
    input_file.close()
    sys.exit()  

# check if input file is trim data
if line.find("[TrimData]") >= 0:
    # figure out names, output file
    pose_name = ""
    if not options.namebase:
        pose_name = input_file_basename
    else:
        pose_name = options.namebase
    output_filename = ""
    if not options.outputbase:
        output_filename = pose_name + ".xml"
    else:
        output_filename = options.outputbase + ".xml"
    output_file = open(output_filename, "w")
    if not output_file:
        print this_file + ": Error Opening Output File, Aborting."
        input_file.close()
        sys.exit()    
    
    output_file.writelines("<?xml version=\"1.0\" ?>\n\n")
    output_file.writelines("<pose name=\"" + pose_name + "\">\n")
    output_file.writelines("\t<joint_positions duration=\"0\" units=\"" + options.units + "\">\n")
    
    # parse the trim pose  
    while 1:
        # advance to next line
        line = input_file.readline().strip("\n\r")
        if not line:
            break  
            
        # parse servo name and value, convert it, output it
        curr = line.split("=")
        servo_id = int(curr[0].strip("CH"));
        prm = curr[1]
        prm_float = (float(prm) - float(options.center)) * float(scales[servo_id-1])  
        if "_" + str(servo_id) in servo_joint_map:
            joint_name = servo_joint_map["_" + str(servo_id)]
            output_file.writelines("\t\t<joint name=\"" + joint_name + "\"  position=\"" + str(prm_float) + "\" />\n")
        
    output_file.writelines("\t</joint_positions>\n")
    output_file.writelines("</pose>\n\n")        
    output_file.close()
    
# check if input file is legit rcb file
elif not line.find("[GraphicalEdit]") >= 0:
    print this_file + ": Input File Not Recognized, Aborting."
    input_file.close()
    sys.exit()   

# input looks like a legit pose, parse it
curr_name = ""
curr_names = {" ": 1}
while 1:
    # advance to next line
    line = input_file.readline().strip("\n")
    if not line:
        break      
    
    # if entered a new item, lets get its name and check it out
    if line.find("[Item") >= 0:
        line = input_file.readline().strip("\n\r")
        if not line.find("Name=") >= 0:
            continue       
        curr_name = line.split("=")[1]
        
        # dont overwrite duplicate position names
        appendNum = 1
        while curr_name in curr_names:
            curr_name = curr_name + str(appendNum)
            appendNum = appendNum + 1
        curr_names[curr_name] = 1
                    
        # skip ahead to the type
        line = input_file.readline().strip("\n\r")
        line = input_file.readline().strip("\n\r")
        line = input_file.readline().strip("\n\r")
        line = input_file.readline().strip("\n\r")
        line = input_file.readline().strip("\n\r")
        line = input_file.readline().strip("\n\r")
        
        # if entered the right type item, get its prm, make the output
        if line.find("Type=0") >= 0:
            line = input_file.readline().strip("\n\r")
            if not line.find("Prm=") >= 0:
                continue
            prms = line.strip("Prm=").split(",")
            if not len(prms):
                continue
            
            # figure out names, outputfile
            pose_name = ""
            if not options.namebase:
                pose_name = input_file_basename + "_" + curr_name
            else:
                pose_name = options.namebase + "_" + curr_name
            output_filename = ""
            if not options.outputbase:
                output_filename = pose_name + ".xml"
            else:
                output_filename = options.outputbase + ".xml"
            output_file = open(output_filename, "w")
            if not output_file:
                print this_file + ": Error Opening Output File, Aborting."
                input_file.close()
                sys.exit()             
            
            # finally write the file
            output_file.writelines("<?xml version=\"1.0\" ?>\n\n")
            output_file.writelines("<pose name=\"" + pose_name + "\">\n")
            output_file.writelines("\t<joint_positions duration=\"" + str(int(float(prms[0]) * options.timescale)) + "\" units=\"" + options.units + "\">\n")
            prms.pop(0)
            servo_id = 1;
            for prm in prms:
                # apply unit/name conversions and output it
                prm_float = (float(prm) - float(options.center)) * float(scales[servo_id-1])
                if "_" + str(servo_id) in servo_joint_map:
                    joint_name = servo_joint_map["_" + str(servo_id)]
                    output_file.writelines("\t\t<joint name=\"" + joint_name + "\"  position=\"" + str(prm_float) + "\" />\n")
                servo_id = servo_id + 1
            output_file.writelines("\t</joint_positions>\n")
            output_file.writelines("</pose>\n\n")
            output_file.close()
        
input_file.close()

