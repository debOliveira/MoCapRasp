import click 

from mcr.capture.CEC import CEC 
from mcr.capture.GPE import GPE
from mcr.capture.SCR import SCR 

@click.group()
def mocaprasp():
    '''
    MoCap Rasp - Optical Tracking Arena\n\n
    Server script for the MoCap system at the Erobotica Lab, UFCG.\n
    Please use it together with the corresponding client script.
    '''
    pass

@click.command(name='cec')
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 10,        help = 'Trigger time in seconds (Default: 10)')
@click.option('--record',    '-r', default = 120,       help = 'Recording time in seconds (Default: 120)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def cec(cameraids, markers, trigger, record, fps, verbose, save):
    '''
    Camera Extrinsics Calibration\n\n
    - Place 3 collinear markers in the calibration wand;\n
    - Make sure to move it slowly;\n
    - Show it to each adjacent pair of cameras.\n\n
    The default options are already adjusted for this process.
    '''
    cecServer = CEC(cameraids, markers, trigger, record, fps, verbose, save)
    cecServer.connect()
    cecServer.collect() 

@click.command(name='gpe')
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 2,         help = 'Trigger time in seconds (Default: 2)')
@click.option('--record',    '-r', default = 10,        help = 'Recording time in seconds (Default: 10)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def gpe(cameraids, markers, trigger, record, fps, verbose, save):
    '''
    Ground Plane Estimation\n\n
    - Place 3 non-collinear markers in the calibration wand;\n
    - Put it at the center of the capture volume;\n
    - Make sure they are levelled with each other.\n\n
    The default options are already adjusted for this process.
    '''
    gpeServer = GPE(cameraids, markers, trigger, record, fps, verbose, save)
    gpeServer.connect()
    gpeServer.collect()

@click.command(name='scr')
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 5,         help = 'Trigger time in seconds (Default: 5)')
@click.option('--record',    '-r', default = 30,        help = 'Recording time in seconds (Default: 30)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def scr(cameraids, markers, trigger, record, fps, verbose, save):
    '''
    Standard Capture Routine\n\n
    - Routines required to be done previously at least once:\n
        1. CEC;\n
        2. GPE.\n
    - With CEC and GPE routines done, execute SCR as much as you like;\n\n
    Adjust the options to match your desired capture.
    '''
    scrServer = SCR(cameraids, markers, trigger, record, fps, verbose, save)
    scrServer.connect()
    scrServer.collect()

mocaprasp.add_command(cec)
mocaprasp.add_command(scr)
mocaprasp.add_command(gpe)