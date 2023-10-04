import click, CEC, SCR, GPE

@click.group()
def mocaprasp():
    """
    Server for the MoCap system at Erobotica lab of UFCG.
    Please use it together with the corresponding client script.
    """
    pass

@click.command(name="cec")
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 10,        help = 'Trigger time in seconds (Default: 10)')
@click.option('--record',    '-r', default = 120,       help = 'Recording time in seconds (Default: 120)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def cec(cameraids, markers, trigger, record, fps, verbose, save):
    """
    CEC: Camera Extrinsic Calibration
    """
    cecServer = CEC(cameraids, markers, trigger, record, fps, verbose, save)
    cecServer.connect()
    cecServer.collect() 

@click.command(name="gpe")
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 2,         help = 'Trigger time in seconds (Default: 2)')
@click.option('--record',    '-r', default = 10,        help = 'Recording time in seconds (Default: 10)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def gpe(cameraids, markers, trigger, record, fps, verbose, save):
    """
    GPE: Ground Plane Estimation
    """
    gpeServer = GPE(cameraids, markers, trigger, record, fps, verbose, save)
    gpeServer.connect()
    gpeServer.collect()

@click.command(name="scr")
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 4,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 5,         help = 'Trigger time in seconds (Default: 5)')
@click.option('--record',    '-r', default = 30,        help = 'Recording time in seconds (Default: 30)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def scr(cameraids, markers, trigger, record, fps, verbose, save):
    """
    SCR: Standard Capture Routine
    """
    scrServer = SCR(cameraids, markers, trigger, record, fps, verbose, save)
    scrServer.connect()
    scrServer.collect()

mocaprasp.add_command(cec)
mocaprasp.add_command(scr)
mocaprasp.add_command(gpe)