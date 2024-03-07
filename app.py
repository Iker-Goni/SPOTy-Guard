from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify

app = Flask(__name__)

facereg_status = "disable"
patrol_status = "disable"

facereg_color = "red"
patrol_color = "red"

@app.route('/')
def home():
    return render_template('index.html', facereg=facereg_status, patrol=patrol_status, facereg_color=facereg_color, patrol_color=patrol_color)


# TODO: Integrate with SPOTyGuard Backend
@app.route('/facereg-<status>')
def facereg(status):
    global facereg_status, facereg_color
    facereg_status = status
    if status == "enable":
        facereg_color = "green"
    elif status == "disable":
        facereg_color = "red"
    return redirect("/")
# TDO: Integrate with SPOTyGuard Backend
@app.route("/patrol-<status>")
def patrol(status):
    global patrol_status, patrol_color
    patrol_status = status
    if status == "enable":
        patrol_color = "green"
    elif status == "disable":
        patrol_color = "red"
    return redirect("/")

@app.route("/gripper-<status>")
def gripper(status):
    global robot
    if status == "open":
        print("do nothing")
        #robot._start_robot_command('open_gripper', RobotCommandBuilder.claw_gripper_open_command())
    elif status == "close":
        print("do nothing")
        #robot._start_robot_command('close_gripper', RobotCommandBuilder.claw_gripper_close_command())

if __name__ == '__main__':
    #global robot, robot_command_client
    # parse arguments
    #parser = argparse.ArgumentParser()
    #bosdyn.client.util.add_base_arguments(parser)
    #options = parser.parse_args()
    # create robot object
    #sdk = create_standard_sdk('Spot Web Server')
    #robot = sdk.create_robot(options.hostname)
    #robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    # run flask server
    app.run(debug=True)
