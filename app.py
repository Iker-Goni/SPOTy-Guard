from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify
import SpotController
app = Flask(__name__)

# facereg_status = "disable"
patrol_status = "disable"

# facereg_color = "red"
patrol_color = "red"

rc = None

@app.route('/')
def home():
    # return render_template('index.html', facereg=facereg_status, patrol=patrol_status, facereg_color=facereg_color, patrol_color=patrol_color)
    return render_template('index.html', patrol=patrol_status, patrol_color=patrol_color)


# # TODO: Integrate with SPOTyGuard Backend
# @app.route('/facereg-<status>')
# def facereg(status):
#     global facereg_status, facereg_color
#     facereg_status = status
#     if status == "enable":
#         facereg_color = "green"
#     elif status == "disable":
#         facereg_color = "red"
#     return redirect("/")
# TDO: Integrate with SPOTyGuard Backend
@app.route("/patrol-<status>")
def patrol(status):
    global patrol_status, patrol_color
    patrol_status = status
    if status == "enable":
        rc.enablePatrol()
        patrol_color = "green"
    elif status == "disable":
        rc.disablePatrol()
        patrol_color = "red"
    return redirect("/")

@app.route("/gripper-<status>")
def gripper(status):
    global robot
    if status == "open":
        print("do nothing")
    elif status == "close":
        print("do nothing")
    return redirect("/")

@app.route("/estop")
def estop():
    rc.estop()
    return redirect("/")
@app.route("/unestop")
def unestop():
    rc.unestop()
    return redirect("/")

@app.route("/testnotifs")
def notify():
    print("do something lol")
    return redirect("/")

@app.route("/addface")
def addface():
    rc.scanNewFace()
    return redirect("/")
if __name__ == '__main__':
    rc = SpotController.SpotController()
    # run flask server
    app.run(debug=True)
