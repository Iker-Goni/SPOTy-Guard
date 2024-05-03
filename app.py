from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify, request, flash
import SpotController
app = Flask(__name__)

# facereg_status = "disable"
patrol_status = "disable"

# facereg_color = "red"
patrol_color = "red"

rc = SpotController.SpotController()


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

@app.route("/testbark")
def testbark():
    rc.bark()
    return redirect("/")

@app.route("/face-menu")
def face_menu():
    return render_template('facereg.html')
@app.route("/addface", methods=['GET', 'POST'])
def addFace():
    name = request.form.get('name')
    print(request.form)
    rc.scanNewFace(person_name=request.form.get("name"))
    #flash("Face successfully added!")
    return redirect("/")

@app.route("/recognize")
def recognize():
    rc.patrolRecognize()
    return redirect("/")


if __name__ == '__main__':
    # run flask server
    app.run(debug=True, host="0.0.0.0")
