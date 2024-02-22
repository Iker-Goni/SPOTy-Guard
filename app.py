from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify

app = Flask(__name__)

facereg_status = "disable"
patrol_status = "disable"

facereg_color = "red"
patrol_color = "red"

@app.route('/')
def home():
    return render_template('index.html', facereg=facereg_status, patrol=patrol_status, facereg_color=facereg_color, patrol_color=patrol_color)


# ToDO: Integrate with SPOTyGuard Backend
@app.route('/facereg-<status>')
def facereg(status):
    global facereg_status, facereg_color
    facereg_status = status
    if status == "enable":
        facereg_color = "green"
    elif status == "disable":
        facereg_color = "red"
    return redirect("/")
# ToDO: Integrate with SPOTyGuard Backend
@app.route("/patrol-<status>")
def patrol(status):
    global patrol_status, patrol_color
    patrol_status = status
    if status == "enable":
        patrol_color = "green"
    elif status == "disable":
        patrol_color = "red"
    return redirect("/")



if __name__ == '__main__':
    app.run(debug=true)
