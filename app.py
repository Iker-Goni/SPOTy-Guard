from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('index.html')


# ToDO: Integrate with SPOTyGuard Backend
@app.route('/facereg-<status>')
def facereg(status):
    if status == "enable":
        print("waow")
    elif status == "disable":
        print("waow 2")
    return redirect("/")
# ToDO: Integrate with SPOTyGuard Backend
@app.route("/patrol-<status>")
def patrol(status):
    if status == "enable":
        print("wowowo")
    elif status == "disable":
        print("wowowo 2")
    return redirect("/")



if __name__ == '__main__':
    app.run(debug=true)