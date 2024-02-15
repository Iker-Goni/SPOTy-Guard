from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=true)