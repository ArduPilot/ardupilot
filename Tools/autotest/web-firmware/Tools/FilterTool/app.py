import os
from flask import Flask
from flask import render_template

# A flask app to allow hosting filter tool locally

this_path = os.path.dirname(os.path.realpath(__file__))

app = Flask(__name__, template_folder=this_path, static_folder=this_path)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == "__main__":
    app.run()
